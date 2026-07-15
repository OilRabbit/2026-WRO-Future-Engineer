import cv2
import time
import datetime
from enum import Enum

from esp_com.communication import ESP32Communicator
from camera.camera_utils import (
	start_vision_system,
	start_web_server,
	get_latest_data,
	stop_vision_system,
	get_track_distance,
	set_marker_point,
	remove_marker_point,
	clear_marker_points,
	set_all_color_detection,
	configure_vision_pipeline,
)
from picamera2 import Picamera2 as picam2


print("======= Init =======")
esp = ESP32Communicator()
esp.connect()
camera = picam2()
video_size = (400, 225)
sensor_video_size = (2304, 1296)
target_frame_duration_us = 10000
config = camera.create_video_configuration(
	main={"size": video_size, "format": "BGR888"},
	sensor={"output_size": sensor_video_size},
	controls={"FrameDurationLimits": (target_frame_duration_us, target_frame_duration_us)},
	buffer_count=4,
	queue=False,
)
camera.configure(config)
camera.start()
print("Camera: Activated")

record_mp4 = True
live_streaming = True

configure_vision_pipeline(
	draw_overlays=True,
	show_debug_strip=False,
	stream_use_debug_frame=False,
	record_use_debug_frame=False,
	stream_jpeg_quality=70,
)

start_vision_system(camera, record_mp4)

if live_streaming:
	print("Streaming: Activated")
	start_web_server(host="0.0.0.0", port=5000)
else:
	print("Streaming: Not streaming")

print("======= End of Init =======")


def esp_replyNprint():
	try:
		reply = esp.read_message()
		if reply is not None:
			timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
			print(f"{timestamp} ESP: {reply}")
			return reply
	except Exception as exc:
		print(f"\n[SERIAL WARNING] {exc}")
	return None


last_action_logged = ""


def send_command_logged(cmd_str):
	global last_action_logged
	esp.send_command(cmd_str)
	if cmd_str != last_action_logged:
		print(f"[ESP_OUT] Sent Command: {cmd_str}")
		last_action_logged = cmd_str


def stop_vehicle(reason):
	print(f"[STOP] {reason}")
	send_command_logged("0, 0, 0, motor stop")


def add_marker_point(name, x, y, color=(0, 255, 255), radius=5, label=None):
	set_marker_point(name, x, y, color=color, radius=radius, label=label)


def clamp(value, minimum, maximum):
	return max(minimum, min(maximum, value))


def blend(start, end, progress):
	progress = clamp(progress, 0.0, 1.0)
	return start + (end - start) * progress


def find_track_edge_x(track_polygon, row_y, scan_from_left, coarse_step=6):
	if track_polygon is None:
		return None

	frame_width = video_size[0]
	if scan_from_left:
		x_range = range(0, frame_width, coarse_step)
		refine_direction = -1
	else:
		x_range = range(frame_width - 1, -1, -coarse_step)
		refine_direction = 1

	candidate = None
	for x in x_range:
		if cv2.pointPolygonTest(track_polygon, (float(x), float(row_y)), False) >= 0:
			candidate = x
			break

	if candidate is None:
		return None

	refine_x = candidate
	while 0 <= refine_x + refine_direction < frame_width:
		next_x = refine_x + refine_direction
		if cv2.pointPolygonTest(track_polygon, (float(next_x), float(row_y)), False) >= 0:
			refine_x = next_x
		else:
			break
	return refine_x


def find_track_y_from_bottom(track_polygon, col_x, step=3):
	if track_polygon is None:
		return None
	for row_y in range(video_size[1] - 1, -1, -step):
		if cv2.pointPolygonTest(track_polygon, (float(col_x), float(row_y)), False) >= 0:
			return row_y
	return None


def measure_track_profile(track_polygon, sample_rows):
	left_hits = []
	right_hits = []

	for row_y in sample_rows:
		left_x = find_track_edge_x(track_polygon, row_y, scan_from_left=True)
		right_x = find_track_edge_x(track_polygon, row_y, scan_from_left=False)
		if left_x is None or right_x is None:
			continue
		if right_x - left_x < 40:
			continue
		left_hits.append(left_x)
		right_hits.append(right_x)

	if not left_hits:
		return None

	avg_left = sum(left_hits) / len(left_hits)
	avg_right = sum(right_hits) / len(right_hits)
	width = avg_right - avg_left
	return {
		"left_x": avg_left,
		"right_x": avg_right,
		"width": width,
	}


def compute_wall_follow_steering(track_polygon, is_clockwise, previous_error, recovery_mode=False, target_ratio_override=None):
	sample_rows = (120, 140, 160) if recovery_mode else (170, 185, 200)
	profile = measure_track_profile(track_polygon, sample_rows)
	if profile is None:
		return 0, previous_error, None

	target_ratio = target_ratio_override if target_ratio_override is not None else (0.58 if is_clockwise else 0.42)
	target_x = profile["left_x"] + profile["width"] * target_ratio
	error = target_x - (video_size[0] / 2)
	error_delta = error - previous_error

	if recovery_mode:
		kp = 0.45
		kd = 0.55
		max_steer = 55
	else:
		kp = 0.55
		kd = 0.65
		max_steer = 65

	steering = (kp * error) + (kd * error_delta)
	steering = int(round(clamp(steering, -max_steer, max_steer)))
	profile["target_x"] = target_x
	profile["error"] = error
	profile["target_ratio"] = target_ratio
	return steering, error, profile


def measure_followed_wall(track_polygon, is_clockwise):
	if track_polygon is None:
		return None

	if is_clockwise:
		col_samples = range(video_size[0] - 12, video_size[0])
	else:
		col_samples = range(0, 12)

	row_samples = (150, 170, 190, 205)
	bottom_hits = []
	side_hits = []

	for col_x in col_samples:
		bottom_y = find_track_y_from_bottom(track_polygon, col_x)
		if bottom_y is not None:
			bottom_hits.append(bottom_y)

	for row_y in row_samples:
		if is_clockwise:
			edge_x = find_track_edge_x(track_polygon, row_y, scan_from_left=False)
			if edge_x is not None:
				side_hits.append(video_size[0] - edge_x)
		else:
			edge_x = find_track_edge_x(track_polygon, row_y, scan_from_left=True)
			if edge_x is not None:
				side_hits.append(edge_x)

	if not bottom_hits or not side_hits:
		return None

	avg_bottom_y = sum(bottom_hits) / len(bottom_hits)
	avg_side_x = sum(side_hits) / len(side_hits)
	return {
		"avg_bottom_y": avg_bottom_y,
		"avg_side_x": avg_side_x,
		"metric": avg_bottom_y + avg_side_x,
	}


CORNER_DELTA_THRESHOLD = 22.0
CORNER_CONFIRM_FRAMES = 2
CORNER_REFRACTORY_S = 0.45
MIN_NEAR_CORNER_WIDTH = 80.0
FORWARD_WIDTH_RATIO_THRESHOLD = 0.55
FORWARD_CORNER_ROWS = (90, 105)
NEAR_CORNER_ROWS = (170, 185, 200)


def update_corner_detection(track_polygon, is_clockwise, previous_metric, previous_count, last_trigger_time):
	signature = measure_followed_wall(track_polygon, is_clockwise)
	if signature is None:
		return False, previous_metric, 0, signature

	forward_profile = measure_track_profile(track_polygon, FORWARD_CORNER_ROWS)
	near_profile = measure_track_profile(track_polygon, NEAR_CORNER_ROWS)
	forward_width = forward_profile["width"] if forward_profile is not None else 0.0
	near_width = near_profile["width"] if near_profile is not None else 0.0

	metric = signature["metric"]
	if previous_metric is None:
		signature["delta"] = 0.0
		signature["forward_width"] = forward_width
		signature["near_width"] = near_width
		signature["geometry_trigger"] = False
		return False, metric, 0, signature

	delta = metric - previous_metric
	signature["delta"] = delta
	signature["forward_width"] = forward_width
	signature["near_width"] = near_width

	now = time.perf_counter()
	if now - last_trigger_time < CORNER_REFRACTORY_S:
		signature["geometry_trigger"] = False
		return False, metric, 0, signature

	geometry_trigger = (
		near_width > MIN_NEAR_CORNER_WIDTH
		and (forward_width == 0.0 or forward_width < near_width * FORWARD_WIDTH_RATIO_THRESHOLD)
	)
	signature["geometry_trigger"] = geometry_trigger

	if delta < -CORNER_DELTA_THRESHOLD or geometry_trigger:
		confirm_count = previous_count + 1
	else:
		confirm_count = 0

	return confirm_count >= CORNER_CONFIRM_FRAMES, metric, confirm_count, signature


def get_sector_target_ratio(is_clockwise, completed_turns):
	start_ratio = 0.52 if is_clockwise else 0.48
	final_ratio = 0.58 if is_clockwise else 0.42
	progress = clamp((completed_turns - 1) / 4.0, 0.0, 1.0)
	return blend(start_ratio, final_ratio, progress)


def get_sector_turn_duration_ms(completed_turns, speed):
	start_duration_ms = 1000 * 10 / speed
	min_duration_ms = 900 * 10 / speed
	progress = clamp((completed_turns - 1) / 4.0, 0.0, 1.0)
	return blend(start_duration_ms, min_duration_ms, progress)


front_point = [200, 85]
left_turning_point = [5, 200]
right_turning_point = [395, 200]
clockwise_indicator = [395, 100]
anticlockwise_indicator = [5, 100]
end_sector_point = [250, 35]

add_marker_point("Front Turning Point", front_point[0], front_point[1], color=(0, 0, 255), radius=2, label="Front P")
add_marker_point("Left Turning Point", left_turning_point[0], left_turning_point[1], color=(0, 0, 255), radius=2, label="Left TP")
add_marker_point("Right Turning Point", right_turning_point[0], right_turning_point[1], color=(0, 0, 255), radius=2, label="Right TP")
add_marker_point("Clockwise Indicator", clockwise_indicator[0], clockwise_indicator[1], color=(0, 0, 255), radius=2, label="C Indi")
add_marker_point("Anticlockwise Indicator", anticlockwise_indicator[0], anticlockwise_indicator[1], color=(0, 0, 255), radius=2, label="AntiC Indi")


run_OC1 = False
run_OC2 = False
reset_OC = True
num_of_turn = 0
is_clockwise = True
previous_wall_error = 0.0
previous_corner_metric = None
corner_confirm_count = 0
last_corner_trigger_time = 0.0
dash_start_time = 0.0
start_time = 0.0
end_time = 0.0
recorded_time = 0.0
speed = 6


class States(Enum):
	INIT = 0
	FIRST_SECTOR = 1
	TURNING_STATE = 2
	DASH_AFTER_TURNING_STATE = 3
	RUN_SECTOR_STATE = 4
	LAST_RUN = 5


state = States.INIT
last_state = States.INIT


def reset_markers():
	clear_marker_points()
	add_marker_point("Front Turning Point", front_point[0], front_point[1], color=(0, 0, 255), radius=2, label="Front P")
	add_marker_point("Left Turning Point", left_turning_point[0], left_turning_point[1], color=(0, 0, 255), radius=2, label="Left TP")
	add_marker_point("Right Turning Point", right_turning_point[0], right_turning_point[1], color=(0, 0, 255), radius=2, label="Right TP")
	add_marker_point("Clockwise Indicator", clockwise_indicator[0], clockwise_indicator[1], color=(0, 0, 255), radius=2, label="C Indi")
	add_marker_point("Anticlockwise Indicator", anticlockwise_indicator[0], anticlockwise_indicator[1], color=(0, 0, 255), radius=2, label="AntiC Indi")


try:
	print("IDLE")
	while True:
		if reset_OC:
			if run_OC1:
				num_of_turn = 0
				is_clockwise = True
				previous_wall_error = 0.0
				previous_corner_metric = None
				corner_confirm_count = 0
				last_corner_trigger_time = 0.0
				dash_start_time = 0.0
				state = States.FIRST_SECTOR
				last_state = None
				reset_markers()
				send_command_logged("0, 0, 0, R")
				print("Reset Complete. state = FIRST_SECTOR")
			reset_OC = False
			continue

		elif run_OC1:
			send_command_logged("OC1")
			start_time = time.perf_counter()
			while run_OC1:
				reply = esp_replyNprint()
				if reply == "EOC1":
					print(f"\n[{state.name}] EOC1 received")
					stop_vehicle("OC1 stop requested by ESP32")
					run_OC1 = False
					break

				set_all_color_detection(red=False, green=False, magenta=False)
				_, _, track = get_latest_data()
				track_polygon = track["polygon"]
				front_sees_track = get_track_distance(front_point[0], front_point[1])[0]

				if last_state != state:
					print(f"\n[{state.name}]")
					last_state = state

				if state == States.FIRST_SECTOR:
					send_command_logged(f"{speed}, 0, -1, forward")
					if not front_sees_track and (
						get_track_distance(left_turning_point[0], left_turning_point[1])[0]
						or get_track_distance(right_turning_point[0], right_turning_point[1])[0]
					):
						if get_track_distance(clockwise_indicator[0], clockwise_indicator[1])[0]:
							is_clockwise = True
							remove_marker_point("Anticlockwise Indicator")
						else:
							is_clockwise = False
							remove_marker_point("Clockwise Indicator")
						previous_wall_error = 0.0
						previous_corner_metric = None
						corner_confirm_count = 0
						start_turning_time = time.perf_counter_ns()
						last_corner_trigger_time = time.perf_counter()
						state = States.TURNING_STATE
						continue
					time.sleep(0.001)
					continue

				elif state == States.TURNING_STATE:
					turn_elapsed_ms = (time.perf_counter_ns() - start_turning_time) / 1000000
					blind_turn_duration_ms = get_sector_turn_duration_ms(num_of_turn, speed)

					if turn_elapsed_ms < blind_turn_duration_ms or not front_sees_track:
						steering = 100 if is_clockwise else -100
						send_command_logged(f"{speed}, {steering}, -1, turn-in")
						time.sleep(0.001)
						continue

					steering, previous_wall_error, profile = compute_wall_follow_steering(
						track_polygon,
						is_clockwise,
						previous_wall_error,
						recovery_mode=True,
						target_ratio_override=(0.52 if is_clockwise else 0.48),
					)
					if profile is None:
						steering = 55 if is_clockwise else -55
					send_command_logged(f"{speed}, {steering}, -1, turn-align")

					if turn_elapsed_ms < 1100:
						continue

					if profile is not None and abs(profile["error"]) <= 18:
						num_of_turn += 1
						print(f"num turn: {num_of_turn}")
						dash_start_time = time.perf_counter()
						last_corner_trigger_time = time.perf_counter()
						previous_corner_metric = None
						corner_confirm_count = 0
						state = States.DASH_AFTER_TURNING_STATE
						continue

					if turn_elapsed_ms < 1500:
						continue

					num_of_turn += 1
					print(f"num turn: {num_of_turn}")
					dash_start_time = time.perf_counter()
					last_corner_trigger_time = time.perf_counter()
					previous_corner_metric = None
					corner_confirm_count = 0
					state = States.DASH_AFTER_TURNING_STATE
					continue

				elif state == States.DASH_AFTER_TURNING_STATE:
					target_ratio = get_sector_target_ratio(is_clockwise, num_of_turn)
					steering, previous_wall_error, _ = compute_wall_follow_steering(
						track_polygon,
						is_clockwise,
						previous_wall_error,
						target_ratio_override=target_ratio,
					)
					send_command_logged(f"{speed}, {steering}, -1, settle after turn")
					if time.perf_counter() - dash_start_time < 0.35:
						time.sleep(0.001)
						continue
					state = States.RUN_SECTOR_STATE if num_of_turn < 12 else States.LAST_RUN
					continue

				elif state == States.RUN_SECTOR_STATE:
					target_ratio = get_sector_target_ratio(is_clockwise, num_of_turn)
					steering, previous_wall_error, profile = compute_wall_follow_steering(
						track_polygon,
						is_clockwise,
						previous_wall_error,
						target_ratio_override=target_ratio,
					)
					send_command_logged(f"{speed}, {steering}, -1, wall follow")

					corner_detected, previous_corner_metric, corner_confirm_count, signature = update_corner_detection(
						track_polygon,
						is_clockwise,
						previous_corner_metric,
						corner_confirm_count,
						last_corner_trigger_time,
					)
					if signature is not None:
						print(
							f"[CORNER METRIC] side={signature['avg_side_x']:.2f} bottom={signature['avg_bottom_y']:.2f} "
							f"metric={signature['metric']:.2f} delta={signature.get('delta', 0.0):+.2f} "
							f"near_w={signature.get('near_width', 0.0):.2f} forward_w={signature.get('forward_width', 0.0):.2f} "
							f"geom={int(signature.get('geometry_trigger', False))} count={corner_confirm_count}"
						)

					if corner_detected and not front_sees_track:
						previous_wall_error = 0.0
						start_turning_time = time.perf_counter_ns()
						last_corner_trigger_time = time.perf_counter()
						state = States.TURNING_STATE
						continue

					time.sleep(0.001)
					continue

				elif state == States.LAST_RUN:
					add_marker_point("End Sector Point", end_sector_point[0], end_sector_point[1], color=(0, 0, 255), radius=2, label="Stop P")
					target_ratio = get_sector_target_ratio(is_clockwise, num_of_turn)
					steering, previous_wall_error, _ = compute_wall_follow_steering(
						track_polygon,
						is_clockwise,
						previous_wall_error,
						target_ratio_override=target_ratio,
					)
					send_command_logged(f"{speed}, {steering}, -1, move forward")
					if get_track_distance(end_sector_point[0], end_sector_point[1])[0]:
						time.sleep(0.001)
						continue
					stop_vehicle("OC1 final stop")
					end_time = time.perf_counter()
					run_OC1 = False
					reset_OC = True
					break

				time.sleep(0.001)

		elif run_OC2:
			reply = esp_replyNprint()
			if reply == "EOC2":
				stop_vehicle("OC2 stop requested by ESP32")
				run_OC2 = False
			time.sleep(0.001)

		else:
			recorded_time = end_time - start_time
			reply = esp_replyNprint()
			if reply == "ROC1":
				reset_OC = True
				run_OC1 = True
				continue
			elif reply == "ROC2":
				print("OC2 not implemented in main2.py")
				continue
		time.sleep(0.001)

except KeyboardInterrupt:
	print("\nShutting down software pipeline gracefully...")
	try:
		esp.send_command("0, 0, 0, emergency_stop")
	except Exception:
		pass
	esp.disconnect()
	stop_vision_system()
	time.sleep(0.5)
