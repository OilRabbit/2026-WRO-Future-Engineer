# testing
import cv2
import time
import datetime
import math
from esp_com.communication import ESP32Communicator
from camera.camera_utils import start_vision_system, start_web_server, get_latest_data, stop_vision_system, get_track_distance, set_marker_point, remove_marker_point, clear_marker_points, set_marker_line, remove_marker_line, clear_marker_lines, set_color_detection, set_all_color_detection, configure_vision_pipeline
from picamera2 import Picamera2 as picam2
from enum import Enum

# ESP communication and Picam init
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

# Keep the control path light: no debug strip in the hot loop, and stream the
# main camera frame instead of the combined debug mosaic.
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
        start_web_server(host='0.0.0.0', port=5000)
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
        except Exception as e:
                print(f"\n[SERIAL WARNING] Caught hardware communication glitch: {e}")
                print("Attempting to bypass frame drop...")
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

def remove_marker(name):
	remove_marker_point(name)

def add_marker_line(name, x1, y1, x2, y2, color=(0, 255, 255), thickness=2, label=None):
	set_marker_line(name, x1, y1, x2, y2, color=color, thickness=thickness, label=label)

def remove_line(name):
	remove_marker_line(name)

def set_red_detection(enabled):
	set_color_detection("red", enabled)

def set_green_detection(enabled):
	set_color_detection("green", enabled)

def set_magenta_detection(enabled):
	set_color_detection("magenta", enabled)

def set_color_block_detection(red=None, green=None, magenta=None):
	set_all_color_detection(red=red, green=green, magenta=magenta)

def set_debug_mode(enabled):
	if enabled:
		configure_vision_pipeline(
			draw_overlays=True,
			show_debug_strip=True,
			stream_use_debug_frame=True,
			record_use_debug_frame=True,
			stream_jpeg_quality=85,
		)
	else:
		configure_vision_pipeline(
			draw_overlays=True,
			show_debug_strip=False,
			stream_use_debug_frame=False,
			record_use_debug_frame=False,
			stream_jpeg_quality=70,
		)

def clamp(value, minimum, maximum):
	return max(minimum, min(maximum, value))

def blend(start, end, progress):
	progress = clamp(progress, 0.0, 1.0)
	return start + (end - start) * progress

def draw_and_check_line(name, start_point, end_point, color=(0, 255, 255), thickness=2, label=None, sample_spacing=2, track_polygon=None):
	add_marker_line(name, start_point[0], start_point[1], end_point[0], end_point[1], color=color, thickness=thickness, label=label)

	if track_polygon is None:
		_, _, track = get_latest_data()
		track_polygon = track["polygon"]
	if track_polygon is None:
		return True

	x1, y1 = start_point
	x2, y2 = end_point
	delta_x = x2 - x1
	delta_y = y2 - y1
	line_length = math.hypot(delta_x, delta_y)
	if line_length == 0:
		return cv2.pointPolygonTest(track_polygon, (float(x1), float(y1)), False) < 0

	sample_count = max(2, int(line_length / max(1, sample_spacing)) + 1)
	for step in range(sample_count):
		progress = step / (sample_count - 1)
		x = x1 + (delta_x * progress)
		y = y1 + (delta_y * progress)
		if cv2.pointPolygonTest(track_polygon, (float(x), float(y)), False) < 0:
			return True
	return False

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

def _compute_pid_steering_from_error(error, previous_error, kp, kd, max_steer):
	error_delta = error - previous_error
	steering = (kp * error) + (kd * error_delta)
	steering = int(round(clamp(steering, -max_steer, max_steer)))
	return steering, error_delta

outer_wall_offset_px = 140
speed = 7

def compute_wall_follow_steering(track_polygon, is_clockwise, previous_error, recovery_mode=False, target_ratio_override=None):
	if recovery_mode:
		kp = 0.50
		kd = 0.30
		max_steer = 30 * speed / 6
		sample_row_candidates = (
			(120, 140, 160),
			(135, 155, 175),
			(150, 170, 190),
		)
	else:
		kp = 0.30
		kd = 0.20
		max_steer = 20 * speed / 6
		sample_row_candidates = ((170, 185, 200),)

	profile = None
	for sample_rows in sample_row_candidates:
		profile = measure_track_profile(track_polygon, sample_rows)
		if profile is not None:
			profile["sample_rows"] = sample_rows
			break

	if profile is None:
		if recovery_mode:
			steering, error_delta = _compute_pid_steering_from_error(previous_error, previous_error, kp, kd, max_steer)
			profile = {
				"left_x": None,
				"right_x": None,
				"width": None,
				"target_x": None,
				"error": previous_error,
				"error_delta": error_delta,
				"target_ratio": None,
				"sample_rows": None,
				"fallback_pid": True,
			}
			return steering, previous_error, profile
		return 0, previous_error, None

	target_ratio = target_ratio_override if target_ratio_override is not None else (0.6 if is_clockwise else 0.4)
	if is_clockwise:
		target_x = profile["left_x"] + outer_wall_offset_px
	else:
		target_x = profile["right_x"] - outer_wall_offset_px
	error = target_x - (video_size[0] / 2)
	steering, error_delta = _compute_pid_steering_from_error(error, previous_error, kp, kd, max_steer)

	profile["target_x"] = target_x
	profile["error"] = error
	profile["error_delta"] = error_delta
	profile["target_ratio"] = target_ratio
	profile["fallback_pid"] = False
	return steering, error, profile

def get_sector_target_ratio(is_clockwise, completed_turns):
	start_ratio = 0.46 if is_clockwise else 0.54
	final_ratio = 0.46 if is_clockwise else 0.54
	progress = clamp((completed_turns - 1) / 4.0, 0.0, 1.0)
	return blend(start_ratio, final_ratio, progress)

def get_sector_turn_duration_ms(completed_turns):
	start_duration_ms = 900 * 10 / speed
	min_duration_ms = 850 * 10 / speed
	progress = clamp((completed_turns - 1) / 4.0, 0.0, 1.0)
	return blend(start_duration_ms, min_duration_ms, progress)

def get_recovery_duration_ms():
	return 1000 * 6 / speed

# Checkpoints (default as clockwise case)
front_point = [200, 93]
add_marker_point("Front Turning Point", front_point[0], front_point[1], color=(0, 0, 255), radius=2, label="Front P")
left_turning_point = [5, 200] #check direction
add_marker_point("Left Turning Point", left_turning_point[0], left_turning_point[1], color=(0, 0, 255), radius=2, label="Left TP")
right_turning_point = [395, 200] #check direction
add_marker_point("Right Turning Point", right_turning_point[0], right_turning_point[1], color=(0, 0, 255), radius=2, label="Right TP")
clockwise_indicator = [395, 100]
add_marker_point("Clockwise Indicator", clockwise_indicator[0], clockwise_indicator[1], color=(0, 0, 255), radius=2, label="C Indi")
anticlockwise_indicator = [5, 100]
add_marker_point("Anticlockwise Indicator", anticlockwise_indicator[0], anticlockwise_indicator[1], color=(0, 0, 255), radius=2, label="AntiC Indi")

turning_point = right_turning_point #check if the robot get to the position that should turn
track_left = [375, 225] #check if the robot is getting left from the ideal track
track_right = [400, 225] #check if the robot is getting right from the ideal track
ending_point = [200, 62.5] #check if the robot is at the ideal point to end

# Global variables
run_OC1 = False
run_OC2 = False
reset_OC = True
num_of_turn = 0
is_clockwise = True
previous_wall_error = 0.0
dash_start_time = 0.0
run_sector_start_time = 0.0
start_time = 0
end_time = 0
recorded_time = 0

# OC1 States
class States(Enum):
	INIT = 0
	FIRST_SECTOR = 1 #Move forward until it knows the direction to run
	WAIT_TURN_STATE = 2 #Move forward for fixed distance to get to the ideal point to turn
	TURNING_STATE = 3 #Turn until it is parallel to the next path
	DASH_AFTER_TURNING_STATE = 4 #Move forward until it passed the corner sector
	RUN_SECTOR_STATE = 5 #Move forward until is time to turn
	LAST_RUN = 6 #Move forward to stop at the right place

state = States.INIT
last_state = States.INIT
run_target_sent = False

try:
        print("IDLE")
        while True:
                if reset_OC:
                        if run_OC1 or run_OC2:
                                angle = 0
                                turn_time = 0
                                num_of_turn = 0
                                lap_count_oc1 = 0
                                last_purple_lap_time = 0.0
                                purple_wall_lockout_active = False
                                is_clockwise = True
                                previous_wall_error = 0.0
                                dash_start_time = 0.0
                                run_sector_start_time = 0.0
                                turning_point = right_turning_point
                                state = States.FIRST_SECTOR
                                last_state = None
                                clear_marker_points()
                                clear_marker_lines()
                                # Checkpoints (default as clockwise case)
                                # front_point = [200, 105]
                                add_marker_point("Front Turning Point", front_point[0], front_point[1], color=(0, 0, 255), radius=2, label="Front P")
                                left_turning_point = [5, 200] #check direction
                                add_marker_point("Left Turning Point", left_turning_point[0], left_turning_point[1], color=(0, 0, 255), radius=2, label="Left TP")
                                right_turning_point = [395, 200] #check direction
                                add_marker_point("Right Turning Point", right_turning_point[0], right_turning_point[1], color=(0, 0, 255), radius=2, label="Right TP")
                                clockwise_indicator = [395, 100]
                                add_marker_point("Clockwise Indicator", clockwise_indicator[0], clockwise_indicator[1], color=(0, 0, 255), radius=2, label="C Indi")
                                anticlockwise_indicator = [5, 100]
                                add_marker_point("Anticlockwise Indicator", anticlockwise_indicator[0], anticlockwise_indicator[1], color=(0, 0, 255), radius=2, label="AntiC Indi")
                                esp.send_command("0, 0, 0, R")
                                print("Reset Complete. state = FIRST_SECTOR")
                        reset_OC = False
                        continue

                # ======================================================================
                # RUN OC1 BLOCK
                # ======================================================================
                elif run_OC1:
                        send_command_logged("OC1")
                        start_time = time.perf_counter()
                        while run_OC1:
                                reply = esp_replyNprint()
                                if reply == "EOC1":
                                        print(f"\n[{state.name}][FSM ALERT] EOC1 Signal Received from ESP32. Terminating run immediately.")
                                        stop_vehicle("OC1 stop requested by ESP32")
                                        run_OC1 = False
                                        break
                                set_color_block_detection(False, False, False)
                                _, _, track = get_latest_data()
                                #print("num turn = {}".format(num_of_turn))

                                if last_state != state:
                                	print(f"\n[{state.name}]")
                                	last_state = state

                                if state == States.FIRST_SECTOR:
                                	esp.send_command(str(speed) + ", 0, -1, forward")
                                	if get_track_distance(front_point[0], front_point[1])[0] == False and (get_track_distance(left_turning_point[0], left_turning_point[1])[0] == True or get_track_distance(right_turning_point[0], right_turning_point[1])[0] == True):
                                		if num_of_turn == 0:
                                			if get_track_distance(clockwise_indicator[0], clockwise_indicator[1])[0]:
                                				is_clockwise = True
                                				remove_marker("Anticlockwise Indicator")
                                				turning_point = right_turning_point
                                				#front_point = [280, 98]
                                			else:
                                				is_clockwise = False
                                				remove_marker("Clockwise Indicator")
                                				turning_point = left_turning_point
                                				#front_point = [120, 98]
                                			add_marker_point("Front Turning Point", front_point[0], front_point[1], color=(0, 0, 255), radius=2, label="Front P")
                                			is_clockwise = get_track_distance(clockwise_indicator[0], clockwise_indicator[1])[0]
                                			previous_wall_error = 0.0
                                		start_turning_time = time.perf_counter_ns()
                                		run_sector_start_time = 0.0
                                		state = States.TURNING_STATE
                                		continue
                                	else:
                                        	time.sleep(0.001)
                                        	continue

                                # Turn
                                elif state == States.TURNING_STATE:
                                	if is_clockwise:
                                		front_point = [260, 93]
                                	else:
                                		front_point = [140, 93]
                                	turn_elapsed_ms = (time.perf_counter_ns() - start_turning_time) / 1000000
                                	blind_turn_duration_ms = get_sector_turn_duration_ms(num_of_turn)
                                	front_sees_track = get_track_distance(front_point[0], front_point[1])[0]
                                	if turn_elapsed_ms < blind_turn_duration_ms or not front_sees_track:
                                		steering = 100 if is_clockwise else -100
                                		esp.send_command(str(speed) + ", " + str(steering) + ", -1, turn-in")
                                		continue

                                	steering, previous_wall_error, profile = compute_wall_follow_steering(
                                		track["polygon"],
                                		is_clockwise,
                                		previous_wall_error,
                                		recovery_mode=False,
                                		target_ratio_override=(0.48 if is_clockwise else 0.52),
                                	)
                                	esp.send_command(str(speed) + ", " + str(steering) + ", -1, turn-align")

                                	if turn_elapsed_ms < 1100:
                                		continue

                                	if profile is not None and abs(profile["error"]) <= 18:
                                		num_of_turn += 1
                                		print("num turn: {}".format(num_of_turn))
                                		dash_start_time = time.perf_counter()
                                		run_sector_start_time = 0.0
                                		state = States.DASH_AFTER_TURNING_STATE
                                		continue

                                	if turn_elapsed_ms < 1500:
                                		continue

                                	num_of_turn += 1
                                	# print("num turn: {}".format(num_of_turn))
                                	# front_point = [200, 98]
                                	dash_start_time = time.perf_counter()
                                	run_sector_start_time = 0.0
                                	state = States.DASH_AFTER_TURNING_STATE
                                	continue
                                
                                elif state == States.DASH_AFTER_TURNING_STATE:
                                	front_point = [200, 93]
                                	target_ratio = get_sector_target_ratio(is_clockwise, num_of_turn)
                                	steering, previous_wall_error, _ = compute_wall_follow_steering(
                                		track["polygon"],
                                		is_clockwise,
                                		previous_wall_error,
                                		target_ratio_override=target_ratio,
                                	)
                                	esp.send_command(str(speed) + ", " + str(steering) + ", -1, settle after turn")
                                	if time.perf_counter() - dash_start_time < 1:
                                		continue
                                	run_sector_start_time = time.perf_counter()
                                	state = States.RUN_SECTOR_STATE if num_of_turn < 12 else States.LAST_RUN
                                	continue

                                # Run sector and keep a certain distance from the inner barrier
                                elif state == States.RUN_SECTOR_STATE:
                                	if get_track_distance(front_point[0], front_point[1])[0] == False:
                                        	state = States.TURNING_STATE
                                        	previous_wall_error = 0.0
                                        	run_sector_start_time = 0.0
                                        	start_turning_time = time.perf_counter_ns()
                                        	continue
                                	if run_sector_start_time == 0.0:
                                		run_sector_start_time = time.perf_counter()
                                	run_sector_elapsed_ms = (time.perf_counter() - run_sector_start_time) * 1000
                                	target_ratio = get_sector_target_ratio(is_clockwise, num_of_turn)
                                	steering, previous_wall_error, profile = compute_wall_follow_steering(
                                		track["polygon"],
                                		is_clockwise,
                                		previous_wall_error,
                                		recovery_mode=(run_sector_elapsed_ms < get_recovery_duration_ms()),
                                		target_ratio_override=target_ratio,
                                	)
                                	print("steering = {}".format(steering))
                                	esp.send_command(str(speed) + ", " + str(steering) + ", -1, wall follow")
                                	continue

                                # Last forward to stop
                                elif state == States.LAST_RUN:
                                	end_sector_point = [200, 70] 
                                	add_marker_point("End Sector Point", end_sector_point[0], end_sector_point[1], color=(0, 0, 255), radius=2, label="Stop P")
                                	target_ratio = get_sector_target_ratio(is_clockwise, num_of_turn)
                                	steering, previous_wall_error, _ = compute_wall_follow_steering(track["polygon"], is_clockwise, previous_wall_error, target_ratio_override=target_ratio)
                                	esp.send_command(str(speed) + ", " + str(steering) + ", -1, move forward")
                                	# Wait until the ending_point reach the wall in front of the robot
                                	if get_track_distance(end_sector_point[0], end_sector_point[1])[0]:
                                		time.sleep(0.001)
                                		continue
                                	esp.send_command("0, 0, 0, motor stop")
                                	end_time = time.perf_counter()
                                	run_OC1 = False
                                	reset_OC = True
                                	break

                                # End of OC1 FSM #
                                time.sleep(0.001)
                elif run_OC2:
                        send_command_logged("OC2")
                        while run_OC2:
                                reply = esp_replyNprint()
                                if reply == "EOC2":
                                        stop_vehicle("OC2 stop requested by ESP32")
                                        run_OC2 = False
                                        break
                                time.sleep(0.001)

                else:
                        recorded_time = end_time - start_time
                        reply = esp_replyNprint()
                        if reply == "ROC1":
                                reset_OC = True
                                run_OC1 = True
                                continue
                        elif reply == "ROC2":
                                reset_OC = True
                                run_OC2 = True
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
