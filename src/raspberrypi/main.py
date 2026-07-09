<<<<<<< HEAD
# testing
import cv2
=======
>>>>>>> 14854120d11683213666a55507ca8feeafb4b4b3
import time
import datetime
import math
from esp_com.communication import ESP32Communicator
from camera.camera_utils import start_vision_system, start_web_server, get_latest_data, stop_vision_system, get_track_distance, set_marker_point, remove_marker_point, clear_marker_points, set_color_detection, set_all_color_detection, configure_vision_pipeline
from picamera2 import Picamera2 as picam2
from enum import Enum

# ESP communication and Picam init
print("======= Init =======")

esp = ESP32Communicator()
esp.connect()
camera = picam2()
<<<<<<< HEAD
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
=======
full_sensor_res = camera.sensor_resolution
config = camera.create_video_configuration(main={"size": (640, 360), "format": "BGR888"},sensor={"output_size": full_sensor_res})
>>>>>>> 14854120d11683213666a55507ca8feeafb4b4b3
camera.configure(config)
camera.start()
print("Camera: Activated")

<<<<<<< HEAD
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

=======
start_vision_system(camera, True)

live_streaming = False
>>>>>>> 14854120d11683213666a55507ca8feeafb4b4b3
if live_streaming:
	print("Streaming: Activated")
	start_web_server(host='0.0.0.0', port=5000)
else:
	print("Streaming: Not streaming")

print("======= End of Init =======")

<<<<<<< HEAD
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

def compute_wall_follow_steering(track_polygon, is_clockwise, previous_error, recovery_mode=False, target_ratio_override=None):
    sample_rows = (120, 140, 160) if recovery_mode else (170, 185, 200)
    profile = measure_track_profile(track_polygon, sample_rows)
    if profile is None:
        return 0, previous_error, None

    target_ratio = target_ratio_override if target_ratio_override is not None else (0.6 if is_clockwise else 0.4)
    target_x = profile["left_x"] + profile["width"] * target_ratio
    error = target_x - (video_size[0] / 2)
    error_delta = error - previous_error

    if recovery_mode:
        kp = 1.2
        kd = 0.7
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

def get_sector_target_ratio(is_clockwise, completed_turns):
    start_ratio = 0.52 if is_clockwise else 0.48
    final_ratio = 0.58 if is_clockwise else 0.42
    progress = clamp((completed_turns - 1) / 4.0, 0.0, 1.0)
    return blend(start_ratio, final_ratio, progress)

def get_sector_turn_duration_ms(completed_turns):
    start_duration_ms = 1000
    min_duration_ms = 750
    progress = clamp((completed_turns - 1) / 4.0, 0.0, 1.0)
    return blend(start_duration_ms, min_duration_ms, progress)


# Checkpoints (default as clockwise case)
front_point = [200, 80]
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
speed = 10

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
                                turning_point = right_turning_point
                                state = States.FIRST_SECTOR
                                last_state = None
                                clear_marker_points()
                                # Checkpoints (default as clockwise case)
                                front_point = [200, 80]
                                add_marker_point("Front Turning Point", front_point[0], front_point[1], color=(0, 0, 255), radius=2, label="Front P")
                                left_turning_point = [5, 200] #check direction
                                add_marker_point("Left Turning Point", left_turning_point[0], left_turning_point[1], color=(0, 0, 255), radius=2, label="Left TP")
                                right_turning_point = [395, 200] #check direction
                                add_marker_point("Right Turning Point", right_turning_point[0], right_turning_point[1], color=(0, 0, 255), radius=2, label="Right TP")
                                clockwise_indicator = [395, 100]
                                add_marker_point("Clockwise Indicator", clockwise_indicator[0], clockwise_indicator[1], color=(0, 0, 255), radius=2, label="C Indi")
                                anticlockwise_indicator = [5, 100]
                                add_marker_point("Anticlockwise Indicator", anticlockwise_indicator[0], anticlockwise_indicator[1], color=(0, 0, 255), radius=2, label="AntiC Indi")

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

                                if last_state != state:
                                    print(f"\n[{state.name}]")
                                    last_state = state

                                if state == States.FIRST_SECTOR:
                                    esp.send_command(str(speed) + ", 0, -1, forward")
                                    print(f"[{state.name}][MOVE] Sent forward command: Speed={speed}, Angle=0")
                                    if get_track_distance(front_point[0], front_point[1])[0] == False and (get_track_distance(left_turning_point[0], left_turning_point[1])[0] == True or get_track_distance(right_turning_point[0], right_turning_point[1])[0] == True):
                                        if num_of_turn == 0:
                                            if get_track_distance(clockwise_indicator[0], clockwise_indicator[1])[0]:
                                                is_clockwise = True
                                                # front_point = [140, 80]
                                                remove_marker("Anticlockwise Indicator")
                                                innerwall_white = [335, 220]
                                                innerwall_black = [365, 220]
                                            else:
                                                is_clockwise = False
                                                # front_point = [260, 80]
                                                remove_marker("Clockwise Indicator")
                                                innerwall_white = [65, 220]
                                                innerwall_black = [35, 220]
                                            add_marker_point("Front Turning Point", front_point[0], front_point[1], color=(0, 0, 255), radius=2, label="Front P")
                                            add_marker_point("Innerwall White", innerwall_white[0], innerwall_white[1], color=(0, 0, 255), radius=3, label="Danger")
                                            add_marker_point("Innerwall Black", innerwall_black[0], innerwall_black[1], color=(255, 0, 0), radius=3, label="Safe")
                                            is_clockwise = get_track_distance(clockwise_indicator[0], clockwise_indicator[1])[0]
                                            previous_wall_error = 0.0
                                        start_turning_time = time.perf_counter_ns()
                                        state = States.TURNING_STATE
                                        continue
                                    else:
                                            time.sleep(0.001)
                                            continue

                                # Turn
                                elif state == States.TURNING_STATE:
                                    turn_elapsed_ms = (time.perf_counter_ns() - start_turning_time) / 1000000
                                    blind_turn_duration_ms = get_sector_turn_duration_ms(num_of_turn)
                                    if turn_elapsed_ms < blind_turn_duration_ms:
                                        steering = 80 if is_clockwise else -80
                                        esp.send_command(str(speed) + ", " + str(steering) + ", -1, turn-in")
                                        print(f"[{state.name}][MOVE][BLIND-TURN] Sent turn-in command: Speed={speed}, Steering={steering} (Elapsed: {turn_elapsed_ms:.1f}ms / Blind Target: {blind_turn_duration_ms:.1f}ms)")
                                        continue

                                    steering, previous_wall_error, profile = compute_wall_follow_steering(
                                        track["polygon"],
                                        is_clockwise,
                                        previous_wall_error,
                                        recovery_mode=True,
                                        target_ratio_override=(0.52 if is_clockwise else 0.48),
                                    )
                                    if profile is None:
                                        steering = 55 if is_clockwise else -55
                                    esp.send_command(str(speed) + ", " + str(steering) + ", -1, turn-align")
                                    print(f"[{state.name}][MOVE][ALIGN-TURN] Sent turn-align command: Speed={speed}, Steering={steering} (Error Profile: {None if profile is None else profile.get('error')})")

                                    if turn_elapsed_ms < 1100:
                                        continue

                                    if profile is not None and abs(profile["error"]) <= 18:
                                        num_of_turn += 1
                                        print("num turn: {}".format(num_of_turn))
                                        dash_start_time = time.perf_counter()
                                        state = States.DASH_AFTER_TURNING_STATE if num_of_turn < 12 else States.LAST_RUN
                                        continue

                                    if turn_elapsed_ms < 1500:
                                        continue

                                    num_of_turn += 1
                                    print("num turn: {}".format(num_of_turn))
                                    dash_start_time = time.perf_counter()
                                    state = States.DASH_AFTER_TURNING_STATE if num_of_turn < 12 else States.LAST_RUN
                                    continue
                                
                                elif state == States.DASH_AFTER_TURNING_STATE:
                                    target_ratio = get_sector_target_ratio(is_clockwise, num_of_turn)
                                    steering, previous_wall_error, _ = compute_wall_follow_steering(
                                        track["polygon"],
                                        is_clockwise,
                                        previous_wall_error,
                                        target_ratio_override=target_ratio,
                                    )
                                    if is_clockwise:
                                        steering = steering + 40
                                    else:
                                        steering = steering - 40    
                                    esp.send_command(str(speed) + ", " + str(steering) + ", -1, settle after turn")
                                    print(f"[{state.name}][MOVE] Sent settle after turn command: Speed={speed}, Steering={steering}, Target Ratio={target_ratio:.3f}")
                                    if time.perf_counter() - dash_start_time < 0.35:
                                        continue
                                    state = States.RUN_SECTOR_STATE
                                    continue

                                # Run sector and keep a certain distance from the inner barrier
                                elif state == States.RUN_SECTOR_STATE:
                                    if get_track_distance(front_point[0], front_point[1])[0] == False and (get_track_distance(left_turning_point[0], left_turning_point[1])[0] == True or get_track_distance(right_turning_point[0], right_turning_point[1])[0] == True):
                                            state = States.TURNING_STATE
                                            previous_wall_error = 0.0
                                            start_turning_time = time.perf_counter_ns()
                                            continue
                                    target_ratio = get_sector_target_ratio(is_clockwise, num_of_turn)
                                    steering, previous_wall_error, profile = compute_wall_follow_steering(
                                        track["polygon"],
                                        is_clockwise,
                                        previous_wall_error,
                                        target_ratio_override=target_ratio,
                                    )
                                    if profile is None:
                                        if not get_track_distance(innerwall_white[0], innerwall_white[1])[0]:
                                            steering = -50 if is_clockwise else 50
                                        elif get_track_distance(innerwall_black[0], innerwall_black[1])[0]:
                                            steering = 5 if is_clockwise else -5
                                        else:
                                            steering = 0
                                            
                                    steering = steering * 1.2
                                    esp.send_command(str(speed) + ", " + str(steering) + ", -1, wall follow")
                                    print(f"[{state.name}][MOVE] Sent wall follow command: Speed={speed}, Steering={steering}, Target Ratio={target_ratio:.3f}")
                                    continue

                                # Last forward to stop
                                elif state == States.LAST_RUN:
                                    end_sector_point = [250, 35] if is_clockwise else [150, 35]
                                    add_marker_point("End Sector Point", end_sector_point[0], end_sector_point[1], color=(0, 0, 255), radius=2, label="Stop P")
                                    target_ratio = get_sector_target_ratio(is_clockwise, num_of_turn)
                                    steering, previous_wall_error, _ = compute_wall_follow_steering(track["polygon"], is_clockwise, previous_wall_error, target_ratio_override=target_ratio)
                                    esp.send_command(str(speed) + ", " + str(steering) + ", -1, move forward")
                                    print(f"[{state.name}][MOVE] Sent final run command: Speed={speed}, Steering={steering}")
                                    # Wait until the ending_point reach the wall in front of the robot
                                    if get_track_distance(end_sector_point[0], end_sector_point[1])[0]:
                                        time.sleep(0.001)
                                        continue
                                    esp.send_command("0, 0, 0, motor stop")
                                    print(f"[{state.name}][STOP] Sent absolute motor stop command at end point marker.")
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
=======
# Global variables
run_OC1 = False
run_OC2 = False
reset_OC = True
num_of_turn = 0
is_clockwise = True
start_time = 0
end_time = 0
recorded_time = 0

#Checkpoints (default as clockwise case)
angle = 0 #steering percentage
speed = 0
turn_flag = 0
turn_indi_1 = [320, 140] #check when to turn
turn_indi_2 = [320, 170] #check when to turn
sector_indi = [[240, 140], [400, 140]] #check when is sector
turn_time = 0
run_time = 0
lot_count = 0
lot_count_flag = 1
pillar_count = 0
pillar_count_flag = 1
pillar_count_temp = 0

front_turning_point = [320, 70]
left_turning_point = [40, 200] #check direction
right_turning_point = [600, 200] #check direction
turning_point = right_turning_point #check if the robot get to the position that should turn
track_left = [[80, 180], [520, 180]] #check if the robot is getting left from the ideal track
track_right = [[120, 180], [560, 180]] #check if the robot is getting right from the ideal track
ending_point = [320, 30] #check if the robot is at the ideal point to end

#States
class States(Enum):
	INIT = 0
	FIRST_SECTOR = 1 #Move forward until it knows the direction to run
	WAIT_TURN_STATE = 2 #Move forward for fixed distance to get to the ideal point to turn
	TURNING_STATE = 3 #Turn until it is parallel to the next path
	DASH_AFTER_TURNING_STATE = 4 #Move forward until it passed the corner sector
	RUN_SECTOR_STATE = 5 #Move forward until is time to turn
	LAST_RUN = 6 #Move forward to stop at the right place

class OC2_States(Enum):
	INIT = 0
	LEAVE = 1
	WHITE = 2
	PILLAR = 3
	ENTER = 4

# Function for receiving msg from ESP and print the message with timestamp
def esp_replyNprint():
	timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
	reply = esp.read_message()
	if reply is not None:
		timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
		print(f"{timestamp} ESP: {reply}")
	return reply

state = States.INIT
OC2_state = OC2_States.INIT
try:
	print("IDLE")
	while True:
		if reset_OC:
			# Add anything you need to reset before each OC run below this line #
			if run_OC1:
				angle = 0
				turn_time = 0
				run_time = 0
				num_of_turn = 0
				is_clockwise = True
				turning_point = right_turning_point
				state = States.FIRST_SECTOR
			if run_OC2:
				angle = 0
				speed = 0
				is_clockwise = True
				lot_count = 0
				lot_count_flag = 1
				pillar_count = 0
				pillar_count_flag = 1
				OC2_state = OC2_States.LEAVE
				print("leave") 
			# End of reset #
			reset_OC = False
			print("Resetted")
			# print("IDLE")
			continue
			
		# TODO: add timer
		elif run_OC1:
			esp.send_command("OC1")
			start_time = time.perf_counter()
			while run_OC1:
				end_time = time.perf_counter()
				reply = esp_replyNprint()
				if reply == "EOC1":
					print("EOC1")
					run_OC1 = False
					break
				_, _, track = get_latest_data()
	
				# OC1 FSM
				#Format eg: esp.send_command("<speed>, <streering percentage>, <distance(-1 when not needed)>, <what to do>")
				#The first sector
				if state == States.FIRST_SECTOR:
					angle = (track["center_x"]-320)*abs(track["center_x"]-320)/80
					esp.send_command("12, " + str(angle) + ", -1, move forward")
					time.sleep(0.025)
					if get_track_distance(turn_indi_1[0], turn_indi_1[1])[0] == False and get_track_distance(turn_indi_2[0], turn_indi_2[1])[0] == True and track["center_x"] != 0:
						if turn_flag:
							turn_flag = 0
							if track["center_x"] < 320:
								is_clockwise = False
							else:
								is_clockwise = True
							print(is_clockwise)
							state = States.TURNING_STATE
							num_of_turn += 1
							print(num_of_turn)
							print("turning")
						else:
							turn_flag = 1
					else:
						turn_flag = 0
					continue

					esp.send_command("8, 0, -1, move forward")
					if get_track_distance(turn_indi_1[0], turn_indi_1[1])[0] == False:
						if track["center_x"] < 320 :
							is_clockwise = False
							turning_point = left_turning_point
						state = States.WAIT_TURN_STATE
						print(is_clockwise)
						print("wait")
					continue
				
				#Wait turn
				elif state == States.WAIT_TURN_STATE:
					esp.send_command("8, 0, -1, wait turn")
					if get_track_distance(turning_point[0], turning_point[1])[0] == True:
						state = States.TURNING_STATE
						num_of_turn += 1
						print(num_of_turn)
						print("turning")
						time.sleep(0.25)
					continue
				
				#Turn 
				elif state == States.TURNING_STATE:
					angle = (track["center_x"]-320)/0.4
					if angle > 100:
						angle = 100
					if angle <-100:
						angle = -100
					esp.send_command("12, " + str(angle) +", -1, turn")
					if get_track_distance(sector_indi[is_clockwise][0], sector_indi[is_clockwise][1])[0] == True:
						if turn_time <= 0.1:
							num_of_turn -= 1
							print(num_of_turn)
							if num_of_turn > 0:
								state = States.RUN_SECTOR_STATE
								print("sector")
							else:
								state = States.FIRST_SECTOR
								print("first")
							turn_time = 0
							angle = 0
							continue 
						buffer = angle/5
						for i in range (4):
							angle -= buffer
							esp.send_command("12, " + str(angle) +", -1, turn")
							time.sleep(0.005)
						if num_of_turn == 12:
							state = States.LAST_RUN
							print("last")
						else:
							state = States.RUN_SECTOR_STATE
							print("sector")
						turn_time = 0
					time.sleep(0.025)
					turn_time += 0.025
					continue

					esp.send_command("8, " + str(is_clockwise*160-80) + ", -1, turn")
					time.sleep(2)
					state = States.DASH_AFTER_TURNING_STATE
					print("after")
					continue
				
				#Dash to pass the corner
				elif state == States.DASH_AFTER_TURNING_STATE:
					esp.send_command("8, 0, -1, go forward")
					time.sleep(0.25)
					if num_of_turn == 12:
						state = States.LAST_RUN
						print("last")
					else:
						state = States.RUN_SECTOR_STATE
						print("sector")
					continue
				
				#Run sector and keep a certain distance from the inner barrier
				elif state == States.RUN_SECTOR_STATE:                    
					if track["center_x"] != 0:
						angle_temp = (track["center_x"]-320)*abs(track["center_x"]-320)/300
						if angle_temp < 50 and angle_temp > -50:
							angle = angle_temp
					esp.send_command("12, " + str(angle) + ", -1, move forward")
					time.sleep(0.025)
					if get_track_distance(turn_indi_1[0], turn_indi_1[1])[0] == False and get_track_distance(turn_indi_2[0], turn_indi_2[1])[0] == True and track["center_x"] != 0:
						if turn_flag: 
							state = States.TURNING_STATE
							num_of_turn += 1
							print(num_of_turn)
							print("turning")
						else:
							turn_flag = 1
					else:
						turn_flag = 0
					continue

					if get_track_distance(track_right[is_clockwise][0], track_right[is_clockwise][1])[0] == is_clockwise:
						esp.send_command("8, 25, -1, move right")
						print("right")
					elif get_track_distance(track_left[is_clockwise][0], track_left[is_clockwise][1])[0] != is_clockwise:
						esp.send_command("8, -25, -1, move left")
						print("left")
					else:
						esp.send_command("8, 0, -1, go forward")
					if get_track_distance(turning_point[0], turning_point[1])[0] == True and get_track_distance(front_turning_point[0], front_turning_point[1])[0] == False:
						state = States.WAIT_TURN_STATE
						print("wait")
					continue
	
				#Last forward to stop
				elif state == States.LAST_RUN:
					time.sleep(0.5)
					esp.send_command("0, 0, 0,motor stop")
					break
					if track["center_x"] != 0:
						angle = (track["center_x"]-320)*abs(track["center_x"]-320)/60
					esp.send_command("10, " + str(angle) + ", -1, move forward")
					time.sleep(0.05)
					#Wait until the ending_point reach the wall in front of the robot
					if get_track_distance(ending_point[0], ending_point[1])[0] == False:
						esp.send_command("0, 0, 0, motor stop")
						end_time = time.perf_counter()
						print(end_time)
						break

				# End of OC1 FSM #
				time.sleep(0.005)
				
		elif run_OC2:
			esp.send_command("OC2")
			while run_OC2:
				reply = esp_replyNprint()
				if reply == "EOC2":
					run_OC2 = False
					break
				pillar, lot, track = get_latest_data()
				time.sleep(0.025)

				# OC2 FSM #
				#left = [240, 120] 
				#right = [400, 120]
				#while 1:
					#dl = get_track_distance(left[0], left[1])[1]
					#dr = get_track_distance(right[0], right[1])[1]
					#esp.send_command(str((dl + dr >= 0) * 16 - 8) + ", " + str(dl + dr) + ", -1, move")
				if lot["center_x"] > 0:
					if lot_count_flag:
						lot_count += 1
						print("lot " + str(lot_count))
						lot_count_flag = 0
						pillar_count_temp = pillar_count
				elif (pillar_count - 2) > pillar_count_temp:
					lot_count_flag = 1
				
				if lot_count == 4 and pillar_count % 3 == 0 and lot["center_y"] > 270:
					OC2_state = OC2_States.ENTER

				if OC2_state == OC2_States.LEAVE:
					if track["center_x"] < 320:
						is_clockwise = False						
					esp.send_command("8, " + str(is_clockwise * 200 - 100) + ", -1, turn")
					time.sleep(1.5)
					esp.send_command("-8, " + str(is_clockwise * -100 + 50) + ", -1, turn")
					time.sleep(0.7)
					#esp.send_command("6, " + str(is_clockwise * 200 - 100) + ", -1, turn")
					#time.sleep(0.75)
					#esp.send_command("-6, " + str(is_clockwise * -200 + 100) + ", -1, turn")
					#time.sleep(0.75)
					#esp.send_command("6, " + str(is_clockwise * 200 - 100) + ", -1, turn")
					#time.sleep(0.75)
					#esp.send_command("-6, " + str(is_clockwise * -200 + 100) + ", -1, turn")
					#time.sleep(0.75)
					OC2_state = OC2_States.WHITE
					print("white")
					continue

				if OC2_state == OC2_States.WHITE:
					if pillar["color"] != None:
						OC2_state = OC2_States.PILLAR
						print("pillar")
						continue
					angle = (track["center_x"] - 340 + is_clockwise * 40) / 0.3
					if angle > 80:
						speed = 9
					else:
						speed = 8
					esp.send_command(str(speed) + ", " + str(angle) + ", -1, move")
					continue

				if OC2_state == OC2_States.PILLAR:
					if pillar["color"] == None:
						OC2_state = OC2_States.WHITE
						print("white")
						continue
					angle = (pillar["center_x"] * 2 + ((pillar["color"] == "RED") * 2 - 1) * pillar["center_y"] - 620 - (pillar["color"] == "GREEN") * 50) / 1.8
					if angle > 80:
						speed = 9
					else:
						speed = 8
					esp.send_command(str(speed) + ", " + str(angle) + ", -1, move")
					if pillar["center_y"] > 270:
						if pillar_count_flag:
							pillar_count += 1
							print("pillar " + str(pillar_count))
							pillar_count_flag = 0
					elif pillar["center_y"] < 250:
						pillar_count_flag = 1
					continue
				
				if OC2_state == OC2_States.ENTER:
					esp.send_command("0, 0, 0, stop")
					run_OC2 = 0
					break

				# End of OC2 FSM #
				time.sleep(0.005)
				
		else:
			# IDLE
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
		time.sleep(0.05)
>>>>>>> 14854120d11683213666a55507ca8feeafb4b4b3

except KeyboardInterrupt:
	print("\nShutting down...")
	esp.disconnect()
	stop_vision_system()
	time.sleep(0.5)
