# testing
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


# Checkpoints (default as clockwise case)
front_point = [200, 80]
add_marker_point("Front Turning Point", front_point[0], front_point[1], color=(0, 0, 255), radius=2, label="Front P")
left_turning_point = [5, 200] #check direction
add_marker_point("Left Turning Point", left_turning_point[0], left_turning_point[1], color=(0, 0, 255), radius=2, label="Left TP")
right_turning_point = [395, 200] #check direction
add_marker_point("Right Turning Point", right_turning_point[0], right_turning_point[1], color=(0, 0, 255), radius=2, label="Right TP")
clockwise_indicator = [395, 135]
add_marker_point("Clockwise Indicator", clockwise_indicator[0], clockwise_indicator[1], color=(0, 0, 255), radius=2, label="C Indi")
anticlockwise_indicator = [5, 135]
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
                                clockwise_indicator = [395, 135]
                                add_marker_point("Clockwise Indicator", clockwise_indicator[0], clockwise_indicator[1], color=(0, 0, 255), radius=2, label="C Indi")
                                anticlockwise_indicator = [5, 135]
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
                                		start_turning_time = time.perf_counter_ns()
                                		state = States.TURNING_STATE
                                		continue
                                	else:
                                        	time.sleep(0.001)
                                        	continue

                                # Turn
                                elif state == States.TURNING_STATE:
                                	steering = 80 if is_clockwise else -80
                                	esp.send_command(str(speed) + ", " + str(steering) + ", -1, turn")
                                	if (time.perf_counter_ns() - start_turning_time) / 1000000 < 1100:
                                		print((time.perf_counter_ns() - start_turning_time) / 1000000)
                                		continue
                                        # state = States.INIT
                                        # esp.send_command("0, 0, -1, stop")
                                        # break
                                        # indicator = clockwise_indicator if is_clockwise else anticlockwise_indicator
                                        # if get_track_distance(indicator[0], indicator[1])[0]:
                                        # 	time.sleep(0.001)
                                        # 	continue
                                        # overshoot_point = [260, 100] if is_clockwise else [140, 100]
                                        # add_marker_point("Overshoot Point", overshoot_point[0], overshoot_point[1], color=(0, 0, 255), radius=2, label="Overshoot P")
                                        # steering = -80 if is_clockwise else 80
                                        # esp.send_command(str(speed) + ", 0, -1, go forward")
                                	num_of_turn += 1
                                	print("num turn: {}".format(num_of_turn))
                                	state = States.DASH_AFTER_TURNING_STATE if num_of_turn < 12 else States.LAST_RUN
                                	continue

                                elif state == States.DASH_AFTER_TURNING_STATE:
                                	esp.send_command(str(speed) + ", 0, -1, forward")
                                	time.sleep(0.5)
                                	state = States.RUN_SECTOR_STATE
                                	continue

                                # Run sector and keep a certain distance from the inner barrier
                                elif state == States.RUN_SECTOR_STATE:
                                	esp.send_command(str(speed) + ", 0, -1, forward")
                                	if get_track_distance(front_point[0], front_point[1])[0] == False and (get_track_distance(left_turning_point[0], left_turning_point[1])[0] == True or get_track_distance(right_turning_point[0], right_turning_point[1])[0] == True):
                                        	state = States.TURNING_STATE
                                        	start_turning_time = time.perf_counter_ns()
                                        	continue
                                	else:
                                        	if not get_track_distance(innerwall_white[0], innerwall_white[1])[0]:
                                        		steering = -50 if is_clockwise else 50
                                        	elif get_track_distance(innerwall_black[0], innerwall_black[1])[0]:
                                        		steering = 5 if is_clockwise else -5
                                        	else:
                                        		steering = 0
                                        		esp.send_command(str(speed) + ", " + str(steering) + ", -1, go forward")
                                        		continue

                                # Last forward to stop
                                elif state == States.LAST_RUN:
                                	end_sector_point = [250, 75] if is_clockwise else [150, 75]
                                	add_marker_point("End Sector Point", end_sector_point[0], end_sector_point[1], color=(0, 0, 255), radius=2, label="Stop P")
                                	esp.send_command(str(speed) + ", 0, -1, move forward")
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
