# testing
import time
import datetime
import math
from esp_com.communication import ESP32Communicator
from camera.camera_utils import start_vision_system, start_web_server, get_latest_data, stop_vision_system, get_track_distance
from picamera2 import Picamera2 as picam2
from enum import Enum

# ESP communication and Picam init
print("======= Init =======")
esp = ESP32Communicator()
esp.connect()
camera = picam2()
video_size = (640, 360)
sensor_video_size = (2304, 1296)
target_frame_duration_us = 33333
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

start_vision_system(camera, True)

live_streaming = True
if live_streaming:
        print("Streaming: Activated")
        start_web_server(host='0.0.0.0', port=5000)
else:
        print("Streaming: Not streaming")

print("======= End of Init =======")

# Global tracking variables
run_OC1 = False
run_OC2 = False
reset_OC = True
num_of_turn = 0
lap_count_oc1 = 0  # Track completed loops via purple wall signatures
is_clockwise = True
start_time = 0
end_time = 0
recorded_time = 0

# Checkpoints & Indicators
angle = 0
turn_indi_1 = [320, 30]
turn_indi_2 = [320, 60]
sector_indi = [320, 50]
turn_time = 0

# Purple Wall Detector Config & Anti-Double-Count Time Locks
last_purple_lap_time = 0.0
PURPLE_LAP_BUFFER = 3.0
MIN_PURPLE_AREA = 25                # Minimum contour area scaled size to confirm it's the wall

# Persistent lockout flag for tracking when purple wall is actively on screen
purple_wall_lockout_active = False

front_turning_point = [320, 70]
left_turning_point = [40, 200]
right_turning_point = [600, 200]
turning_point = right_turning_point
track_left = [[80, 180], [520, 180]]
track_right = [[120, 180], [560, 180]]
ending_point = [320, 30]

# States
class States(Enum):
        INIT = 0
        FIRST_SECTOR = 1
        WAIT_TURN_STATE = 2
        TURNING_STATE = 3
        DASH_AFTER_TURNING_STATE = 4
        RUN_SECTOR_STATE = 5
        LAST_RUN = 6

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

def calculate_oc2_steering_bias(track_data, obstacle_data, current_angle):
        if not hasattr(calculate_oc2_steering_bias, "was_dodging"):
                calculate_oc2_steering_bias.was_dodging = False
                calculate_oc2_steering_bias.last_dodge_dir = None

        target_angle = current_angle

        base_kick = 30.0
        gain_multiplier = 0.8
        max_clamp = 85.0
        deadzone_weigh = 3.0

        COUNTER_STEER_FORCE = 25.0
        COUNTER_STEER_TIME = 0.08

        pillar_detected_this_frame = False

        if obstacle_data["color"] in ["RED", "GREEN"]:
                cx = obstacle_data["center_x"]
                obstacle_area = (obstacle_data["width"] * obstacle_data["height"]) // 100

                if (0 <= cx <= deadzone_weigh) or (640-deadzone_weigh <= cx <= 640):
                        print(f"[FILTER_IGNORE] Spotted {obstacle_data['color']} pillar at boundary edge (X: {cx}). Skipping processing.")
                        return target_angle

                print(f"[OBSTACLE_ALERT] Mid-lane Pillar Active! Color: {obstacle_data['color']} | Center X: {cx} | Area: {obstacle_area}")

                if obstacle_area > 20:
                        pillar_detected_this_frame = True
                        dynamic_offset = base_kick + (obstacle_area * gain_multiplier)
                        dynamic_offset = min(dynamic_offset, max_clamp)

                        if obstacle_data["color"] == "RED":
                                target_angle += dynamic_offset
                                calculate_oc2_steering_bias.was_dodging = True
                                calculate_oc2_steering_bias.last_dodge_dir = "RIGHT"
                                print(f"[AVOIDANCE_ACTION!!] RED Pillar. Steering Bias RIGHT (+{dynamic_offset:.2f}).")
                        elif obstacle_data["color"] == "GREEN":
                                target_angle -= dynamic_offset
                                calculate_oc2_steering_bias.was_dodging = True
                                calculate_oc2_steering_bias.last_dodge_dir = "LEFT"
                                print(f"[AVOIDANCE_ACTION!!] GREEN Pillar. Steering Bias LEFT (-{dynamic_offset:.2f}).")
                else:
                        print("[AVOIDANCE_SKIP] Distance gap safe. Skipping bias modifier adjustment.")

        if not pillar_detected_this_frame and calculate_oc2_steering_bias.was_dodging:
                print(f"[COUNTER-STEER RECOVERY] Pillar cleared! Executing stabilization correction.")

                if calculate_oc2_steering_bias.last_dodge_dir == "RIGHT":
                        recovery_angle = -COUNTER_STEER_FORCE
                        target_angle += recovery_angle
                        print(f" -> counter action: Pulsing LEFT ({recovery_angle}) to catch alignment.")
                else:
                        recovery_angle = COUNTER_STEER_FORCE
                        target_angle += recovery_angle
                        print(f" -> counter action: Pulsing RIGHT (+{recovery_angle}) to catch alignment.")

                calculate_oc2_steering_bias.was_dodging = False
                calculate_oc2_steering_bias.last_dodge_dir = None

        return target_angle

state = States.INIT
last_state = None

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
                                if hasattr(calculate_oc2_steering_bias, "was_dodging"):
                                        calculate_oc2_steering_bias.was_dodging = False
                                        calculate_oc2_steering_bias.last_dodge_dir = None
                                print(f"[{state.name}] FIRST_SECTOR - Reset Complete")
                        reset_OC = False
                        print(f"[{state.name}] Resetted")
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

                                _, _, track = get_latest_data()

                                if state == States.FIRST_SECTOR:
					esp.send_command("10, 0, -1, go forward")
					while get_track_distance(left_turning_point[0], left_turning_point[1])[0] == False and get_track_distance(right_turning_point[0], right_turning_point[1])[0] == False:
						time.sleep(0.005)
					if get_track_distance(left_turning_point[0], left_turning_point[1])[0] == True:
						is_clockwise = False
						turning_point = left_turning_point
						track_left = [0, 360]
						track_right = [40, 360]
						state = WAIT_TURN_STATE
					continue
								
				#Wait turn
				elif state == WAIT_TURN_STATE:
					esp.send_command("10, 0, 100, wait turn")
					state = TURNING_STATE
					continue
				
				#Turn 
				elif state == TURNING_STATE:
					num_of_turn += 1
					esp.send_command("10, 70, 200, turn")
					state = DASH_AFTER_TURNING_STATE
					continue
				
				#Dash to pass the corner
				elif state == DASH_AFTER_TURING_STATE:
					esp.send_command("10, 0, 100, go forward")
					if num_of_turn == 12:
						state = LAST_RUN
					else:
						state = RUN_SECTOR_STATE
					continue
				
				#Run sector and keep a certain distance from the inner barrier
				elif state == RUN_SECTOR_STATE:
					if (get_track_distance(track_right[0], track_right[1])[0] == True and is_clockwise) or (get_track_distance(track_left[0], track_left[1])[0] == True and not is_clockwise):
						esp.send_command("10, 10, -1, move right")
					elif (get_track_distance(track_left[0], track_left[1])[0] == False and is_clockwise) or (get_track_distance(track_right[0], track_right[1])[0] == False and not is_clockwise):
						esp.send_command("10, 10, -1, move left")
					if get_track_distance(turning_point[0], turning_point[1])[0] == True:
						state = WAIT_TURN_STATE
					continue
	
				#Last forward to stop
				elif state == LAST_RUN:
					esp.send_command("10, 0, -1, move forward")
					#Wait until the ending_point reach the wall in front of the robot
					while get_track_distance(ending_point[0], ending_point[1])[0] == True:
						time.sleep(0.005)
					esp.send_command("0, 0, 0, motor stop")
					end_time = time.pref_counter()
					break
	
				# End of OC1 FSM #
				time.sleep(0.005)
                elif run_OC2:
                        send_command_logged("OC2")
                        while run_OC2:
                                reply = esp_replyNprint()
                                if reply == "EOC2":
                                        stop_vehicle("OC2 stop requested by ESP32")
                                        run_OC2 = False
                                        break
                                time.sleep(0.005)

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
                time.sleep(0.05)

except KeyboardInterrupt:
        print("\nShutting down software pipeline gracefully...")
        try:
                esp.send_command("0, 0, 0, emergency_stop")
        except Exception:
                pass
        esp.disconnect()
        stop_vision_system()
        time.sleep(0.5)
