# testing
import time
import datetime
import math
from esp_com.communication import ESP32Communicator
from camera.camera_utils import start_vision_system, start_web_server, get_latest_data, stop_vision_system, get_track_distance
from picamera2 import Picamera2 as picam2
from enum import Enum

"""
================================================================================
WRO 2026 FUTURE ENGINEERS - SELF-DRIVING CARS
HIGH-SENSITIVITY MODULAR MASTER CONTROL PROGRAM
================================================================================
"""

# ESP communication and Picam init
print("======= Init =======")
esp = ESP32Communicator()
esp.connect()
camera = picam2()
config = camera.create_preview_configuration(main={"size": (640, 360), "format": "BGR888"})
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
        reply = esp.read_message()
        if reply is not None:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                print(f"{timestamp} ESP: {reply}")
                return reply
        return None

last_action_logged = ""
def send_command_logged(cmd_str):
        global last_action_logged
        esp.send_command(cmd_str)
        if cmd_str != last_action_logged:
                print(f"[ESP_OUT] Sent Command: {cmd_str}")
                last_action_logged = cmd_str

# ======================================================================
# HIGH-SENSITIVITY PERCEPTION ENGINE WITH EDGE DEADZONES
# ======================================================================
def calculate_oc2_steering_bias(track_data, obstacle_data, current_angle):
        """
        Calculates track centering baseline and layers an aggressive variable evasion
        offset proportional to the target area size for high-sensitivity avoidance.
        """
        target_angle = current_angle

        # ==========================================
        # TUNING KNOBS - ADJUST THESE FOR SENSITIVITY
        # ==========================================
        base_kick = 25.0        # Instant minimum steer when a pillar is valid
        gain_multiplier = 0.8  # Aggressiveness scaling of the area
        max_clamp = 75.0       # Maximum allowed steering angle deflection
        # ==========================================

        # 1. Base tracking profile (Lane Line Centering)
        if track_data["center_x"] != 0:
                target_angle = (track_data["center_x"] - 320) * abs(track_data["center_x"] - 320) / 60
                print(f"[VISION_TRACK] Base Center X: {track_data['center_x']} | Target Base Angle: {target_angle:.2f}")
        else:
                print("[VISION_WARN] Track Line Missing! Locking to last safe target angle.")

        # 2. Obstacle avoidance layer
        if obstacle_data["color"] in ["RED", "GREEN"]:
                cx = obstacle_data["center_x"]
                obstacle_area = (obstacle_data["width"] * obstacle_data["height"]) // 100

                # EDGE DETECTION FILTER: Ignore if the center_x falls within 0-30 or 610-640
                if (0 <= cx <= 30) or (610 <= cx <= 640):
                        print(f"[FILTER_IGNORE] Spotted {obstacle_data['color']} pillar at boundary edge (X: {cx}). Skipping processing.")
                        return target_angle

                print(f"[OBSTACLE_ALERT] Mid-lane Pillar Active! Color: {obstacle_data['color']} | Center X: {cx} | Area: {obstacle_area}")

                if obstacle_area > 20:
                        # HIGH-SENSITIVITY FORMULA
                        dynamic_offset = base_kick + (obstacle_area * gain_multiplier)
                        dynamic_offset = min(dynamic_offset, max_clamp)

                        if obstacle_data["color"] == "RED":
                                target_angle += dynamic_offset
                                print(f"[AVOIDANCE_ACTION!!] RED Pillar. HIGH SENSITIVITY Steering Bias RIGHT (+{dynamic_offset:.2f}).")
                        elif obstacle_data["color"] == "GREEN":
                                target_angle -= dynamic_offset
                                print(f"[AVOIDANCE_ACTION!!] GREEN Pillar. HIGH SENSITIVITY Steering Bias LEFT (-{dynamic_offset:.2f}).")
                else:
                        print("[AVOIDANCE_SKIP] Distance gap safe. Skipping bias modifier adjustment.")

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
                                is_clockwise = True
                                turning_point = right_turning_point
                                state = States.FIRST_SECTOR
                                last_state = None
                                print("FIRST_SECTOR - Reset Complete")
                        reset_OC = False
                        print("Resetted")
                        continue

                # ======================================================================
                # RUN OC1 BLOCK (RUNNING HIGH-SENSITIVITY LOGIC OVER TRADITIONAL SERIAL)
                # ======================================================================
                elif run_OC1:
                        send_command_logged("OC1")
                        start_time = time.perf_counter()
                        while run_OC1:
                                end_time = time.perf_counter()
                                reply = esp_replyNprint()
                                if reply == "EOC1":
                                        print("\n[FSM ALERT] EOC1 Signal Received from ESP32. Terminating run immediately.")
                                        run_OC1 = False
                                        break

                                if state != last_state:
                                        print(f"\n==================================================")
                                        print(f"[FSM CHANGE] State Transition: {last_state} ===> {state}")
                                        print(f"==================================================\n")
                                        last_state = state

                                obstacle, parking, track = get_latest_data()

                                # --------------------------------------------------------------
                                # FIRST SECTOR (Initial straightaway layout & wall evaluation)
                                # --------------------------------------------------------------
                                if state == States.FIRST_SECTOR:
                                        angle = calculate_oc2_steering_bias(track, obstacle, angle)

                                        print(f"[STEERING_DECISION] First Sector Output -> Speed: 12 | Combined Angle: {angle:.2f}")
                                        send_command_logged(f"12, {angle}, -1, move forward")
                                        time.sleep(0.025)

                                        indi1_inside = get_track_distance(turn_indi_1[0], turn_indi_1[1])[0]
                                        indi2_inside = get_track_distance(turn_indi_2[0], turn_indi_2[1])[0]

                                        if indi1_inside == False and indi2_inside == True and track["center_x"] != 0:
                                                print("[FSM_STAGE_1] Corner confirmation pass 1 triggered. Double-checking sensor alignment...")
                                                time.sleep(0.033)

                                                indi1_retry = get_track_distance(turn_indi_1[0], turn_indi_1[1])[0]
                                                indi2_retry = get_track_distance(turn_indi_2[0], turn_indi_2[1])[0]

                                                if indi1_retry == False and indi2_retry == True and track["center_x"] != 0:
                                                        if track["center_x"] < 320:
                                                                is_clockwise = False
                                                        print(f"[FSM_SUCCESS] Corner verified! Direction: {'CLOCKWISE' if is_clockwise else 'COUNTER-CLOCKWISE'}")
                                                        state = States.TURNING_STATE
                                                        num_of_turn += 1
                                                        print(f"[TRACKING] Current Session Turn Count: {num_of_turn}")
                                                else:
                                                        print("[FSM_STAGE_1_FAILED] False corner read on pass 2. Resuming straight line run.")

                                # --------------------------------------------------------------
                                # WAIT TURN STATE (Time-controlled entry alignment)
                                # --------------------------------------------------------------
                                elif state == States.WAIT_TURN_STATE:
                                        print(f"[WAIT_ALIGN] Executing delayed forward roll. Evaluating Turn Point Coordinate: {turning_point}")
                                        send_command_logged("8, 0, -1, wait turn")

                                        at_turning_node = get_track_distance(turning_point[0], turning_point[1])[0]
                                        print(f"[WAIT_ALIGN] Coordinate Target Status: {at_turning_node}")

                                        if at_turning_node == True:
                                                print(f"[WAIT_ALIGN] Target coordinate hit. Initializing Turn Sequence #{num_of_turn + 1}")
                                                state = States.TURNING_STATE
                                                num_of_turn += 1
                                                time.sleep(0.25)

                                # --------------------------------------------------------------
                                # TURNING STATE (Tracking arc rotation)
                                # --------------------------------------------------------------
                                elif state == States.TURNING_STATE:
                                        angle = (track["center_x"] - 320) / 0.4
                                        angle = max(-100, min(100, angle))
                                        print(f"[TURNING_ARC] Center Line Delta: {track['center_x'] - 320} | Arc Target Angle: {angle:.2f}")
                                        send_command_logged(f"8, {angle}, -1, turn")

                                        exit_landmark_detected = get_track_distance(sector_indi[0], sector_indi[1])[0]
                                        print(f"[TURNING_EXIT_CHECK] Sector Indicator Status: {exit_landmark_detected} | Ellapsed Turn Time: {turn_time:.3f}s")

                                        if exit_landmark_detected == True:
                                                if turn_time <= 0.3:
                                                        print(f"[TURNING_REJECT] Corner landmark triggered too early ({turn_time:.3f}s). Reverting turn count registry.")
                                                        num_of_turn -= 1
                                                        state = States.RUN_SECTOR_STATE
                                                        turn_time = 0
                                                        angle = 0
                                                        continue

                                                print(f"[TURNING_EXIT_CONFIRMED] Apex cleared at {turn_time:.3f}s. Initializing steering dampening routine.")
                                                buffer = angle / 5
                                                for i in range(4):
                                                        angle -= buffer
                                                        print(f"[TURNING_DAMPEN] Dampening Cycle Step {i+1}/4 -> Reduced Steering: {angle:.2f}")
                                                        send_command_logged(f"10, {angle}, -1, turn")
                                                        time.sleep(0.02)

                                                if num_of_turn == 12:
                                                        print("[FSM_COMPLETION] Turn #12 registered! Redirecting to Final Stop Run Line.")
                                                        state = States.LAST_RUN
                                                else:
                                                        state = States.RUN_SECTOR_STATE
                                                turn_time = 0
                                        else:
                                                time.sleep(0.025)
                                                turn_time += 0.025

                                # --------------------------------------------------------------
                                # DASH AFTER TURNING STATE
                                # --------------------------------------------------------------
                                elif state == States.DASH_AFTER_TURNING_STATE:
                                        print(f"[DASH_RUN] Clearing intersection threshold. Straight run locking for 250ms.")
                                        send_command_logged("8, 0, -1, go forward")
                                        time.sleep(0.25)

                                        if num_of_turn == 12:
                                                print("[FSM_COMPLETION] Final Dash Complete. Redirecting to Final Stop Run Line.")
                                                state = States.LAST_RUN
                                        else:
                                                state = States.RUN_SECTOR_STATE

                                # --------------------------------------------------------------
                                # RUN SECTOR STATE (High-Speed Obstacle & Path Processing)
                                # --------------------------------------------------------------
                                elif state == States.RUN_SECTOR_STATE:
                                        angle = calculate_oc2_steering_bias(track, obstacle, angle)

                                        print(f"[STEERING_DECISION] Cruise Sector Output -> Speed: 12 | Combined Angle: {angle:.2f}")
                                        send_command_logged(f"12, {angle}, -1, move forward")
                                        time.sleep(0.025)

                                        sector_indi1 = get_track_distance(turn_indi_1[0], turn_indi_1[1])[0]
                                        sector_indi2 = get_track_distance(turn_indi_2[0], turn_indi_2[1])[0]

                                        if sector_indi1 == False and sector_indi2 == True:
                                                print("[CRUISE_FSM] Upcoming corner landmark detected on pass 1. Validating entry frame...")
                                                time.sleep(0.033)

                                                sector_indi1_retry = get_track_distance(turn_indi_1[0], turn_indi_1[1])[0]
                                                sector_indi2_retry = get_track_distance(turn_indi_2[0], turn_indi_2[1])[0]

                                                if sector_indi1_retry == False and sector_indi2_retry == True:
                                                        print(f"[CRUISE_FSM_SUCCESS] Corner entry verified. Transferring control to TURNING_STATE.")
                                                        state = States.TURNING_STATE
                                                        num_of_turn += 1
                                                        print(f"[TRACKING] Current Session Turn Count: {num_of_turn}/12")
                                                else:
                                                        print("[CRUISE_FSM_FALSE] Intersection validation failed on pass 2. Disregarding.")

                                # --------------------------------------------------------------
                                # LAST RUN (Final Lane Stretch and Stop Routine)
                                # --------------------------------------------------------------
                                elif state == States.LAST_RUN:
                                        print("[FINAL_LANE] Core loops complete. Decelerating and centering vehicle for stop line...")
                                        time.sleep(0.5)
                                        print("[FINAL_STOP] Execution finished. Sending absolute zero brake code down serial line.")
                                        send_command_logged("0, 0, 0, motor stop")
                                        break

                                time.sleep(0.005)

                elif run_OC2:
                        send_command_logged("OC2")
                        while run_OC2:
                                reply = esp_replyNprint()
                                if reply == "EOC2":
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
