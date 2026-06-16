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

# ======================================================================
# METHOD 1: CRITICAL HIERARCHICAL BUMPER PERCEPTION ENGINE
# ======================================================================
def calculate_oc2_steering_bias(track_data, obstacle_data, current_angle, current_state, clockwise_mode):
        """
        Enforces a strict priority hierarchy:
        1. Corner State Lock (Absolute Highest)
        2. Critical Wall Crash Zone (Overrides Pillars)
        3. Thickened Wall Hazard Zone (Overrides Pillars)
        4. Pillar/Obstacle Avoidance Layer (Only runs when boundaries are safe)
        """
        MAX_EMERGENCY_STEER = 85.0 
        
        # PRIORITY 1: FORCE THE CORNER DIRECTION IMMEDIATELY IF CORNER STATE IS ENGAGED
        if current_state == States.TURNING_STATE:
                if clockwise_mode:
                        print("[DIRECTION_LOCK] Corner active: Locking full steering RIGHT (+85).")
                        return MAX_EMERGENCY_STEER
                else:
                        print("[DIRECTION_LOCK] Corner active: Locking full steering LEFT (-85).")
                        return -MAX_EMERGENCY_STEER

        # Tuning Parameters
        CRITICAL_CRASH_Y  = 240   
        SOFT_HAZARD_Y     = 150   
        SOFT_DRIFT_ANGLE  = 35.0  
        
        base_kick = 30.0          
        gain_multiplier = 0.8
        deadzone_weigh = 3.0

        # PRIORITY 2 & 3: EVALUATE ABSOLUTE BOUNDARY SAFETY FIRST
        if track_data["center_x"] != 0 and track_data["center_y"] != 0:
                cx = track_data["center_x"]
                cy = track_data["center_y"]

                # LAYER A: CRITICAL EMERGENCY OVERRIDE ZONE (Immediate return, bypasses pillar algorithm)
                if cy >= CRITICAL_CRASH_Y:
                        print(f"[CRITICAL BOUNDARY PRIORITY] Wall deep in bumper! X: {cx}, Y: {cy}. Ignoring pillar logic.")
                        if cx < 320:
                                return MAX_EMERGENCY_STEER
                        else:
                                return -MAX_EMERGENCY_STEER
                
                # LAYER B: THICKENED SOFT HAZARD ZONE (Immediate return, bypasses pillar algorithm)
                elif cy >= SOFT_HAZARD_Y and current_state != States.FIRST_SECTOR:
                        print(f"[HAZARD BOUNDARY PRIORITY] Wall near bumper. X: {cx}, Y: {cy}. Ignoring pillar logic.")
                        if cx < 320:
                                return SOFT_DRIFT_ANGLE
                        else:
                                return -SOFT_DRIFT_ANGLE

        # PRIORITY 4: PILLAR AVOIDANCE LAYER (Only executes if the boundary layers above did not trigger)
        target_angle = current_angle

        if obstacle_data["color"] in ["RED", "GREEN"]:
                ocx = obstacle_data["center_x"]
                obstacle_area = (obstacle_data["width"] * obstacle_data["height"]) // 100

                # Deadzone safety check
                if not ((0 <= ocx <= deadzone_weigh) or (640-deadzone_weigh <= ocx <= 640)):
                        if obstacle_area > 20:
                                dynamic_offset = base_kick + (obstacle_area * gain_multiplier)
                                dynamic_offset = min(dynamic_offset, MAX_EMERGENCY_STEER)

                                if obstacle_data["color"] == "RED":
                                        target_angle += dynamic_offset
                                        print(f"[AVOIDANCE] Track safe. Evading RED Pillar: Bias (+{dynamic_offset:.2f})")
                                elif obstacle_data["color"] == "GREEN":
                                        target_angle -= dynamic_offset
                                        print(f"[AVOIDANCE] Track safe. Evading GREEN Pillar: Bias (-{dynamic_offset:.2f})")

        return max(-MAX_EMERGENCY_STEER, min(MAX_EMERGENCY_STEER, target_angle))

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
                # RUN OC1 BLOCK
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
                                # FIRST SECTOR
                                # --------------------------------------------------------------
                                if state == States.FIRST_SECTOR:
                                        angle = calculate_oc2_steering_bias(track, obstacle, angle, state, is_clockwise)

                                        print(f"[STEERING_DECISION] First Sector Output -> Speed: 9 | Combined Angle: {angle:.2f}")
                                        send_command_logged(f"9, {angle}, -1, move forward")
                                        time.sleep(0.025)

                                        indi1_inside = get_track_distance(turn_indi_1[0], turn_indi_1[1])[0]
                                        indi2_inside = get_track_distance(turn_indi_2[0], turn_indi_2[1])[0]

                                        if indi1_inside == False and indi2_inside == True and track["center_x"] != 0:
                                                time.sleep(0.010)
                                                indi1_retry = get_track_distance(turn_indi_1[0], turn_indi_1[1])[0]
                                                indi2_retry = get_track_distance(turn_indi_2[0], turn_indi_2[1])[0]

                                                if indi1_retry == False and indi2_retry == True and track["center_x"] != 0:
                                                        if track["center_x"] < 320:
                                                                is_clockwise = False
                                                        print(f"[FSM_SUCCESS] Corner verified! Direction: {'CLOCKWISE' if is_clockwise else 'COUNTER-CLOCKWISE'}")
                                                        state = States.TURNING_STATE
                                                        num_of_turn += 1
                                                        
                                                        print(f"\n**************************************************")
                                                        print(f">>>>>>>> [ TURN COUNT REGISTERED: {num_of_turn} / 12 ] <<<<<<<<")
                                                        print(f"**************************************************\n")

                                # --------------------------------------------------------------
                                # WAIT TURN STATE
                                # --------------------------------------------------------------
                                elif state == States.WAIT_TURN_STATE:
                                        send_command_logged("9, 0, -1, wait turn")
                                        at_turning_node = get_track_distance(turning_point[0], turning_point[1])[0]
                                        if at_turning_node == True:
                                                state = States.TURNING_STATE
                                                num_of_turn += 1
                                                time.sleep(0.25)

                                # --------------------------------------------------------------
                                # TURNING STATE
                                # --------------------------------------------------------------
                                elif state == States.TURNING_STATE:
                                        angle = calculate_oc2_steering_bias(track, obstacle, angle, state, is_clockwise)
                                        
                                        print(f"[TURNING_ARC] Locked Steering Turn Mode Engine Target Angle: {angle:.2f}")
                                        send_command_logged(f"9, {angle}, -1, turn")

                                        exit_landmark_detected = get_track_distance(sector_indi[0], sector_indi[1])[0]
                                        
                                        if exit_landmark_detected == True:
                                                if turn_time <= 0.10:
                                                        print(f"[TURNING_REJECT] Corner landmark triggered too early ({turn_time:.3f}s). Reverting.")
                                                        num_of_turn -= 1
                                                        state = States.RUN_SECTOR_STATE
                                                        turn_time = 0
                                                        angle = 0
                                                        continue

                                                print(f"[TURNING_EXIT_CONFIRMED] Apex cleared at {turn_time:.3f}s. Initializing steering dampening routine.")
                                                buffer = angle / 5
                                                for i in range(4):
                                                        angle -= buffer
                                                        send_command_logged(f"11, {angle}, -1, turn")
                                                        time.sleep(0.02)

                                                if num_of_turn == 12:
                                                        state = States.LAST_RUN
                                                else:
                                                        state = States.RUN_SECTOR_STATE
                                                turn_time = 0
                                        else:
                                                if turn_time > 1.5:
                                                        print("[FSM_EMERGENCY] Turn timeout breached! Forcing recovery to cruise state.")
                                                        state = States.RUN_SECTOR_STATE
                                                        turn_time = 0
                                                else:
                                                        time.sleep(0.025)
                                                        turn_time += 0.025

                                # --------------------------------------------------------------
                                # DASH AFTER TURNING STATE
                                # --------------------------------------------------------------
                                elif state == States.DASH_AFTER_TURNING_STATE:
                                        send_command_logged("9, 0, -1, go forward")
                                        time.sleep(0.25)
                                        if num_of_turn == 12:
                                                state = States.LAST_RUN
                                        else:
                                                state = States.RUN_SECTOR_STATE

                                # --------------------------------------------------------------
                                # RUN SECTOR STATE (Hierarchical Priority Maintained Here)
                                # --------------------------------------------------------------
                                elif state == States.RUN_SECTOR_STATE:
                                        if track["center_x"] != 0:
                                                error_x = track["center_x"] - 320
                                                base_tracking_angle = error_x * 0.15
                                        else:
                                                base_tracking_angle = 0.0

                                        # Safety loops inside calculate_oc2_steering_bias will now instantly override base_tracking_angle
                                        angle = calculate_oc2_steering_bias(track, obstacle, base_tracking_angle, state, is_clockwise)

                                        print(f"[STEERING_DECISION] Cruise Sector Output -> Speed: 12 | Combined Angle: {angle:.2f}")
                                        send_command_logged(f"12, {angle}, -1, move forward")
                                        time.sleep(0.025)

                                        sector_indi1 = get_track_distance(turn_indi_1[0], turn_indi_1[1])[0]
                                        sector_indi2 = get_track_distance(turn_indi_2[0], turn_indi_2[1])[0]

                                        if sector_indi1 == False and sector_indi2 == True:
                                                time.sleep(0.010)
                                                sector_indi1_retry = get_track_distance(turn_indi_1[0], turn_indi_1[1])[0]
                                                sector_indi2_retry = get_track_distance(turn_indi_2[0], turn_indi_2[1])[0]

                                                if sector_indi1_retry == False and sector_indi2_retry == True:
                                                        state = States.TURNING_STATE
                                                        num_of_turn += 1
                                                        print(f"\n**************************************************")
                                                        print(f">>>>>>>> [ TURN COUNT REGISTERED: {num_of_turn} / 12 ] <<<<<<<<")
                                                        print(f"**************************************************\n")

                                # --------------------------------------------------------------
                                # LAST RUN
                                # --------------------------------------------------------------
                                elif state == States.LAST_RUN:
                                        time.sleep(0.5)
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
