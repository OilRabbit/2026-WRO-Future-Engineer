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
full_sensor_res = camera.sensor_resolution
config = camera.create_video_configuration(main={"size": (640, 360), "format": "BGR888"},sensor={"output_size": full_sensor_res})
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
                                current_frame_time = time.perf_counter()
                                reply = esp_replyNprint()
                                if reply == "EOC1":
                                        print(f"\n[{state.name}][FSM ALERT] EOC1 Signal Received from ESP32. Terminating run immediately.")
                                        run_OC1 = False
                                        break

                                if state != last_state:
                                        print(f"\n==================================================")
                                        print(f"[{state.name}][FSM CHANGE] State Transition: {last_state} ===> {state}")
                                        print(f"==================================================\n")
                                        last_state = state

                                obstacle, parking, track = get_latest_data()

                                # --------------------------------------------------------------
                                # DYNAMIC LOCKOUT LOGIC: PURPLE WALL RANGE BOUNDARY WINDOW
                                # --------------------------------------------------------------
                                purple_area_check = (parking["width"] * parking["height"]) // 100

                                # Condition A: Purple wall is scanned on screen
                                if parking["center_x"] != 0 and purple_area_check >= MIN_PURPLE_AREA:
                                        if not purple_wall_lockout_active:
                                                print(f"[{state.name}][LOCKOUT ENGAGED] Purple wall entered frame. Pillar detection completely stopped.")
                                                purple_wall_lockout_active = True

                                # Condition B: Purple wall has completely gone out of the screen
                                elif parking["center_x"] == 0:
                                        if purple_wall_lockout_active:
                                                print(f"[{state.name}][LOCKOUT RELEASED] Purple wall went completely off screen. Restoring pillar detection.")
                                                purple_wall_lockout_active = False

                                # If the lockout state is active, suppress all pillar calculations
                                if purple_wall_lockout_active:
                                        obstacle["color"] = None
                                # --------------------------------------------------------------

                                # --------------------------------------------------------------
                                # FIRST SECTOR
                                # --------------------------------------------------------------
                                if state == States.FIRST_SECTOR:
                                        angle = calculate_oc2_steering_bias(track, obstacle, angle)
                                        print(f"[{state.name}][STEERING_DECISION] First Sector Output -> Speed: 8 | Combined Angle: {angle:.2f}")
                                        send_command_logged(f"8, {angle}, -1, move forward")
                                        time.sleep(0.025)

                                        indi1_inside = get_track_distance(turn_indi_1[0], turn_indi_1[1])[0]
                                        indi2_inside = get_track_distance(turn_indi_2[0], turn_indi_2[1])[0]

                                        if indi1_inside == False and indi2_inside == True and track["center_x"] != 0:
                                                print(f"[{state.name}][FSM_STAGE_1] Corner confirmation pass 1 triggered. Double-checking sensor alignment...")
                                                time.sleep(0.033)

                                                indi1_retry = get_track_distance(turn_indi_1[0], turn_indi_1[1])[0]
                                                indi2_retry = get_track_distance(turn_indi_2[0], turn_indi_2[1])[0]

                                                if indi1_retry == False and indi2_retry == True and track["center_x"] != 0:
                                                        if track["center_x"] < 320:
                                                                is_clockwise = False
                                                        print(f"[{state.name}][FSM_SUCCESS] Corner verified! Direction: {'CLOCKWISE' if is_clockwise else 'COUNTER-CLOCKWISE'}")
                                                        state = States.TURNING_STATE
                                                        num_of_turn += 1
                                                else:
                                                        print(f"[{state.name}][FSM_STAGE_1_FAILED] False corner read on pass 2. Resuming straight line run.")

                                # --------------------------------------------------------------
                                # WAIT TURN STATE
                                # --------------------------------------------------------------
                                elif state == States.WAIT_TURN_STATE:
                                        print(f"[{state.name}][WAIT_ALIGN] Executing delayed forward roll. Evaluating Turn Point Coordinate: {turning_point}")
                                        send_command_logged("8, 0, -1, wait turn")

                                        at_turning_node = get_track_distance(turning_point[0], turning_point[1])[0]
                                        print(f"[{state.name}][WAIT_ALIGN] Coordinate Target Status: {at_turning_node}")

                                        if at_turning_node == True:
                                                print(f"[{state.name}][WAIT_ALIGN] Target coordinate hit. Initializing Turn Sequence #{num_of_turn + 1}")
                                                state = States.TURNING_STATE
                                                num_of_turn += 1
                                                time.sleep(0.25)

                                # --------------------------------------------------------------
                                # TURNING STATE
                                # --------------------------------------------------------------
                                elif state == States.TURNING_STATE:
                                        base_turn_angle = (track["center_x"] - 320) / 0.4
                                        base_turn_angle = max(-100, min(100, base_turn_angle))
                                        angle = calculate_oc2_steering_bias(track, obstacle, base_turn_angle)

                                        print(f"[{state.name}][TURNING_ARC] Center Line Delta: {track['center_x'] - 320} | Final Arc Target Angle: {angle:.2f}")
                                        send_command_logged(f"8, {angle}, -1, turn")

                                        exit_landmark_detected = get_track_distance(sector_indi[0], sector_indi[1])[0]
                                        print(f"[{state.name}][TURNING_EXIT_CHECK] Sector Indicator Status: {exit_landmark_detected} | Elapsed Turn Time: {turn_time:.3f}s")

                                        if exit_landmark_detected == True:
                                                if turn_time <= 0.3:
                                                        print(f"[{state.name}][TURNING_REJECT] Corner landmark triggered too early ({turn_time:.3f}s). Overriding old FSM turn count revert.")
                                                        state = States.RUN_SECTOR_STATE
                                                        turn_time = 0
                                                        angle = 0
                                                        continue

                                                print(f"[{state.name}][TURNING_EXIT_CONFIRMED] Apex cleared at {turn_time:.3f}s. Initializing steering dampening routine.")
                                                buffer = angle / 5
                                                for i in range(4):
                                                        angle -= buffer
                                                        print(f"[{state.name}][TURNING_DAMPEN] Dampening Cycle Step {i+1}/4 -> Reduced Steering: {angle:.2f}")
                                                        send_command_logged(f"10, {angle}, -1, turn")
                                                        time.sleep(0.02)

                                                if lap_count_oc1 >= 3:
                                                        print(f"[{state.name}][FSM_COMPLETION] Objective loops cleared via Magenta Wall registry! Shifting to Stop Routine.")
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
                                        print(f"[{state.name}][DASH_RUN] Clearing intersection threshold. Straight run locking for 250ms.")
                                        send_command_logged("8, 0, -1, go forward")
                                        time.sleep(0.25)
                                        if lap_count_oc1 >= 3:
                                                state = States.LAST_RUN
                                        else:
                                                state = States.RUN_SECTOR_STATE

                                # --------------------------------------------------------------
                                # RUN SECTOR STATE
                                # --------------------------------------------------------------
                                elif state == States.RUN_SECTOR_STATE:
                                        angle = calculate_oc2_steering_bias(track, obstacle, angle)
                                        print(f"[{state.name}][STEERING_DECISION] Cruise Sector Output -> Speed: 12 | Combined Angle: {angle:.2f}")
                                        send_command_logged(f"12, {angle}, -1, move forward")
                                        time.sleep(0.025)

                                        # ---- MAGENTA CHROMATIC OVERLAY LAP COUNTER ----
                                        purple_x = parking["center_x"]
                                        purple_area = (parking["width"] * parking["height"]) // 100

                                        if purple_x != 0 and purple_area >= MIN_PURPLE_AREA:
                                                time_since_last_lap = current_frame_time - last_purple_lap_time

                                                if time_since_last_lap >= PURPLE_LAP_BUFFER:
                                                        lap_count_oc1 += 1
                                                        last_purple_lap_time = current_frame_time  # Set time-lock anchor

                                                        print(f"\n🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮")
                                                        print(f"[{state.name}][LAP REGISTERED] Magenta wall verified! Loop Count: [ {lap_count_oc1} / 3 ]")
                                                        print(f"[{state.name}][TIME LOCK] Guard frame window active for {PURPLE_LAP_BUFFER} seconds.")
                                                        print(f"🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮🔮\n")

                                                        if lap_count_oc1 >= 3:
                                                                print(f"[{state.name}][OC1 METRIC PASSED] Target loop requirement achieved. Swapping to stop sequence.")
                                                                state = States.LAST_RUN
                                                                continue
                                                else:
                                                        remaining_lockout = PURPLE_LAP_BUFFER - time_since_last_lap
                                                        print(f"[{state.name}][LAP BLOCK] Magenta marker active, but locked in debounce window. Shield: {remaining_lockout:.2f}s.")
                                        # --------------------------------------------------------

                                        # Keep scanning corner boundaries for tracking stability
                                        sector_indi1 = get_track_distance(turn_indi_1[0], turn_indi_1[1])[0]
                                        sector_indi2 = get_track_distance(turn_indi_2[0], turn_indi_2[1])[0]

                                        if sector_indi1 == False and sector_indi2 == True:
                                                print(f"[{state.name}][CRUISE_FSM] Upcoming corner landmark detected. Validating entry frame...")
                                                time.sleep(0.033)
                                                sector_indi1_retry = get_track_distance(turn_indi_1[0], turn_indi_1[1])[0]
                                                sector_indi2_retry = get_track_distance(turn_indi_2[0], turn_indi_2[1])[0]

                                                if sector_indi1_retry == False and sector_indi2_retry == True:
                                                        print(f"[{state.name}][CRUISE_FSM_SUCCESS] Corner entry verified. Control transferred to TURNING_STATE.")
                                                        state = States.TURNING_STATE

                                # --------------------------------------------------------------
                                # LAST RUN
                                # --------------------------------------------------------------
                                elif state == States.LAST_RUN:
                                        print(f"[{state.name}][FINAL_LANE] Target loops complete. Decelerating and centering vehicle for stop line...")
                                        time.sleep(0.5)
                                        print(f"[{state.name}][FINAL_STOP] Execution finished. Sending absolute zero brake code down serial line.")
                                        send_command_logged("0, 0, 0, motor stop")
                                        run_OC1 = False
                                        break

                                time.sleep(0.005)

                # ======================================================================
                # RUN OC2 BLOCK (Bypassed)
                # ======================================================================
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
