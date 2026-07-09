# testing
import time
import datetime
from esp_com.communication import ESP32Communicator
from camera.camera_utils import start_vision_system, start_web_server, get_latest_data, stop_vision_system, get_track_distance
from picamera2 import Picamera2 as picam2
from enum import Enum

"""
================================================================================
WRO 2026 FUTURE ENGINEERS - SELF-DRIVING CARS
CORE CHALLENGE 1 (OC1) - EDGE DISTANCE SCANNING DIRECTION LOGIC
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

# Global variables
run_OC1 = False
run_OC2 = False
reset_OC = True
num_of_turn = 0
is_clockwise = True
start_time = 0
end_time = 0
recorded_time = 0

angle = 0  # steering percentage
turn_indi_1 = [320, 30]  # check when to turn
turn_indi_2 = [320, 60]  # check when to turn
sector_indi = [320, 50]  # check when is sector
turn_time = 0

# GLOBAL SPEED VARIABLE
speed = 18

# PD Tracking Memory for stabilizing the ultra-tight inner wall path
last_track_error = 0.0

left_turning_point = [40, 180]
right_turning_point = [600, 180]
turning_point = right_turning_point

# CENTRAL VERTICAL SCURTAIN PARAMETERS (For see-front-wall array)
FRONT_SCAN_Y_START = 140
FRONT_SCAN_Y_END = 360
FRONT_SCAN_STEP = 5
FIXED_FRONT_COLUMNS = [240, 320, 400] # Checks left, center, right columns ahead

# HORIZONTAL SCAN SCANNING ROW FOR DIRECTION
INIT_SCAN_Y = 220 

# SAFETY BOUNDARIES (15 Pixels from Screen Margins)
track_left = [40, 280]
track_right = [600, 280]

# Retained early-reaction finish line configuration
ending_point = [320, 280]

# States
class States(Enum):
    INIT = 0
    FIRST_SECTOR = 1
    WAIT_TURN_STATE = 2
    TURNING_STATE = 3
    DASH_AFTER_TURNING_STATE = 4
    RUN_SECTOR_STATE = 5
    LAST_RUN = 6

# Dynamic multi-line vertical range check for front wall detection
def check_front_wall_range(columns_to_scan):
    for x_pos in columns_to_scan:
        for y in range(FRONT_SCAN_Y_START, FRONT_SCAN_Y_END + 1, FRONT_SCAN_STEP):
            if get_track_distance(x_pos, y)[0]:
                return True
    return False

# Scan from left edge (0) inward to find where the left wall starts
def get_left_wall_distance():
    for x in range(0, 320, 10):
        if get_track_distance(x, INIT_SCAN_Y)[0]:
            return x  # Returns pixel distance from left edge
    return 320

# Scan from right edge (640) inward to find where the right wall starts
def get_right_wall_distance():
    for x in range(640, 320, -10):
        if get_track_distance(x, INIT_SCAN_Y)[0]:
            return (640 - x)  # Returns pixel distance from right edge
    return 320

# Hardened function for receiving messages from ESP, intercepting connection dropped issues safely
def esp_replyNprint():
    global esp
    try:
        reply = esp.read_message()
        if reply is not None:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            print(f"[{timestamp}] [ESP_IN] Received: {reply}")
            return reply
    except (OSError, Exception) as e:
        print(f"\n[SERIAL WARNING] Caught hardware connection glitch: {e}")
        print("Attempting to bypass frame drop and reconnect to ESP32...")
        try:
            esp.disconnect()
            time.sleep(0.1)
            esp.connect()
            print("[SERIAL SUCCESS] Reconnected successfully!")
        except Exception as recon_err:
            print(f"[SERIAL CRITICAL] Automatic reconnection failed: {recon_err}")
    return None

# Debug filter to prevent terminal flooding during high-frequency execution
last_action_logged = ""
def send_command_logged(cmd_str):
    """Sends command to ESP32 only if it differs from the last sent command"""
    global last_action_logged
    try:
        esp.send_command(cmd_str)
        if cmd_str != last_action_logged:
            print(f"[ESP_OUT] Sent Command: {cmd_str}")
            last_action_logged = cmd_str
    except (OSError, Exception) as e:
        print(f"[SERIAL WARNING] Failed to transmit command '{cmd_str}': {e}")

state = States.INIT
last_state = None
first_sector_start_time = None  # Tracks 1s ignore window for launch

try:
    print("IDLE")
    while True:
        if reset_OC:
            num_of_turn = 0
            is_clockwise = True
            turning_point = right_turning_point
            track_left = [40, 280]
            track_right = [600, 280]
            last_track_error = 0.0
            if run_OC1:
                state = States.FIRST_SECTOR
            reset_OC = False
            last_state = None
            first_sector_start_time = None
            print("Resetted")
            continue

        elif run_OC1:
            print("[FSM] Starting OC1 Challenge Operations!")
            send_command_logged("OC1")
            start_time = time.perf_counter()

            while run_OC1:
                end_time = time.perf_counter()
                reply = esp_replyNprint()

                if state != last_state:
                    print(f"\n[FSM] STATE CHANGE: {last_state} ===> {state}")
                    last_state = state

                if reply == "EOC1":
                    print("EOC1")
                    run_OC1 = False
                    break

                _, _, track = get_latest_data()

                # ======================================================================
                # 1. FIRST SECTOR (INWARD EDGE-DISTANCE CALCULATION)
                # ======================================================================
                if state == States.FIRST_SECTOR:
                    temp_speed = int(speed * 0.75)
                    send_command_logged(f"{temp_speed}, 0, 1, go forward")

                    left_wall_dist = get_left_wall_distance()
                    right_wall_dist = get_right_wall_distance()
                    time.sleep(0.03)

                    # A larger distance means it took longer to hit a wall -> that side is open space!
                    if abs(left_wall_dist - right_wall_dist) > 20: # 20px threshold to filter noise
                        if left_wall_dist > right_wall_dist:
                            print(f"[STARTUP LOCK] Left side open space deeper ({left_wall_dist}px vs {right_wall_dist}px). Corridor is LEFT: Counter-Clockwise.")
                            is_clockwise = False
                            turning_point = left_turning_point
                            state = States.WAIT_TURN_STATE
                            continue
                        else:
                            print(f"[STARTUP LOCK] Right side open space deeper ({right_wall_dist}px vs {left_wall_dist}px). Corridor is RIGHT: Clockwise.")
                            is_clockwise = True
                            turning_point = right_turning_point
                            state = States.WAIT_TURN_STATE
                            continue

                # ======================================================================
                # 2. WAIT TURN STATE (ORIGINAL SPATIAL COORDINATE CHECK)
                # ======================================================================
                elif state == States.WAIT_TURN_STATE:
                    temp_speed = int(speed * 0.6)
                    send_command_logged(f"{temp_speed}, 0, 1, encoder approach")

                    see_target_side = get_track_distance(turning_point[0], turning_point[1])[0]

                    if reply == "DEGREE_DONE" or see_target_side:
                        state = States.TURNING_STATE
                        num_of_turn += 1
                        print(f"[FSM] Turning condition reached. Moving to turn {num_of_turn}")
                        time.sleep(0.05)
                        continue

                # ======================================================================
                # 3. TURNING STATE (FIXED MATH BIAS + LOOP CATCH MECHANISM)
                # ======================================================================
                elif state == States.TURNING_STATE:
                    angle = (track["center_x"] - 320) / 0.4

                    if angle > 100:
                        angle = 100
                    elif angle < -100:
                        angle = -100

                    # Fixed Sign Inversions: Force assertive turning values if tracking slips
                    if is_clockwise == True and angle < 15:
                        angle = 60  # Keep turning right confidently
                    if is_clockwise == False and angle > -15:
                        angle = -60 # Keep turning left confidently

                    turn_speed = int(speed * 0.75)
                    send_command_logged(f"{turn_speed}, {int(angle)}, -1, turn")

                    is_sector_detected = get_track_distance(sector_indi[0], sector_indi[1])[0]

                    if is_sector_detected:
                        if turn_time <= 0.3:
                            num_of_turn -= 1
                            state = States.RUN_SECTOR_STATE
                            turn_time = 0
                            angle = 0
                            last_track_error = 0.0
                            continue

                        # Clean deceleration block
                        buffer = angle / 5
                        for i in range(4):
                            angle -= buffer
                            send_command_logged(f"12, {int(angle)}, -1, turn")
                            time.sleep(0.02)

                        # Explicit state jump definitions avoiding drop-through glitching
                        if num_of_turn == 12:
                            state = States.LAST_RUN
                        else:
                            state = States.DASH_AFTER_TURNING_STATE
                        turn_time = 0
                        continue # Escape frame gracefully
                    else:
                        time.sleep(0.025)
                        turn_time += 0.025
                        continue

                # ======================================================================
                # 4. DASH AFTER TURNING STATE
                # ======================================================================
                elif state == States.DASH_AFTER_TURNING_STATE:
                    if is_clockwise:
                        send_command_logged(f"{speed}, 35, 150, violent dive right")
                    else:
                        send_command_logged(f"{speed}, -35, 150, violent dive left")

                    time.sleep(0.01)

                    if num_of_turn == 12:
                        state = States.LAST_RUN
                    else:
                        state = States.RUN_SECTOR_STATE
                        last_track_error = 0.0

                # ======================================================================
                # 5. RUN SECTOR STATE (DIRECTION-BASED BOUNDARY GUARD LOGIC)
                # ======================================================================
                elif state == States.RUN_SECTOR_STATE:
                    see_target_side = get_track_distance(turning_point[0], turning_point[1])[0]
                    see_front_wall = check_front_wall_range(FIXED_FRONT_COLUMNS)
                    
                    too_close_left = get_track_distance(track_left[0], track_left[1])[0]
                    too_close_right = get_track_distance(track_right[0], track_right[1])[0]

                    # Change state logic guarded by specific tracking boundaries
                    should_transition = False
                    if is_clockwise and (see_target_side or see_front_wall) and not too_close_right:
                        should_transition = True
                    elif not is_clockwise and (see_target_side or see_front_wall) and not too_close_left:
                        should_transition = True

                    if should_transition:
                        print(f"[VISION LOCK-ON] Target detected while clear of outer bounds. Transitioning to corner approach.")
                        state = States.WAIT_TURN_STATE
                        continue

                    Kp = 1.4
                    Kd = 0.8
                    reduce_factor = 4

                    if is_clockwise:
                        if too_close_right:
                            base_angle = -50
                            current_error = 0.0
                        else:
                            current_error = (595 - track["center_x"]) / reduce_factor
                            derivative = current_error - last_track_error
                            base_angle = (current_error * Kp) + (derivative * Kd)
                    else:
                        if too_close_left:
                            base_angle = 50
                            current_error = 0.0
                        else:
                            current_error = (45 - track["center_x"]) / reduce_factor
                            derivative = current_error - last_track_error
                            base_angle = (current_error * Kp) + (derivative * Kd)

                    last_track_error = current_error

                    base_angle = max(-85, min(85, base_angle))
                    send_command_logged(f"{speed}, {int(base_angle)}, -1, magnet runner")

                # ======================================================================
                # 6. LAST RUN
                # ======================================================================
                elif state == States.LAST_RUN:
                    send_command_logged("50, 0, 200, search final line")
                    send_command_logged("0, 0, 100, motor stop")

                    if get_track_distance(ending_point[0], ending_point[1])[0]:
                        print("[VISION SUCCESS] Final wall spotted early! Initiating braking sequence.")

                        send_command_logged("0, 0, 0, move forward")
                        time.sleep(0.45)

                        send_command_logged("0, 0, 0, motor stop")
                        end_time = time.perf_counter()
                        run_OC1 = False
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
                print("[EVENT] Received start instruction for OC1 Challenge.")
                reset_OC = True
                run_OC1 = True
                continue
            elif reply == "ROC2":
                print("[EVENT] Received start instruction for OC2 Challenge.")
                reset_OC = True
                run_OC2 = True
                continue
        time.sleep(0.05)

except KeyboardInterrupt:
    print("\nShutting down...")
    try:
        esp.send_command("0, 0, 0, emergency_stop")
    except Exception:
        pass
    esp.disconnect()
    stop_vision_system()
    time.sleep(0.5)
