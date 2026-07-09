# testing
import time
import datetime
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

live_streaming = False
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

# PD Tracking Memory for stabilizing the ultra-tight inner wall path
last_track_error = 0.0

left_turning_point = [5, 100]
right_turning_point = [635, 100]
turning_point = right_turning_point

# Central vertical curtain parameters
FRONT_SCAN_Y_START = 0
FRONT_SCAN_Y_END = 140
FRONT_SCAN_STEP = 5

# Radar scanning bands matching baseline setup
RADAR_COUNTER_CW = [320, 340, 360]
RADAR_CLOCKWISE = [280, 300, 320]

# FIRST SECTOR HORIZONTAL RANGES
# these values are highly related to the sensitivity of the robot turning around the corners
INIT_SCAN_Y = 140
LEFT_SCAN_X_START = 20
LEFT_SCAN_X_END = 40
RIGHT_SCAN_X_START = 600
RIGHT_SCAN_X_END = 620
SCAN_STEP_X = 8

# track_left // track_right
TRACK_INIT_SCAN_Y = 100
TRACK_LEFT_SCAN_X_START = 0
TRACK_LEFT_SCAN_X_END = 350
TRACK_RIGHT_SCAN_X_START = 290
TRACK_RIGHT_SCAN_X_END = 640
TRACK_SCAN_STEP_X = 8

# ======================================================================
# TUNED: SAFETY ZONES (15 Pixels from Screen Edges)
# ======================================================================
track_left = [140, 200]
track_right = [500, 200]

# Retained early-reaction finish line configuration
ending_point = [320, 110]

#Universal speed parameter
speed = 12

# States
class States(Enum):
    INIT = 0
    FIRST_SECTOR = 1
    WAIT_TURN_STATE = 2
    TURNING_STATE = 3
    DASH_AFTER_TURNING_STATE = 4
    RUN_SECTOR_STATE = 5
    LAST_RUN = 6

# Dynamic multi-line vertical range check
def check_front_wall_range(columns_to_scan):
    for x_pos in columns_to_scan:
        for y in range(FRONT_SCAN_Y_START, FRONT_SCAN_Y_END + 1, FRONT_SCAN_STEP):
            if get_track_distance(x_pos, y)[0]:
                return True
    return False

# Horizontal side sector range scanners for startup accuracy
def check_left_sector_range():
    for x in range(LEFT_SCAN_X_START, LEFT_SCAN_X_END + 1, SCAN_STEP_X):
        if get_track_distance(x, INIT_SCAN_Y)[0]:
            return True
    return False

def check_right_sector_range():
    for x in range(RIGHT_SCAN_X_START, RIGHT_SCAN_X_END + 1, SCAN_STEP_X):
        if get_track_distance(x, INIT_SCAN_Y)[0]:
            return True
    return False

#track_left // track_right
def check_left_track_range():
    for x in range(TRACK_LEFT_SCAN_X_START, TRACK_LEFT_SCAN_X_END + 1, TRACK_SCAN_STEP_X):
        if get_track_distance(x, TRACK_INIT_SCAN_Y)[0]:
            return True
    return False

def check_right_track_range():
    for x in range(TRACK_RIGHT_SCAN_X_START, TRACK_RIGHT_SCAN_X_END + 1, TRACK_SCAN_STEP_X):
        if get_track_distance(x, TRACK_INIT_SCAN_Y)[0]:
            return True
    return False
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
        #if cmd_str != last_action_logged:
        if True:
            print(f"[ESP_OUT] Sent Command: {cmd_str}")
            last_action_logged = cmd_str
    except (OSError, Exception) as e:
        print(f"[SERIAL WARNING] Failed to transmit command '{cmd_str}': {e}")

state = States.INIT
last_state = None

try:
    print("IDLE")
    while True:
        if reset_OC:
            num_of_turn = 0
            is_clockwise = True
            turning_point = right_turning_point
            track_left = [15, 280]
            track_right = [625, 280]
            last_track_error = 0.0
            if run_OC1:
                state = States.FIRST_SECTOR
            reset_OC = False
            last_state = None
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

                # Determine active radar setup dynamically based on current tracking state
                active_radar = RADAR_CLOCKWISE if is_clockwise else RADAR_COUNTER_CW

                # ======================================================================
                # 1. FIRST SECTOR
                # ======================================================================
                if state == States.FIRST_SECTOR:
                    send_command_logged("10, 0, -1, go forward")

                    see_left_zone = check_left_sector_range()
                    see_right_zone = check_right_sector_range()
                    see_front_wall_range = check_front_wall_range(active_radar)

                    if see_left_zone:
                        print(f"[STARTUP LOCK] Left corridor array triggered! Orientation: Counter-Clockwise.")
                        is_clockwise = False
                        turning_point = left_turning_point
                        state = States.WAIT_TURN_STATE
                        continue

                    elif see_right_zone:
                        print(f"[STARTUP LOCK] Right corridor array triggered! Orientation: Clockwise.")
                        is_clockwise = True
                        turning_point = right_turning_point
                        state = States.WAIT_TURN_STATE
                        continue

                    elif see_front_wall_range:
                        if track["center_x"] < 320:
                            print(f"[STARTUP FAILSAFE] Front curtain triggered! Bias Left -> Counter-Clockwise.")
                            is_clockwise = False
                            turning_point = left_turning_point
                        else:
                            print(f"[STARTUP FAILSAFE] Front curtain triggered! Bias Right -> Clockwise.")
                            is_clockwise = True
                            turning_point = right_turning_point
                        state = States.WAIT_TURN_STATE
                        continue

                # ======================================================================
                # 2. WAIT TURN STATE
                # ======================================================================
                elif state == States.WAIT_TURN_STATE:
                    send_command_logged("{speed}, 0, 2500, encoder approach")

                    see_target_side = get_track_distance(turning_point[0], turning_point[1])[0]
                    see_front_wall_range = check_front_wall_range(active_radar)

                    if reply == "DEGREE_DONE" or see_target_side or see_front_wall_range:
                        state = States.TURNING_STATE
                        num_of_turn += 1
                        print(f"[FSM] Dynamic radar array triggered! Moving to turn {num_of_turn}")
                        time.sleep(0.05)
                        continue

                # ======================================================================
                # 3. TURNING STATE
                # ======================================================================
                elif state == States.TURNING_STATE:
                    
                    if is_clockwise == True:
                        angle = abs((330 - track["center_x"]) / 0.4)
                    elif is_clockwise == False:
                        angle = abs((track["center_x"] - 320) / 0.4)
                        
                    if is_clockwise == True: angle = angle
                    elif is_clockwise == False: angle = -angle
                    if angle > 100: angle = 100
                    elif angle < -100: angle = -100

                    send_command_logged(f"{int(speed*0.75)}, {int(angle)}, -1, turn")

                    is_sector_detected = get_track_distance(sector_indi[0], sector_indi[1])[0]

                    if (is_sector_detected or turn_time >= 1.5) and (turn_time >= 2.5 or track["center_x"] > 320):
                        if turn_time <= 0.3:
                            num_of_turn -= 1
                            state = States.RUN_SECTOR_STATE
                            turn_time = 0
                            angle = 0
                            last_track_error = 0.0
                            continue

                        #buffer = angle / 5
                        #for i in range(4):
                            #angle -= buffer
                            #send_command_logged(f"12, {angle}, -1, turn")
                            #time.sleep(0.02)

                        if num_of_turn == 12:
                            state = States.LAST_RUN
                        else:
                            print(f"\n is_sector_detected={is_sector_detected}||turn_time={turn_time}")
                            state = States.DASH_AFTER_TURNING_STATE
                        turn_time = 0
                    else:
                        time.sleep(0.025)
                        turn_time += 0.025
                        continue

                # ======================================================================
                # 4. DASH AFTER TURNING STATE
                # ======================================================================
                elif state == States.DASH_AFTER_TURNING_STATE:
                    if is_clockwise:
                        send_command_logged("{speed*1.2}, 35, 300, violent dive right")
                    else:
                        send_command_logged("{speed*1.2}, -35, 300, violent dive left")

                    time.sleep(0.14)

                    if num_of_turn == 12:
                        state = States.LAST_RUN
                    else:
                        state = States.RUN_SECTOR_STATE
                        last_track_error = 0.0

                # ======================================================================
                # 5. RUN SECTOR STATE
                # ======================================================================
                elif state == States.RUN_SECTOR_STATE:
                    if is_clockwise == False:
                        see_target_zone = check_left_sector_range()
                        see_track_zone = check_left_sector_range()
                    else:
                        see_target_zone = check_right_sector_range()
                        see_track_zone = check_right_sector_range()
                    see_target_side = get_track_distance(turning_point[0], turning_point[1])[0]
                    see_front_wall_range = check_front_wall_range(active_radar)

                    if (see_target_zone or see_target_side) and see_front_wall_range:
                        print(f"[VISION LOCK-ON] Imminent wall registered by dynamic radar. Moving to corner approach. see_target_zone={see_target_zone} || see_target_side={see_target_side} || see_front_wall_range={see_front_wall_range}")
                        state = States.WAIT_TURN_STATE
                        continue

                    # Evaluates proximity based on new [15] and [625] coordinates
                    too_close_left = see_track_zone or get_track_distance(track_left[0], track_left[1])[0]
                    too_close_right = see_track_zone or get_track_distance(track_right[0], track_right[1])[0]

                    Kp = 1.4
                    Kd = 0.8
                    print(f"see_target_zone={see_track_zone}||too_close_left={too_close_left}||too_close_right={too_close_right}")

                    if is_clockwise:
                        if too_close_right:
                            base_angle = -5
                            current_error = 0.0
                        else:
                            current_error = (track["center_x"] - 595) / 40
                            derivative = current_error - last_track_error
                            base_angle = (current_error * Kp) + (derivative * Kd)
                            if base_angle < 0:
                                base_angle = -base_angle
                    else:
                        if too_close_left:
                            base_angle = 5
                            current_error = 0.0
                        else:
                            current_error = (track["center_x"] - 45) / 40
                            derivative = current_error - last_track_error
                            base_angle = (current_error * Kp) + (derivative * Kd)
                            if base_angle > 0:
                                base_angle = -base_angle

                    last_track_error = current_error

                    #base_angle = max(-20, min(20, base_angle))
                    if base_angle != 5 and base_angle != -5:
                        send_command_logged(f"{int(speed)}, {int(base_angle)}, -1, magnet runner")
                    else:
                        send_command_logged(f"{int(speed)}, {int(base_angle)}, -1, re-turn")
                        
                    #time.sleep(0.025)

                # ======================================================================
                # 6. LAST RUN
                # ======================================================================
                elif state == States.LAST_RUN:
                    send_command_logged("{speed}, 0, 100, search final line")

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
