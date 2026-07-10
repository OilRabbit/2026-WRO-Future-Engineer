import cv2
import time
import datetime
import math
from esp_com.communication import ESP32Communicator
from camera.camera_utils import (
    start_vision_system,
    start_web_server,
    get_latest_data,
    stop_vision_system,
    get_track_distance
)
from picamera2 import Picamera2 as picam2
from enum import Enum

# ESP communication and Picam init
print("======= Init =======")

esp = ESP32Communicator()
esp.connect()
camera = picam2()
full_sensor_res = camera.sensor_resolution

# Resolution set to 640x360
config = camera.create_video_configuration(main={"size": (640, 360), "format": "BGR888"}, sensor={"output_size": full_sensor_res})
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
run_OC1 = True  # Set to True so that it executes immediately on start
run_OC2 = False
reset_OC = True
num_of_turn = 0
is_clockwise = True  # Set to True for clockwise track, False for anticlockwise
start_time = 0
end_time = 0
recorded_time = 0

run_target_sent = False
target_done = False

PURPLE_TARGET_X = 490
PURPLE_ALIGN_SPEED = 7.5
PURPLE_STOP_THRESHOLD = 150

# Checkpoints (default as clockwise case)
angle = 0
speed = 0
turn_flag = 0
turn_indi_1 = [320, 140]
turn_indi_2 = [320, 170]
sector_indi = [[240, 140], [400, 140]]
turn_time = 0
run_time = 0
lot_count = 0
lot_count_flag = 1
pillar_count = 0
pillar_count_flag = 1
pillar_count_temp = 0

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

class OC2_States(Enum):
    INIT = 0
    LEAVE = 1
    WHITE = 2
    PILLAR = 3
    ENTER = 4

# SUB-STATES FOR THE TWO-WALL PARKING SEQUENCE
class ParkingStates(Enum):
    ALIGNING = 0          # Visually tracking the current purple wall
    FORWARD_OVERRIDE = 1  # Blindly driving straight for 0.5s after Wall 1
    MOVE_FORWARD = 2      # Step 1: Final adjustments after Wall 2 / Timeout
    BACKWARD_TURN_1 = 3   # Step 2: Reverse corner-turn
    BACKWARD_STRAIGHT = 4 # NEW STEP: Straight reverse segment to get deeper into the space
    BACKWARD_TURN_2 = 5   # Step 3: Reverse counter-steer parallel alignment
    COMPLETED = 6         # Step 4: Finished

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))

def send_command_logged(cmd):
    """Centralized logging connection method for handling transmission packets."""
    try:
        esp.send_command(cmd)
    except Exception as e:
        print(f"[SERIAL WRITE FAULT] Failed to transmit packet: {e}")

# Function for receiving msg from ESP and print the message with timestamp
def esp_replyNprint():
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    reply = esp.read_message()
    if reply is not None:
        print(f"{timestamp} ESP: {reply}")
    return reply

state = States.INIT
OC2_state = OC2_States.INIT
oc1_parking_state = ParkingStates.ALIGNING
maneuver_start_time = 0
wall_count = 0  # 0 = looking for Wall 1, 1 = looking for Wall 2

# TIMEOUT MONITORING FOR WALL 2
lost_tracking_start_time = None  # Stores the timestamp when Wall 2 is lost

try:
    print("IDLE")
    while True:
        if reset_OC:
            if run_OC1:
                print("[INIT] Starting Objective Challenge 1 Stage...")
                oc1_parking_state = ParkingStates.ALIGNING
                wall_count = 0
                lost_tracking_start_time = None
                target_done = False

            if run_OC2:
                angle = 0
                speed = 0
                is_clockwise = True
                lot_count = 0
                lot_count_flag = 1
                pillar_count = 0
                pillar_count_flag = 1
                OC2_state = OC2_States.LEAVE
                print("[INIT] Starting Objective Challenge 2 Stage...")

            reset_OC = False
            print("Resetted")
            continue

        elif run_OC1:
            # ==================================================================
            # SUB-STATE: VISUAL ALIGNMENT & APPROACH (WALL 1 & WALL 2)
            # ==================================================================while run_OC1:
            end_time = time.perf_counter()
            reply = esp_replyNprint()
            if reply == "EOC1":
                print("EOC1")
                run_OC1 = False
                break
            if reply == "Done Target":
                target_done = True
            _, _, track = get_latest_data()
            print("States = {}".format(oc1_parking_state))
            if oc1_parking_state == ParkingStates.ALIGNING:
                obstacle, parking, track = get_latest_data()

                purple_x = parking.get("center_x", 0)
                purple_width = parking.get("width", 0)
                purple_height = parking.get("height", 0)
                purple_area = (purple_width * purple_height) // 100

                print(f"[TRACKING WALL {wall_count + 1}] X: {purple_x} | Area: {purple_area} / Target Threshold: {PURPLE_STOP_THRESHOLD}")

                if purple_x != 0 and purple_area > 10:
                    # Reset the timeout counter whenever tracking data is valid
                    lost_tracking_start_time = None

                    if purple_area >= PURPLE_STOP_THRESHOLD:
                        wall_count += 1

                        if wall_count == 1:
                            print(f"\n[⚠️ WALL 1 DETECTED] Area {purple_area} >= {PURPLE_STOP_THRESHOLD}. Overriding vision to push forward straight...")
                            oc1_parking_state = ParkingStates.FORWARD_OVERRIDE
                            maneuver_start_time = time.time()
                        else:
                            send_command_logged("0, 0, 0, R")
                            print(f"\n[🛑 WALL 2 DETECTED] Area {purple_area} >= {PURPLE_STOP_THRESHOLD}. Initiating parallel parking adjustment...")
                            oc1_parking_state = ParkingStates.MOVE_FORWARD
                            maneuver_start_time = time.time()
                            target_done = False
                            continue
                    else:
                        error = purple_x - PURPLE_TARGET_X
                        kp_purple = 0.55
                        align_steering = int(clamp(error * kp_purple, -65, 65))

                        send_command_logged(f"{PURPLE_ALIGN_SPEED}, {align_steering}, 0, purple align")

                else:
                    # Handle Frame Drops / Area = 0 Cases
                    send_command_logged("0, 0, 0, tracking lost holding")

                    # Target loss logic applies exclusively when tracking the second wall
                    if wall_count == 1:
                        if lost_tracking_start_time is None:
                            # Start the dropout clock timer
                            lost_tracking_start_time = time.time()
                            print(" -> [WARN] Lost Wall 2 visibility anchor! Commencing 1-second safety timer...")
                        else:
                            elapsed_lost_time = time.time() - lost_tracking_start_time
                            print(f" -> [SEARCHING] Wall 2 absent for {elapsed_lost_time:.2f}s / 1.00s max.")

                            # Timeout limit check (1.0 second rule trigger)
                            if elapsed_lost_time >= 1.0:
                                send_command_logged("0, 0, 0, motor stop")
                                print("\n[🚨 TIMEOUT OVERRIDE] Wall 2 missing for over 1.0s. Skipping directly to parallel parking sequence!")

                                # Elevate state immediately to step 1 of parking maneuvers
                                oc1_parking_state = ParkingStates.MOVE_FORWARD
                                maneuver_start_time = time.time()
                                target_done = False
                    else:
                        print(" -> [SEARCHING] Wall 1 dropped out of tracking frame.")

            # ==================================================================
            # TRANSITION OVERRIDE: MOVE STRAIGHT FOR 0.5 SECONDS PAST WALL 1
            # ==================================================================
            elif oc1_parking_state == ParkingStates.FORWARD_OVERRIDE:
                run_target_sent = True
                if time.time() - maneuver_start_time < 0.1:
                    send_command_logged(f"{PURPLE_ALIGN_SPEED}, 0, 0, wall 1 bypass straight")
                else:
                    print("[SYSTEM] 0.5s override finished. Resuming visual tracking to find Wall 2...")
                    oc1_parking_state = ParkingStates.ALIGNING
                    run_target_sent = False
                    continue

            # ==================================================================
            # STEP 1: MOVE FORWARD A BIT (FINAL PARKING INITIATION)
            # ==================================================================
            elif oc1_parking_state == ParkingStates.MOVE_FORWARD:
                print("hiiiii")
                if run_target_sent == False:
                    send_command_logged("8, 0, 10, forward target")
                    run_target_sent = True
                    target_done = False

                if not target_done:
                    time.sleep(0.001)
                    continue

                else:
                    send_command_logged("0, 0, 0, R")
                    print("[PARKING] Step 1 finished. Transitioning to Step 2...")
                    oc1_parking_state = ParkingStates.BACKWARD_TURN_1
                    maneuver_start_time = time.time()
                    run_target_sent = False
                    target_done = False
                    send_command_logged("0, 0, 0, R")
                    continue

            # ==================================================================
            # STEP 2: BACKWARD TURN 1
            # ==================================================================
            elif oc1_parking_state == ParkingStates.BACKWARD_TURN_1:
                if run_target_sent == False:
                    steer = -95 if is_clockwise else 95
                    send_command_logged(f"-8, {steer}, 10, forward target")
                    run_target_sent = True
                    target_done = False

                if not target_done:
                    time.sleep(0.001)
                    continue

                else:
                    send_command_logged("0, 0, 0, motor stop")
                    print("[PARKING] Step 2 finished. Transitioning to Step 2b (Straight Backward)...")
                    oc1_parking_state = ParkingStates.BACKWARD_STRAIGHT
                    maneuver_start_time = time.time()
                    run_target_sent = False
                    target_done = False
                    send_command_logged("0, 0, 0, R")
                    continue

            # ==================================================================
            # STEP 2b: NEW - STRAIGHT BACKWARD OVERRIDE (DEEPING INTO POSITION)
            # ==================================================================
            elif oc1_parking_state == ParkingStates.BACKWARD_STRAIGHT:
                if run_target_sent == False:
                    send_command_logged("-8, 0, 10, forward target")
                    run_target_sent = True
                    target_done = False

                if not target_done:
                    time.sleep(0.001)
                    continue

                else:
                    send_command_logged("0, 0, 0, motor stop")
                    print("[PARKING] Step 2b finished. Transitioning to Step 3 (Counter-Steer)...")
                    oc1_parking_state = ParkingStates.BACKWARD_TURN_2
                    maneuver_start_time = time.time()
                    run_target_sent = False
                    target_done = False
                    send_command_logged("0, 0, 0, R")
                    continue

            # ==================================================================
            # STEP 3: BACKWARD TURN 2 (COUNTER-STEER TO PARALLEL)
            # ==================================================================
            elif oc1_parking_state == ParkingStates.BACKWARD_TURN_2:
                if run_target_sent == False:
                    steer = -95 if is_clockwise else 95
                    send_command_logged(f"-8, {steer}, 10, forward target")
                    run_target_sent = True
                    target_done = False

                if not target_done:
                    time.sleep(0.001)
                    continue

                else:
                    send_command_logged("0, 0, 0, motor stop")
                    print("[PARKING] Step 3 finished. Parallel alignment complete.")
                    oc1_parking_state = ParkingStates.COMPLETED
                    run_target_sent = False
                    target_done = False
                    continue

            # ==================================================================
            # STEP 4: COMPLETED PARKING MANEUVER
            # ==================================================================
            elif oc1_parking_state == ParkingStates.COMPLETED:
                send_command_logged("0, 0, 0, parking complete")
                print("\n[🏁 PARKING TASK COMPLETE] Robot is successfully aligned parallel to the second wall.")
                run_OC1 = False
                reset_OC = True

            time.sleep(0.02)

        elif run_OC2:
            time.sleep(0.02)

except KeyboardInterrupt:
    print("\nShutting down...")
    esp.disconnect()
    stop_vision_system()
    time.sleep(0.5)
