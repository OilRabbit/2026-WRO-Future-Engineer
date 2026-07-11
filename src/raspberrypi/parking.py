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
    get_track_distance  # Returns (is_inside, exact_distance)
)
from picamera2 import Picamera2 as picam2
from enum import Enum

# ESP communication and Picam init
print("======= Init =======")

esp = ESP32Communicator()
esp.connect()
camera = picam2()
full_sensor_res = camera.sensor_resolution

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
run_OC1 = True
run_OC2 = False
reset_OC = True
is_clockwise = False
target_done = False
run_target_sent = False

# ==================================================================
# CONFIGURATION PARAMETERS
# ==================================================================
PURPLE_TARGET_X = 470
PURPLE_ALIGN_SPEED = 7.5
PURPLE_STOP_THRESHOLD = 130

PROBE_Y = 129.5                  # Fixed vertical look-ahead line
PROBE_LEFT_X = 260             # Inward adjusted left column
PROBE_RIGHT_X = 300            # Inward adjusted right column

# STOPPING CONDITION
STOP_DISTANCE_THRESHOLD = 1.0  # Stop walking forward when distance to edge < 1

# PD Controller gains for steering alignment
KP = 20
KD = 1.2

# Track derivative terms
last_alignment_error = 0.0

class ParkingStates(Enum):
    ALIGNING = 0            # Track Wall 1 (Purple Wall)
    WALL_1_PAUSE = 1        # Settle robot momentum
    BLACK_WALL_PD_APPROACH = 2 # Continuous PD straight approach (White Edge)
    BACKWARD = 3        # Step 1: Encoder micro-positioning
    BACKWARD_TURN_1 = 4     # Step 2: Reverse entry swing
    BACKWARD_STRAIGHT = 5   # Step 2b: Straight depth segment
    BACKWARD_TURN_2 = 6     # Step 3: Counter-steer alignment
    COMPLETED = 7           # Complete

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))

def send_command_logged(cmd):
    try:
        esp.send_command(cmd)
    except Exception as e:
        print(f"[SERIAL WRITE FAULT] Failed to transmit packet: {e}")

def esp_replyNprint():
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    reply = esp.read_message()
    if reply is not None:
        print(f"{timestamp} ESP: {reply}")
    return reply

oc1_parking_state = ParkingStates.ALIGNING
maneuver_start_time = 0

try:
    print("IDLE")
    while True:
        if reset_OC:
            if run_OC1:
                print("[INIT] Starting Objective Challenge 1...")
                oc1_parking_state = ParkingStates.ALIGNING
                target_done = False
                run_target_sent = False
                last_alignment_error = 0.0
            reset_OC = False
            print("Resetted")
            continue

        elif run_OC1:
            while run_OC1:
                reply = esp_replyNprint()
                if reply == "EOC1":
                    run_OC1 = False
                    break
                if reply == "Done Target":
                    target_done = True

                # ==================================================================
                # SUB-STATES: WALL 1 DETECTION (PURPLE WALL ALIGNMENT)
                # ==================================================================
                if oc1_parking_state == ParkingStates.ALIGNING:
                    _, parking, _ = get_latest_data()
                    purple_x = parking.get("center_x", 0)
                    purple_width = parking.get("width", 0)
                    purple_height = parking.get("height", 0)
                    purple_area = (purple_width * purple_height) // 100

                    print(f"[TRACKING WALL 1] X: {purple_x} | Area: {purple_area}")

                    if purple_x != 0 and purple_area > 10:
                        if purple_area >= PURPLE_STOP_THRESHOLD:
                            print(f"\n[⚠️ WALL 1 DETECTED] Halting for temporary pause...")
                            send_command_logged("0, 0, 0, wall 1 stop")
                            oc1_parking_state = ParkingStates.WALL_1_PAUSE
                            maneuver_start_time = time.time()
                        else:
                            error = purple_x - PURPLE_TARGET_X
                            kp_purple = 1.9
                            align_steering = int(clamp(error * kp_purple, -65, 65))
                            send_command_logged(f"{PURPLE_ALIGN_SPEED}, {align_steering}, 0, purple align")
                    else:
                        send_command_logged("0, 0, 0, tracking lost holding")

                elif oc1_parking_state == ParkingStates.WALL_1_PAUSE:
                    if time.time() - maneuver_start_time < 1.5:
                        send_command_logged("0, 0, 0, holding pause")
                    else:
                        print("[SYSTEM] Pause complete. Transitioning to Continuous PD Wall Approach...")
                        oc1_parking_state = ParkingStates.BLACK_WALL_PD_APPROACH
                        last_alignment_error = 0.0

                # ==================================================================
                # CONTINUOUS PD APPROACH (WHITE EDGE ALIGNMENT FROM Y=80)
                # ==================================================================
                elif oc1_parking_state == ParkingStates.BLACK_WALL_PD_APPROACH:
                    # Fetching probe metrics
                    has_poly_l, dist_left = get_track_distance(PROBE_LEFT_X, PROBE_Y)
                    has_poly_r, dist_right = get_track_distance(PROBE_RIGHT_X, PROBE_Y)
                    dist_left = dist_left + 1.7

                    # Print out precise diagnostics on each iteration loop
                    print(f"[PROBE LEFT  ({PROBE_LEFT_X}, {PROBE_Y})] Inside Poly: {int(has_poly_l)} | Distance to Edge: {dist_left:.2f}")
                    print(f"[PROBE RIGHT ({PROBE_RIGHT_X}, {PROBE_Y})] Inside Poly: {int(has_poly_r)} | Distance to Edge: {dist_right:.2f}")

                    # Evaluate arrival stopping trigger (When distance to edge < 1)
                    if dist_left < STOP_DISTANCE_THRESHOLD or dist_right < STOP_DISTANCE_THRESHOLD:
                        send_command_logged("0, 0, 0, R")
                        print(f"\n[⚓ WALL ARRIVAL MET] Edge distance dropped below {STOP_DISTANCE_THRESHOLD}! Proceeding to backward motions.")
                        time.sleep(1.0) # Settle vehicle momentum before parking actions
                        oc1_parking_state = ParkingStates.BACKWARD
                        run_target_sent = False
                        target_done = False
                        continue

                    # --- ACTIVE PD STEERING LOOP ---
                    # Right - Left subtraction fixes the sign inversion to match physical steering
                    alignment_error = dist_left - dist_right
                    derivative = alignment_error - last_alignment_error

                    # Proportional-Derivative steering response output calculation
                    pd_steering = int(clamp((KP * alignment_error) + (KD * derivative), -55, 55))
                    last_alignment_error = alignment_error

                    # Move forward while adjusting wheels via PD controller
                    send_command_logged(f"6, {pd_steering}, 0, pd front wall alignment")
                    print(f"[PD CONTROL] Alignment Error: {alignment_error:+.2f} | Transmitted Steer: {pd_steering}\n")

                # ==================================================================
                # PARKING DISPLACEMENT EXECUTION STATES
                # ==================================================================
                elif oc1_parking_state == ParkingStates.BACKWARD:
                    if run_target_sent == False:
                        send_command_logged(f"-9, 0, 1, forward target")
                        run_target_sent = True
                        target_done = False

                    if not target_done:
                        time.sleep(0.001)
                        continue
                    else:
                        send_command_logged("-1, 0, 0, R")
                        oc1_parking_state = ParkingStates.BACKWARD_TURN_1
                        run_target_sent = False
                        target_done = False
                        continue

                elif oc1_parking_state == ParkingStates.BACKWARD_TURN_1:
                    if run_target_sent == False:
                        steer = -100 if is_clockwise else 100
                        send_command_logged(f"-9, {steer}, 45, forward target")
                        run_target_sent = True
                        target_done = False

                    if not target_done:
                        time.sleep(0.001)
                        continue
                    else:
                        send_command_logged("-1, 0, 0, R")
                        oc1_parking_state = ParkingStates.BACKWARD_STRAIGHT
                        run_target_sent = False
                        target_done = False
                        continue

                elif oc1_parking_state == ParkingStates.BACKWARD_STRAIGHT:
                    if run_target_sent == False:
                        send_command_logged("-9, 0, 3, forward target")
                        run_target_sent = True
                        target_done = False

                    if not target_done:
                        time.sleep(0.001)
                        continue
                    else:
                        send_command_logged("-1, 0, 0, R")
                        oc1_parking_state = ParkingStates.BACKWARD_TURN_2
                        run_target_sent = False
                        target_done = False
                        continue

                elif oc1_parking_state == ParkingStates.BACKWARD_TURN_2:
                    if run_target_sent == False:
                        steer = 100 if is_clockwise else -100
                        send_command_logged(f"-9, {steer}, 55, forward target")
                        run_target_sent = True
                        target_done = False

                    if not target_done:
                        time.sleep(0.001)
                        continue
                    else:
                        send_command_logged("-1, 0, 0, R")
                        oc1_parking_state = ParkingStates.COMPLETED
                        run_target_sent = False
                        target_done = False
                        continue

                elif oc1_parking_state == ParkingStates.COMPLETED:
                    send_command_logged("0, 0, 0, parking complete")
                    run_OC1 = False
                    reset_OC = True

                time.sleep(0.02)

        elif run_OC2:
            time.sleep(0.02)

except KeyboardInterrupt:
    print("\nShutting down software control loop...")
    esp.disconnect()
    stop_vision_system()
    time.sleep(0.5)