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
    get_track_distance,  # Returns (is_inside, exact_distance)
    configure_vision_pipeline,
)
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

configure_vision_pipeline(
    draw_overlays=True,
    show_debug_strip=False,
    stream_use_debug_frame=False,
    record_use_debug_frame=False,
    stream_jpeg_quality=70,
)

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
PURPLE_TARGET_X = 310
PURPLE_ALIGN_SPEED = 5
PURPLE_STOP_THRESHOLD = 61.5

PROBE_Y = 79                  # Fixed vertical look-ahead line
PROBE_LEFT_X = 179             # Inward adjusted left column
PROBE_RIGHT_X = 204            # Inward adjusted right column
FRONT_EDGE_Y_MIN = 62
FRONT_EDGE_Y_MAX = 95
TOUCH_POINT_X = 191
TOUCH_POINT_Y = 78

# STOPPING CONDITION
STOP_DISTANCE_THRESHOLD = 1.0  # Stop walking forward when distance to edge < 1

# PD Controller gains for steering alignment
KP = 18
KD = 1.2
KP_ANGLE = 2.8
KD_ANGLE = 0.9

# Track derivative terms
last_alignment_error = 0.0

class ParkingStates(Enum):
    ALIGNING = 0            # Track Wall 1 (Purple Wall)
    WALL_1_PAUSE = 1        # Settle robot momentum
    BLACK_WALL_PD_APPROACH = 2 # Continuous PD straight approach (White Edge)
    BACKWARD = 3        # Step 1: Encoder micro-positioning
    BACKWARD_TURN_1 = 4     # Step 2: Reverse entry swing
    BACKWARD_STRAIGHT = 5   # Step 2b: Straight depth segment
    BACKWARD_TURN_2 = 6
    BACKWARD_TURN_3 = 7# Step 3: Counter-steer alignment
    COMPLETED = 8           # Complete

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))

def send_command_logged(cmd):
    try:
        esp.send_command(cmd)
    except Exception as e:
        print(f"[SERIAL WRITE FAULT] Failed to transmit packet: {e}")

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

def get_front_edge_angle(track_polygon, y_min=FRONT_EDGE_Y_MIN, y_max=FRONT_EDGE_Y_MAX):
    if track_polygon is None:
        return None

    pts = track_polygon.reshape(-1, 2)
    roi_pts = []
    for x, y in pts:
        if y_min <= y <= y_max:
            roi_pts.append([float(x), float(y)])

    if len(roi_pts) < 5:
        return None

    roi_pts = np.array(roi_pts, dtype=np.float32)
    vx, vy, _, _ = cv2.fitLine(roi_pts, cv2.DIST_L2, 0, 0.01, 0.01)
    angle_deg = math.degrees(math.atan2(float(vy), float(vx)))

    while angle_deg > 90:
        angle_deg -= 180
    while angle_deg < -90:
        angle_deg += 180
    return angle_deg

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
                    _, _, track = get_latest_data()
                    touch_inside, touch_dist = get_track_distance(TOUCH_POINT_X, TOUCH_POINT_Y)
                    edge_angle = get_front_edge_angle(track.get("polygon"))

                    print(f"[TOUCH POINT ({TOUCH_POINT_X}, {TOUCH_POINT_Y})] Inside Poly: {int(touch_inside)} | Distance to Edge: {touch_dist:.2f}")
                    if edge_angle is not None:
                        print(f"[FRONT EDGE ANGLE] {edge_angle:+.2f} deg")

                    # Evaluate arrival stopping trigger using one touch point
                    if touch_dist < STOP_DISTANCE_THRESHOLD:
                        send_command_logged("0, 0, 0, R")
                        print(f"\n[⚓ WALL ARRIVAL MET] Edge distance dropped below {STOP_DISTANCE_THRESHOLD}! Proceeding to backward motions.")
                        time.sleep(1.0) # Settle vehicle momentum before parking actions
                        oc1_parking_state = ParkingStates.BACKWARD
                        run_target_sent = False
                        target_done = False
                        continue

                    if edge_angle is not None:
                        alignment_error = edge_angle
                        derivative = alignment_error - last_alignment_error
                        pd_steering = int(clamp(-(KP_ANGLE * alignment_error) - (KD_ANGLE * derivative), -55, 55))
                        last_alignment_error = alignment_error
                        send_command_logged(f"6, {pd_steering}, 0, front edge line align")
                        print(f"[PD CONTROL] Edge Angle Error: {alignment_error:+.2f} deg | Transmitted Steer: {pd_steering}\n")
                    else:
                        # Fallback if the edge line cannot be fit reliably
                        has_poly_l, dist_left = get_track_distance(PROBE_LEFT_X, PROBE_Y)
                        has_poly_r, dist_right = get_track_distance(PROBE_RIGHT_X, PROBE_Y)
                        dist_left = dist_left + 0.6
                        print(f"[FALLBACK PROBE LEFT  ({PROBE_LEFT_X}, {PROBE_Y})] Inside Poly: {int(has_poly_l)} | Distance to Edge: {dist_left:.2f}")
                        print(f"[FALLBACK PROBE RIGHT ({PROBE_RIGHT_X}, {PROBE_Y})] Inside Poly: {int(has_poly_r)} | Distance to Edge: {dist_right:.2f}")
                        alignment_error = dist_left - dist_right
                        derivative = alignment_error - last_alignment_error
                        pd_steering = int(clamp((KP * alignment_error) + (KD * derivative), -55, 55))
                        last_alignment_error = alignment_error
                        send_command_logged(f"6, {pd_steering}, 0, fallback probe align")
                        print(f"[FALLBACK PD] Alignment Error: {alignment_error:+.2f} | Transmitted Steer: {pd_steering}\n")

                # ==================================================================
                # PARKING DISPLACEMENT EXECUTION STATES
                # ==================================================================
                elif oc1_parking_state == ParkingStates.BACKWARD:
                    if run_target_sent == False:
                        send_command_logged(f"-7, 0, 1, forward target")
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
                        steer = -95 if is_clockwise else 95
                        send_command_logged(f"-8, {steer}, 53, forward target")
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
                        send_command_logged("-8, 0, 2, forward target")
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
                        steer = 95 if is_clockwise else -95
                        send_command_logged(f"-8, {steer}, 35, forward target")
                        run_target_sent = True
                        target_done = False

                    if not target_done:
                        time.sleep(0.001)
                        continue
                    else:
                        send_command_logged("-1, 0, 0, R")
                        oc1_parking_state = ParkingStates.BACKWARD_TURN_3
                        run_target_sent = False
                        target_done = False
                        continue

                elif oc1_parking_state == ParkingStates.BACKWARD_TURN_3:
                    if run_target_sent == False:
                        steer = -95 if is_clockwise else 95
                        send_command_logged(f"8, {steer}, 20, forward target")
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
