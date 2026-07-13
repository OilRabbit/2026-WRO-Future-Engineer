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
PROBE_Y_FORWARD = 134            
PROBE_LEFT_X_FORWARD = 320       
PROBE_RIGHT_X_FORWARD = 360      

TARGET_DIST = 1.0
MIN_STOP_DIST = 0.9            
MAX_STOP_DIST = 1.1            

KP_SPEED = 10.0
KP = 30
KD = 1.2

last_alignment_error = 0.0
last_direction = None          
oscillation_count = 0          

class ParkingStates(Enum):
    BLACK_WALL_PD_APPROACH = 0 
    COMPLETED = 1              
    RIGHT_TURN_90 = 2          
    STEERING_REALIGN = 3       
    BACKWARD_ENCODER_MOVE = 4  
    FINAL_TURN_90 = 5          
    REAR_SAFETY_BACKOFF = 6    
    FINAL_CORRECTION_TURN = 7  

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

oc1_parking_state = ParkingStates.BLACK_WALL_PD_APPROACH

try:
    print("IDLE")
    while True:
        if reset_OC:
            if run_OC1:
                print("[INIT] Starting Direct PD Wall Approach Solution...")
                oc1_parking_state = ParkingStates.BLACK_WALL_PD_APPROACH
                target_done = False
                run_target_sent = False
                last_alignment_error = 0.0
                last_direction = None
                oscillation_count = 0
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
                # CONTINUOUS PD APPROACH WITH OSCILLATION SAFETIES
                # ==================================================================
                if oc1_parking_state == ParkingStates.BLACK_WALL_PD_APPROACH:
                    has_poly_l, dist_left = get_track_distance(PROBE_LEFT_X_FORWARD, PROBE_Y_FORWARD)
                    has_poly_r, dist_right = get_track_distance(PROBE_RIGHT_X_FORWARD, PROBE_Y_FORWARD)
                    
                    dist_left = dist_left + 0.5

                    print(f"[PROBE LEFT  ({PROBE_LEFT_X_FORWARD}, {PROBE_Y_FORWARD})] Distance: {dist_left:.2f}")
                    print(f"[PROBE RIGHT ({PROBE_RIGHT_X_FORWARD}, {PROBE_Y_FORWARD})] Distance: {dist_right:.2f}")

                    if (MIN_STOP_DIST <= dist_left <= MAX_STOP_DIST) and \
                       (MIN_STOP_DIST <= dist_right <= MAX_STOP_DIST):
                        
                        send_command_logged("0, 0, 0, R")
                        oc1_parking_state = ParkingStates.COMPLETED
                        run_target_sent = False
                        target_done = False
                        continue

                    mean_distance = (dist_left + dist_right) / 2.0
                    dist_error = mean_distance - TARGET_DIST
                    
                    current_direction = 1 if dist_error >= 0 else -1
                    if last_direction is not None and current_direction != last_direction:
                        oscillation_count += 1
                        if oscillation_count >= 2:
                            send_command_logged("0, 0, 0, R")
                            oc1_parking_state = ParkingStates.COMPLETED
                            run_target_sent = False
                            target_done = False
                            continue
                            
                    last_direction = current_direction
                    base_speed = float(clamp(dist_error * KP_SPEED, -6.0, 6.0))
                    speed_mode = "smooth forward approach" if base_speed >= 0 else "smooth overshoot backing"

                    alignment_error = dist_left - dist_right
                    derivative = alignment_error - last_alignment_error

                    pd_steering = int(clamp((KP * alignment_error) + (KD * derivative), -55, 55))
                    last_alignment_error = alignment_error

                    if base_speed < 0:
                        pd_steering = -pd_steering

                    send_command_logged(f"{base_speed:.1f}, {pd_steering}, 0, {speed_mode}")

                # ==================================================================
                # STANDBY COMPLETED HOLDING STATE
                # ==================================================================
                elif oc1_parking_state == ParkingStates.COMPLETED:
                    oc1_parking_state = ParkingStates.RIGHT_TURN_90
                    run_target_sent = False
                    target_done = False
                    continue

                # ==================================================================
                # STATE: 90 DEGREE RIGHT TURN VIA ENCODER TARGETING
                # ==================================================================
                elif oc1_parking_state == ParkingStates.RIGHT_TURN_90:
                    if run_target_sent == False:
                        send_command_logged("-8.5, 100, 61, forward target")
                        run_target_sent = True
                        target_done = False

                    if not target_done:
                        time.sleep(0.001)
                        continue
                    else:
                        send_command_logged("0, 0, 0, R") 
                        oc1_parking_state = ParkingStates.STEERING_REALIGN
                        run_target_sent = False
                        target_done = False
                        continue

                # ==================================================================
                # STATE: ACTIVE STEERING FRICTION BREAKER
                # ==================================================================
                elif oc1_parking_state == ParkingStates.STEERING_REALIGN:
                    send_command_logged("0, -30, 0, friction break pulse")
                    time.sleep(0.15)  
                    send_command_logged("0, 0, 0, R") 
                    oc1_parking_state = ParkingStates.BACKWARD_ENCODER_MOVE
                    continue

                # ==================================================================
                # STATE: ENCODER DRIVEN BACKWARD MANEUVER (72 TICKS TARGET)
                # ==================================================================
                elif oc1_parking_state == ParkingStates.BACKWARD_ENCODER_MOVE:
                    if run_target_sent == False:
                        send_command_logged("-8.5, 0, 72, forward target")
                        run_target_sent = True
                        target_done = False

                    if not target_done:
                        time.sleep(0.001)
                        continue
                    else:
                        send_command_logged("0, 0, 0, R")
                        oc1_parking_state = ParkingStates.FINAL_TURN_90
                        run_target_sent = False
                        target_done = False
                        continue

                # ==================================================================
                # STATE: FINAL TURN MANEUVER VIA ENCODER TARGETING (45 TICKS TARGET)
                # ==================================================================
                elif oc1_parking_state == ParkingStates.FINAL_TURN_90:
                    if run_target_sent == False:
                        send_command_logged("8.5, 100, 45, forward target")
                        run_target_sent = True
                        target_done = False

                    if not target_done:
                        time.sleep(0.001)
                        continue
                    else:
                        send_command_logged("0, -30, 0, friction break pulse")
                        time.sleep(0.15) 
                        send_command_logged("0, 0, 0, R")
                        oc1_parking_state = ParkingStates.REAR_SAFETY_BACKOFF
                        run_target_sent = False
                        target_done = False
                        continue

                # ==================================================================
                # STATE: REAR SAFETY BACKOFF MANEUVER (16 TICKS TARGET)
                # ==================================================================
                elif oc1_parking_state == ParkingStates.REAR_SAFETY_BACKOFF:
                    if run_target_sent == False:
                        # Target updated from 18 to 16 Ticks
                        send_command_logged("-8.5, -30, 16, forward target")
                        run_target_sent = True
                        target_done = False

                    if not target_done:
                        time.sleep(0.001)
                        continue
                    else:
                        send_command_logged("0, 0, 0, R")
                        oc1_parking_state = ParkingStates.FINAL_CORRECTION_TURN
                        run_target_sent = False
                        target_done = False
                        continue

                # ==================================================================
                # STATE: FINAL BACKWARD CORRECTION TURN WITH DYNAMIC UN-SHACKLE
                # ==================================================================
                elif oc1_parking_state == ParkingStates.FINAL_CORRECTION_TURN:
                    if run_target_sent == False:
                        send_command_logged("-8.5, -100, 47, forward target")
                        run_target_sent = True
                        target_done = False

                    if not target_done:
                        time.sleep(0.001)
                        continue
                    else:
                        # HIGH-DYNAMIC DE-SHACKLE ROUTINE:
                        print("[SHAKE RESET] Hard flick right to release servo linkage compression...")
                        send_command_logged("0, 60, 0, dynamic shake right")
                        time.sleep(0.18)
                        
                        print("[SHAKE RESET] Command true center...")
                        send_command_logged("0, 0, 0, straight hold")
                        time.sleep(0.10)
                        
                        send_command_logged("0, 0, 0, R")
                        print("[SUCCESS] Hardware steering alignment cleared dynamically.")
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
