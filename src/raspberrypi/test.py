import cv2
import time
import datetime
from esp_com.communication import ESP32Communicator
from camera.camera_utils import (
    start_vision_system,
    start_web_server,
    get_latest_data,
    stop_vision_system,
    set_all_color_detection,
    configure_vision_pipeline
)
from picamera2 import Picamera2 as picam2

# ======================================================================
# CONFIGURATION CONSTANTS FOR PURPLE-ANCHOR PARKING
# ======================================================================
PURPLE_TARGET_X = 200         # Absolute position of the true frame center (400 / 2)
PURPLE_ALIGN_SPEED = 9        # Baseline precision driving speed
PURPLE_STOP_THRESHOLD = 150   # Area threshold -> close enough to trigger final stop

# Hardware Video Profiles (matching camera_utils configuration)
video_size = (400, 225)
sensor_video_size = (2304, 1296)
target_frame_duration_us = 10000

# ======================================================================
# INITIALIZATION
# ======================================================================
print("======= Parking Test Init =======")
esp = ESP32Communicator()
esp.connect()

camera = picam2()
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

start_vision_system(camera, record_mp4=True)
print("Streaming: Activated")
start_web_server(host='0.0.0.0', port=5000)

print("======= System Ready: Starting Test Loop =======")

# ======================================================================
# HELPER ACTIONS
# ======================================================================
def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))

def send_command_logged(cmd):
    """Sends a string command via the serial port connection interface."""
    try:
        esp.send_command(cmd)
    except Exception as e:
        print(f"[SERIAL WRITE FAULT] Failed to transmit packet: {e}")

def esp_reply_flush():
    """Flushes background buffer lines coming from the MCU."""
    try:
        reply = esp.read_message()
        if reply is not None:
            print(f"[ESP IN] {reply}")
    except Exception as e:
        print(f"[SERIAL GLITCH] {e}")

# ======================================================================
# CORE ROUTINE RUNNER
# ======================================================================
try:
    # Trigger initial runtime state execution flag for target microcontroller
    send_command_logged("OC2")

    # Isolate color arrays to focus purely on tracking the magenta pipeline elements
    set_all_color_detection(red=False, green=False, magenta=True)

    while True:
        esp_reply_flush()

        # Extract sensor telemetry directly from memory sharing pipelines
        obstacle, parking, track = get_latest_data()

        purple_x = parking.get("center_x", 0)
        purple_width = parking.get("width", 0)
        purple_height = parking.get("height", 0)

        # Calculate normalized profile area matching your layout rule definitions
        purple_area = (purple_width * purple_height) // 100

        # Real-time console log monitoring to assist manual bench tuning
        print(f"[TRACKING] X: {purple_x} | Area: {purple_area} (Target X: {PURPLE_TARGET_X}, Stop Min: {PURPLE_STOP_THRESHOLD})")

        # Threshold set to > 10 matching your recent runtime parameters
        if purple_x != 0 and purple_area > 10:

            # Case A: Inside the zone -> STOP
            if purple_area >= PURPLE_STOP_THRESHOLD:
                send_command_logged("0, 0, 0, motor stop")
                print(f"\n[🛑 TARGET REACHED] Parking completed! Target space achieved: Area {purple_area} >= {PURPLE_STOP_THRESHOLD}")
                break

            # Case B: Approaching -> Actively steer to keep the wall centered
            else:
                error = purple_x - PURPLE_TARGET_X
                kp_purple = 0.55  # Proportional feedback multiplier (smoothed from 1.55)

                # Calculate steering to track the wall center
                align_steering = int(clamp(error * kp_purple, -65, 65))

                # Dispatches using your centralized logging structure
                send_command_logged(f"{PURPLE_ALIGN_SPEED}, {align_steering}, 0, purple align")
                print(f" -> [COMMAND] Heading Error: {error} | Driving Steering Action: {align_steering}")
        else:
            # Fallback behavior: Safe hold positioning if tracking breaks
            send_command_logged("0, 0, 0, tracking lost holding")
            print(" -> [SEARCHING] Purple target anchor dropped out of tracking frame.")

        time.sleep(0.02)

except KeyboardInterrupt:
    print("\n[TEST INTERRUPTED] Closing connection pipelines...")
    try:
        send_command_logged("0, 0, 0, emergency_stop")
    except Exception:
        pass
    esp.disconnect()
    stop_vision_system()
    print("Cleanup sequence complete.")
