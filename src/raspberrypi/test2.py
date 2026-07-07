import cv2
import time
import numpy as np
from picamera2 import Picamera2 as picam2
from camera.camera_utils import (
    start_vision_system,
    start_web_server,
    stop_vision_system,
    configure_vision_pipeline
)

# ======================================================================
# CONFIGURATION CONSTANTS
# ======================================================================
TARGET_POINT = (200, 150)  # The exact (X, Y) coordinate you want to sample

# Camera frame size matching your current profile setup
video_size = (400, 225)
sensor_video_size = (2304, 1296)
target_frame_duration_us = 10000

# ======================================================================
# INITIALIZATION
# ======================================================================
print("======= HSV Tester Init =======")
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

# Turn off standard debug strips to keep frame clean
configure_vision_pipeline(
    draw_overlays=True,
    show_debug_strip=False,
    stream_use_debug_frame=False,
    record_use_debug_frame=False,
    stream_jpeg_quality=80,
)

# Spin up web server streaming so you can see live feedback at http://<your_pi_ip>:5000
start_web_server(host='0.0.0.0', port=5000)
print("Streaming active. Check the camera feed in your web browser.")
print("======= Ready: Beginning Live Sampling =======")

# ======================================================================
# SAMPLING LOOP
# ======================================================================
try:
    while True:
        # 1. Grab raw image frame from the camera stream array
        # Note: Picamera2 outputs standard RGB/BGR arrays depending on format
        frame = camera.capture_array()
        
        # 2. Convert standard BGR layout to HSV spectrum matrix
        # This matches OpenCV's internal range bounds: H: 0-180, S: 0-255, V: 0-255
        bgr_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv_frame = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2HSV)
        
        # 3. Read the single pixel value array at the specified (Y, X) index
        x_coord, y_coord = TARGET_POINT
        pixel_hsv = hsv_frame[y_coord, x_coord]
        
        hue = pixel_hsv[0]
        saturation = pixel_hsv[1]
        value = pixel_hsv[2]
        
        # 4. Print clean real-time status telemetry to terminal
        print(f"[HSV READOUT] At Point {TARGET_POINT} -> Hue: {hue:3d} | Saturation: {saturation:3d} | Value: {value:3d}")
        
        # 5. Visual overlay helper: Put a small marker crosshair on the frame 
        # so you know visually exactly where the point lands on your magenta target
        cv2.drawMarker(bgr_frame, TARGET_POINT, (0, 0, 255), markerType=cv2.MARKER_CROSS, markerSize=10, thickness=2)
        cv2.putText(bgr_frame, f"HSV: {hue},{saturation},{value}", (x_coord + 10, y_coord + 5), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)
        
        # Give your CPU a tiny break between frame calculations
        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nShutting down software pipeline gracefully...")
    camera.stop()
    stop_vision_system()
    print("Cleanup complete.")
