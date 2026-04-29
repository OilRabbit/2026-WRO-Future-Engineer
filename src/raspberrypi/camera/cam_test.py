from picamera2 import Picamera2 as picam2
from flask import Flask, Response
import cv2
import numpy as np
import time
import datetime
import atexit
import threading

# ==========================================
# 1. GLOBAL THREADING & CAMERA SETUP
# ==========================================

# Threading globals
output_frame = None
lock = threading.Lock()

# Initialize Flask App
app = Flask(__name__)

# Initialize Camera Global
camera = picam2()
config = camera.create_preview_configuration(main={"size": (640, 360), "format": "BGR888"})
camera.configure(config)
camera.start()
print("Warming up camera sensor...")
print("Vision System Active. Autonomous thread running...")

# Initialize Telemetry VideoWriter
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
# Frame size is 640x450 (Main view 360px high + Mask view 90px high)
video_out = cv2.VideoWriter(f"telemetry_{timestamp}.mp4", fourcc, 30.0, (640, 450))
print(f"Recording telemetry to telemetry_{timestamp}.mp4")

def cleanup_hardware():
    """Ensures hardware and files are released cleanly if the script crashes."""
    camera.stop()
    video_out.release()
    print("\nHardware locks and video file released safely.")

atexit.register(cleanup_hardware)

# ==========================================
# 2. COLOR CONVERSIONS & RANGES
# ==========================================

def RGB2HSV(rgb_list):
    c_max = max(rgb_list) / 255
    c_min = min(rgb_list) / 255
    delta_c = c_max - c_min
    
    if delta_c == 0:
        h = 0
    elif c_max == rgb_list[0] / 255:
        h = 60 * ((rgb_list[1] - rgb_list[2]) / (255 * delta_c) % 6)
    elif c_max == rgb_list[1] / 255:
        h = 60 * ((rgb_list[2] - rgb_list[0]) / (255 * delta_c) + 2)
    elif c_max == rgb_list[2] / 255:
        h = 60 * ((rgb_list[0] - rgb_list[1]) / (255 * delta_c) + 4)
    if h < 0:
        h += 360
    h /= 2

    if c_max == 0:
        s = 0
    else:
        s = delta_c / c_max
    s *= 255
    
    v = c_max
    v *= 255
    
    return np.array([h, s, v])

# Obstacle & Parking Lot RGB and HSV value range
RED_LOWER1 = RGB2HSV([41, 25 , 25])
RED_UPPER1 = RGB2HSV([255, 85, 0])
RED_LOWER2 = RGB2HSV([41, 25, 30])
RED_UPPER2 = RGB2HSV([255, 0, 4])

GREEN_LOWER = RGB2HSV([94, 99, 69])
GREEN_UPPER = RGB2HSV([0, 255, 255])

MAGENTA_LOWER = RGB2HSV([47, 40, 50])
MAGENTA_UPPER = RGB2HSV([255, 0, 85])

WHITE_LOWER = RGB2HSV([150, 150, 150])
WHITE_UPPER = RGB2HSV([255, 214, 216])

# ==========================================
# 3. VISION PROCESSING FUNCTIONS
# ==========================================

def get_pillar_center(mask, min_area = 3000):
    mask = cv2.erode(mask, None, iterations = 2)
    mask = cv2.dilate(mask, None, iterations = 2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key = cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        if area > min_area:
            x, y, w, h = cv2.boundingRect(largest_contour)
            # Keeping your offset logic
            center_x = x + (w // 2)
            center_y = y - (h // 2)
            return (center_x, center_y, w, h)
    return None

def get_track_polygon(mask, min_area = 3000):
    mask = cv2.erode(mask, None, iterations = 2)
    mask = cv2.dilate(mask, None, iterations = 2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)
        if area > min_area:
            epsilon = 0.02 * cv2.arcLength(largest_contour, True)
            polygon = cv2.approxPolyDP(largest_contour, epsilon, True)
            
            M = cv2.moments(polygon)
            if M["m00"] != 0:
                center_x = int(M["m10"] / M["m00"])
                center_y = int(M["m01"] / M["m00"])
            else:
                center_x, center_y = 0, 0
                
            return polygon, (center_x, center_y)
    return None, None

# ==========================================
# 4. BACKGROUND VISION LOOP (THE BRAIN)
# ==========================================

def vision_loop():
    """Runs continuously in the background, processing vision and recording video."""
    global output_frame, lock
    prev_time = 0
    
    while True:
        frame = camera.capture_array()

        curr_time = time.time()
        fps = 1 / (curr_time - prev_time) if prev_time > 0 else 0
        prev_time = curr_time

        display_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        cv2.putText(display_frame, f"FPS: {int(fps)}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        
        mask_r1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
        mask_r2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
        red_mask = cv2.bitwise_or(mask_r1, mask_r2)
        green_mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
        magenta_mask = cv2.inRange(hsv, MAGENTA_LOWER, MAGENTA_UPPER)
        white_mask = cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER)

        r_box = get_pillar_center(red_mask, min_area=3000)
        if r_box is not None:
            x, y, w, h = r_box
            # Reversing your offset logic to draw the actual bounding box correctly
            top_left_x = x - (w // 2)
            top_left_y = y + (h // 2)
            cv2.rectangle(display_frame, (top_left_x, top_left_y), (top_left_x+w, top_left_y+h), (0, 0, 255), 2)
            cv2.putText(display_frame, f"RED: ({x}, {y})", (int(x - w / 2), int(y + h / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        g_box= get_pillar_center(green_mask, min_area=3000)
        if g_box is not None:
            x, y, w, h = g_box
            top_left_x = x - (w // 2)
            top_left_y = y + (h // 2)
            cv2.rectangle(display_frame, (top_left_x, top_left_y), (top_left_x+w, top_left_y+h), (0, 255, 0), 2)
            cv2.putText(display_frame, f"GREEN: ({x}, {y})", (int(x - w / 2), int(y + h / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        m_box = get_pillar_center(magenta_mask, min_area=3000)
        if m_box is not None:
            x, y, w, h = m_box
            top_left_x = x - (w // 2)
            top_left_y = y + (h // 2)
            cv2.rectangle(display_frame, (top_left_x, top_left_y), (top_left_x+w, top_left_y+h), (255, 0, 255), 2)
            cv2.putText(display_frame, f"MAGENTA: ({x}, {y})", (int(x - w / 2), int(y + h / 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        
        w_poly, w_center = get_track_polygon(white_mask, min_area=3000)
        if w_poly is not None:
            cv2.drawContours(display_frame, [w_poly], 0, (255, 255, 255), 3)
            c_x, c_y = w_center
            cv2.circle(display_frame, (c_x, c_y), 5, (0, 0, 255), -1)
            cv2.putText(display_frame, f"TRACK X: {c_x}", (c_x - 40, c_y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        red_mask_color = cv2.cvtColor(red_mask, cv2.COLOR_GRAY2BGR)
        green_mask_color = cv2.cvtColor(green_mask, cv2.COLOR_GRAY2BGR)
        magenta_mask_color = cv2.cvtColor(magenta_mask, cv2.COLOR_GRAY2BGR)
        white_mask_color = cv2.cvtColor(white_mask, cv2.COLOR_GRAY2BGR)

        small_red = cv2.resize(red_mask_color, (160, 90))
        small_green = cv2.resize(green_mask_color, (160, 90))
        small_magenta = cv2.resize(magenta_mask_color, (160, 90))
        small_white = cv2.resize(white_mask_color, (160, 90))
        masks_combined = cv2.hconcat([small_red, small_green, small_magenta, small_white])

        final_output = cv2.vconcat([display_frame, masks_combined])

        # Save to MP4 telemetry file safely
        video_out.write(final_output)

        # Encode for web stream
        ret, buffer = cv2.imencode('.jpg', final_output)
        
        # Safely overwrite the global holding area
        with lock:
            output_frame = buffer.tobytes()

# ==========================================
# 5. FLASK SERVER
# ==========================================

def generate_frames():
    """Yields the newest frame from the holding area whenever a browser connects."""
    global output_frame, lock
    while True:
        with lock:
            if output_frame is None:
                continue
            frame_bytes = output_frame
            
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        # Prevent the web server loop from consuming 100% CPU
        time.sleep(0.02) 

@app.route('/')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        # Ignite the autonomous background vision thread
        t = threading.Thread(target=vision_loop, daemon=True)
        t.start()
        
        # Start the Flask web server
        app.run(host='0.0.0.0', port=5000, threaded=True)
    except KeyboardInterrupt:
        # Handled cleanly by atexit
        pass
