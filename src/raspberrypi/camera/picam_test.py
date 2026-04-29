from picamera2 import Picamera2 as picam2
from flask import Flask, Response
import cv2
import numpy as np
import time

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

def get_pillar_center(mask, min_area=3000):
    """Finds the largest object in a mask and returns its center X coordinate."""
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_cnt)
        if area > min_area:
            x, y, w, h = cv2.boundingRect(largest_cnt)
            center_x = x + (w // 2)
            return (x, y, w, h), center_x
    return None, None

def get_track_polygon(mask, min_area=3000):
    """Finds the largest contour, approximates it to a polygon, and calculates its center."""
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        largest_cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_cnt)
        
        if area > min_area:
            # 1. Simplify the contour into a clean polygon
            # Epsilon determines how closely the polygon hugs the raw pixels. 
            # 0.02 is a great balance for smoothing out jagged track edges.
            epsilon = 0.02 * cv2.arcLength(largest_cnt, True)
            polygon = cv2.approxPolyDP(largest_cnt, epsilon, True)
            
            # 2. Calculate the center of mass (Centroid) of the polygon
            M = cv2.moments(polygon)
            if M["m00"] != 0:
                center_x = int(M["m10"] / M["m00"])
                center_y = int(M["m01"] / M["m00"])
            else:
                center_x, center_y = 0, 0
                
            return polygon, (center_x, center_y)
            
    return None, None

def generate_frames():
    prev_time = 0
    """Generator function to continuously capture and process frames."""
    while True:
        # 1. Capture frame (RGB888)
        frame = camera.capture_array()

        curr_time = time.time()
        fps = 1 / (curr_time - prev_time) if prev_time > 0 else 0
        prev_time = curr_time

        # 2. Prepare Display Frame
        display_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        cv2.putText(display_frame, f"FPS: {int(fps)}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # 3. Convert to HSV for processing
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        
        # 4. Create Color Masks
        mask_r1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
        mask_r2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
        red_mask = cv2.bitwise_or(mask_r1, mask_r2)
        green_mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
        magenta_mask = cv2.inRange(hsv, MAGENTA_LOWER, MAGENTA_UPPER)
        white_mask = cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER)

        # 5. Process Pillars
        r_box, r_x = get_pillar_center(red_mask, min_area=3000)
        if r_x is not None:
            x, y, w, h = r_box
            cv2.rectangle(display_frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
            cv2.putText(display_frame, f"RED X: {r_x}", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        g_box, g_x = get_pillar_center(green_mask, min_area=3000)
        if g_x is not None:
            x, y, w, h = g_box
            cv2.rectangle(display_frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(display_frame, f"GREEN X: {g_x}", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        m_box, m_x = get_pillar_center(magenta_mask, min_area=3000)
        if m_x is not None:
            x, y, w, h = m_box
            cv2.rectangle(display_frame, (x, y), (x+w, y+h), (255, 0, 255), 2)
            cv2.putText(display_frame, f"MAGENTA X: {m_x}", (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
        
        w_poly, w_center = get_track_polygon(white_mask, min_area=3000)
        if w_poly is not None:
            cv2.drawContours(display_frame, [w_poly], 0, (255, 255, 255), 3)
            c_x, c_y = w_center
            cv2.circle(display_frame, (c_x, c_y), 5, (0, 0, 255), -1)
            cv2.putText(display_frame, f"TRACK X: {c_x}", (c_x - 40, c_y - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        # 6. Combine masks into color images so they can be stacked
        red_mask_color = cv2.cvtColor(red_mask, cv2.COLOR_GRAY2BGR)
        green_mask_color = cv2.cvtColor(green_mask, cv2.COLOR_GRAY2BGR)
        magenta_mask_color = cv2.cvtColor(magenta_mask, cv2.COLOR_GRAY2BGR)
        white_mask_color = cv2.cvtColor(white_mask, cv2.COLOR_GRAY2BGR)

        # Resize masks to be smaller (half width) and stack them side-by-side
        small_red = cv2.resize(red_mask_color, (160, 90))
        small_green = cv2.resize(green_mask_color, (160, 90))
        small_magenta = cv2.resize(magenta_mask_color, (160, 90))
        small_white = cv2.resize(white_mask_color, (160, 90))
        masks_combined = cv2.hconcat([small_red, small_green, small_magenta, small_white])

        # Stack the main frame on top of the masks
        final_output = cv2.vconcat([display_frame, masks_combined])

        # 7. Encode as JPEG for web streaming
        ret, buffer = cv2.imencode('.jpg', final_output)
        frame_bytes = buffer.tobytes()

        # Yield the frame in byte format compatible with MJPEG
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

# Initialize Flask App
app = Flask(__name__)

# Initialize Camera Global
camera = picam2()
config = camera.create_preview_configuration(main={"size": (640, 360), "format": "BGR888"})
camera.configure(config)
camera.start()
print("Warming up camera sensor...")
#time.sleep(2)
print("Vision System Active. Streaming to web browser...")

@app.route('/')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    try:
        # Run the server on all available network interfaces, port 5000
        app.run(host='0.0.0.0', port=5000, threaded=True)
    finally:
        camera.stop()
        print("Camera shut down cleanly.")
