import cv2
import numpy as np
import time
import datetime
import threading
import atexit
from flask import Flask, Response

_app = Flask(__name__)
_camera = None
_video_out = None
_output_frame = None
_frame_lock = threading.Lock()
_shared_hsv = None
_hsv_lock = threading.Lock()

nearest_obstacle = {"color": None, "center_x": 0, "center_y": 0, "width": 0, "height": 0}
parkinglot_data = {"center_x": 0, "center_y": 0, "width": 0, "height": 0}
track_data = {"polygon": None, "center_x": 0, "center_y": 0}

_display_masks = {"red": None, "green": None, "magenta": None, "white": None}

_data_lock = threading.Lock()

# Convert RGB (R, G, B = [0, 255]) to HSV (H = [0, 180), S = [0, 255], V = [0, 255])
def RGB2HSV(rgb_list):
	c_max = max(rgb_list) / 255
	c_min = min(rgb_list) / 255
	delta_c = c_max - c_min

	if delta_c == 0: h = 0
	elif c_max == rgb_list[0] / 255: h = 60 * ((rgb_list[1] - rgb_list[2]) / (255 * delta_c) % 6)
	elif c_max == rgb_list[1] / 255: h = 60 * ((rgb_list[2] - rgb_list[0]) / (255 * delta_c) + 2)
	elif c_max == rgb_list[2] / 255: h = 60 * ((rgb_list[0] - rgb_list[1]) / (255 * delta_c) + 4)
	
	if h < 0: h += 360
	h /= 2
	s = 0 if c_max == 0 else (delta_c / c_max) * 255
	v = c_max * 255
	return np.array([h, s, v])

RED_LOWER1 = np.array([0, 100, 41]) #RGB2HSV([0, 80, 80])
RED_UPPER1 = np.array([10, 255, 255]) #RGB2HSV([12, 255, 255])
RED_LOWER2 = np.array([176, 100, 41]) #RGB2HSV([41, 25, 30])
RED_UPPER2 = np.array([179, 255, 255]) #RGB2HSV([255, 0, 4])

GREEN_LOWER = np.array([40, 60, 50]) #RGB2HSV([94, 99, 69])
GREEN_UPPER = np.array([85, 255, 255]) #RGB2HSV([0, 255, 255])

MAGENTA_LOWER = np.array([145, 70, 60]) #RGB2HSV([47, 40, 50])
MAGENTA_UPPER = np.array([175, 255, 255]) #RGB2HSV([255, 0, 85])

WHITE_LOWER = RGB2HSV([136, 136, 136])
WHITE_UPPER = RGB2HSV([255, 214, 216])

BLUE_LOWER = RGB2HSV([40, 47, 50]) # np.array([90, 40, 40]) # RGB2HSV([40, 47, 50])
BLUE_UPPER = RGB2HSV([95, 0, 255]) # np.array([140, 255, 255]) # RGB2HSV([95, 0, 255]) 

ORANGE_LOWER = np.array([0, 70, 70])
ORANGE_UPPER = np.array([35, 255, 255])

# Get the center coordinates, width and height of a pillar with specific filter
def get_pillar_center(mask, min_area = 20):
	mask = cv2.erode(mask, None, iterations=2)
	mask = cv2.dilate(mask, None, iterations=2)
	contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	if contours:
		largest_contour = max(contours, key=cv2.contourArea)
		area = cv2.contourArea(largest_contour)
		area = area // 100
		if area > min_area:
			x, y, w, h = cv2.boundingRect(largest_contour)
			center_x = x + (w // 2)
			center_y = y + (h // 2)
			return (center_x, center_y, w, h)
	return None

# Get the center of mass coordinates of the largest white polygon
def get_track_polygon(mask, min_area = 20):
	mask = cv2.erode(mask, None, iterations = 2)
	mask = cv2.dilate(mask, None, iterations = 2)
	contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	if contours:
		largest_contour = max(contours, key = cv2.contourArea)
		area = cv2.contourArea(largest_contour)
		area = area // 100
		if area > min_area:
			epsilon = 0.005 * cv2.arcLength(largest_contour, True)
			polygon = cv2.approxPolyDP(largest_contour, epsilon, True)
			M = cv2.moments(polygon)
			if M["m00"] != 0:
				center_x = int(M["m10"] / M["m00"])
				center_y = int(M["m01"] / M["m00"])
			else:
				center_x, center_y = 0, 0
			return polygon, (center_x, center_y)
	return None, None

# Blacken out things other than the largest white polygon when using the white filter
def isolate_largest_blob(mask):
	clean_mask = np.zeros_like(mask)
	contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	if contours:
		largest_cnt = max(contours, key = cv2.contourArea)
		cv2.drawContours(clean_mask, [largest_cnt], 0, 255, -1)
	return clean_mask

# Thread function for scanning obstacles
def _scan_obstacle_thread():
	global _shared_hsv, nearest_obstacle, _display_masks
	while True:
		with _hsv_lock:
			if _shared_hsv is None:
				time.sleep(0.01)
				continue
			hsv = _shared_hsv.copy()
		    
		mask_r1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
		mask_r2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
		red_mask = cv2.bitwise_or(mask_r1, mask_r2)
		green_mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
		
		r_box = get_pillar_center(red_mask, min_area = 3)
		g_box = get_pillar_center(green_mask, min_area = 3)
		
		largest = None
		color = None
		r_area = (r_box[2] * r_box[3]) // 100 if r_box else 0
		g_area = (g_box[2] * g_box[3]) // 100 if g_box else 0
		
		if r_area > 0 or g_area > 0:
			if r_area > g_area:
				largest = r_box
				color = "RED"
			else:
				largest = g_box
				color = "GREEN"
			
		with _data_lock:
			_display_masks["red"] = red_mask
			_display_masks["green"] = green_mask
			if largest:
				nearest_obstacle.update({"color": color, "center_x": largest[0], "center_y": largest[1], "width": largest[2], "height": largest[3]})
			else:
				nearest_obstacle["color"] = None
		time.sleep(0.01)

# Thread function for scanning the parking lot
def _scan_parkinglot_thread():
	global _shared_hsv, parkinglot_data, _display_masks
	while True:
		with _hsv_lock:
			if _shared_hsv is None:
				time.sleep(0.01)
				continue
			hsv = _shared_hsv.copy()
		    
		magenta_mask = cv2.inRange(hsv, MAGENTA_LOWER, MAGENTA_UPPER)
		m_box = get_pillar_center(magenta_mask, min_area = 20)
		with _data_lock:
			_display_masks["magenta"] = magenta_mask
			if m_box:
				parkinglot_data.update({"center_x": m_box[0], "center_y": m_box[1], "width": m_box[2], "height": m_box[3]})
			else:
				parkinglot_data["center_x"] = 0
		time.sleep(0.01)

def _scan_track_thread():
	global _shared_hsv, track_data, _display_masks
	while True:
		with _hsv_lock:
			if _shared_hsv is None:
				time.sleep(0.01)
				continue
			hsv = _shared_hsv.copy()
		    
		raw_white_mask = cv2.inRange(hsv, WHITE_LOWER, WHITE_UPPER)
		
		mask_r1 = cv2.inRange(hsv, RED_LOWER1, RED_UPPER1)
		mask_r2 = cv2.inRange(hsv, RED_LOWER2, RED_UPPER2)
		red_mask = cv2.bitwise_or(mask_r1, mask_r2)
		
		green_mask = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)
		blue_mask = cv2.inRange(hsv, BLUE_LOWER, BLUE_UPPER)
		orange_mask = cv2.inRange(hsv, ORANGE_LOWER, ORANGE_UPPER)
		
		combined_mask = raw_white_mask
		for m in [red_mask, green_mask, blue_mask, orange_mask]:
			combined_mask = cv2.bitwise_or(combined_mask, m)
			
		track_mask = isolate_largest_blob(combined_mask)
		w_poly, w_center = get_track_polygon(track_mask, min_area = 20)
		
		with _data_lock:
			_display_masks["white"] = track_mask
			if w_poly is not None:
				track_data.update({"polygon": w_poly, "center_x": w_center[0], "center_y": w_center[1]})
			else:
				track_data["polygon"] = None
		time.sleep(0.01)

# Thread function for scanning the track
def _vision_loop():
	global _camera, _video_out, _shared_hsv, _output_frame
	prev_time = 0
    
	empty_mask = np.zeros((360, 640), dtype = np.uint8)
    
	while True:
		frame = _camera.capture_array()
		
		curr_time = time.time()
		fps = 1 / (curr_time - prev_time) if prev_time > 0 else 0
		prev_time = curr_time
	
		display_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
		cv2.putText(display_frame, f"FPS: {int(fps)}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
	
		hsv_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
		with _hsv_lock:
			_shared_hsv = hsv_frame
	
		with _data_lock:
			obs = nearest_obstacle.copy()
			mag = parkinglot_data.copy()
			trk = track_data.copy()
			m_red = _display_masks["red"] if _display_masks["red"] is not None else empty_mask
			m_green = _display_masks["green"] if _display_masks["green"] is not None else empty_mask
			m_mag = _display_masks["magenta"] if _display_masks["magenta"] is not None else empty_mask
			m_white = _display_masks["white"] if _display_masks["white"] is not None else empty_mask
	
		if obs["color"]:
			x, y, w, h = obs["center_x"], obs["center_y"], obs["width"], obs["height"]
			tl_x, tl_y = int(x - w/2), int(y - h/2)
			c = (0, 0, 255) if obs["color"] == "RED" else (0, 255, 0)
			cv2.rectangle(display_frame, (tl_x, tl_y), (tl_x+w, tl_y+h), c, 2)
			cv2.putText(display_frame, f"{obs['color']} ({x}, {y}), A: {w * h // 100}", (tl_x, tl_y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, c, 2)
	
		if mag["center_x"] != 0:
			x, y, w, h = mag["center_x"], mag["center_y"], mag["width"], mag["height"]
			tl_x, tl_y = int(x - w/2), int(y - h/2)
			cv2.rectangle(display_frame, (tl_x, tl_y), (tl_x+w, tl_y+h), (255, 0, 255), 2)
			cv2.putText(display_frame, f"Mag ({x}, {y})", (tl_x, tl_y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
	
		if trk["polygon"] is not None:
			cv2.drawContours(display_frame, [trk["polygon"]], 0, (255, 255, 255), 3)
			cv2.circle(display_frame, (trk["center_x"], trk["center_y"]), 5, (0, 0, 255), -1)
			cv2.putText(display_frame, f"TRACK X: {trk['center_x']}", (trk["center_x"]-40, trk["center_y"]-15), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
	
		s_red = cv2.resize(cv2.cvtColor(m_red, cv2.COLOR_GRAY2BGR), (160, 90))
		s_grn = cv2.resize(cv2.cvtColor(m_green, cv2.COLOR_GRAY2BGR), (160, 90))
		s_mag = cv2.resize(cv2.cvtColor(m_mag, cv2.COLOR_GRAY2BGR), (160, 90))
		s_wht = cv2.resize(cv2.cvtColor(m_white, cv2.COLOR_GRAY2BGR), (160, 90))
		masks_combined = cv2.hconcat([s_red, s_grn, s_mag, s_wht])
	
		final_output = cv2.vconcat([display_frame, masks_combined])
	
		if _video_out is not None:
			_video_out.write(final_output)
	
		ret, buffer = cv2.imencode('.jpg', final_output)
		with _frame_lock:
			_output_frame = buffer.tobytes()

# The main function to start the vision and scanning process
def start_vision_system(camera_instance, record_mp4=True):
	global _camera, _video_out
	_camera = camera_instance
    
	if record_mp4:
		fourcc = cv2.VideoWriter_fourcc(*'mp4v')
		timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
		_video_out = cv2.VideoWriter(f"vid_{timestamp}.mp4", fourcc, 30.0, (640, 450))
		print(f"Recording: Activated")
	else:
		_video_out = None
		print("Recording: Not activated")

	threading.Thread(target=_scan_obstacle_thread, daemon=True).start()
	threading.Thread(target=_scan_parkinglot_thread, daemon=True).start()
	threading.Thread(target=_scan_track_thread, daemon=True).start()
	
	threading.Thread(target=_vision_loop, daemon=True).start()
	print("Vision: Activated")

# Get the latest data of the nearest obstacle, parking lot, and the track
def get_latest_data():
	with _data_lock:
		return nearest_obstacle.copy(), parkinglot_data.copy(), track_data.copy()

# Function to fetch the frames to the web server for live streaming
def _generate_web_frames():
	global _output_frame, _frame_lock
	while True:
		with _frame_lock:
			if _output_frame is None:
				continue
			frame_bytes = _output_frame
		yield (b'--frame\r\n'
			b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
		time.sleep(0.02) 

# Display high-speed digital flipbook as live streaming
@_app.route('/')
def _video_feed():
	return Response(_generate_web_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Thread function to run the web server for live streaming
def _run_server_thread(host, port):
	_app.run(host=host, port=port, threaded=True, use_reloader=False)

# Function to start the web server
def start_web_server(host='0.0.0.0', port=5000):
	t = threading.Thread(target=_run_server_thread, args=(host, port), daemon=True)
	t.start()
	print("Web Server Activated")

# Clean up function to stop the camera
def _cleanup_hardware():
	global _camera, _video_out
	if _camera is not None:
		_camera.stop()
	if _video_out is not None:
		_video_out.release()

# Safely output the video to mp4
def stop_vision_system():
	global _video_out
	if _video_out is not None:
		_video_out.release()
		_video_out = None
	print("Video saved")

atexit.register(_cleanup_hardware)

# Check if the given point lies inside the polygon, and return the shortest distance to the nearest edge
def get_track_distance(x, y):
	with _data_lock:
		poly = track_data["polygon"]
	    
	if poly is None:
		return False, 0.0
	
	raw_distance = cv2.pointPolygonTest(poly, (float(x), float(y)), True)
	
	is_inside = raw_distance >= 0
	exact_distance = abs(raw_distance)
	return is_inside, exact_distance

if __name__ == "__main__":
	from picamera2 import Picamera2 as picam2
	print("=======Initializing=======")
	camera = picam2()
	
	full_sensor_res = camera.sensor_resolution
	
	config = camera.create_video_configuration(main={"size": (640, 360), "format": "BGR888"}, sensor={"output_size": full_sensor_res})
	
	camera.configure(config)
	camera.start()
	
	print(camera.camera_configuration)
	print("Camera: Activated")
	
	start_vision_system(camera, False)
	
	live_streaming = True
	if live_streaming:
		print("Streaming: Activated")
		start_web_server(host='0.0.0.0', port=5000)
	else:
		print("Streaming: Not streaming")
	
	print("=======Initialized=======")
	
	try:
		while True:
			obstacle, parking, track = get_latest_data()
			if obstacle["color"] == "RED":
				pass
			elif track["center_x"] != 0:
				pass
			time.sleep(0.02)
	
	except KeyboardInterrupt:
		print("\nShutting down...")
		stop_vision_system()
		time.sleep(0.5)
