import numpy as np
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
	get_track_distance,
	configure_vision_pipeline
)
from picamera2 import Picamera2 as picam2
from enum import Enum

# ESP communication and Picam init
print("======= Init =======")

esp = ESP32Communicator()
esp.connect()
camera = picam2()

# Matching the optimized frame and sensor metrics from parking2.py
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

# Checkpoints (default as clockwise case)
angle = 0 
speed = 0
speed_var = 0
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
enter_flag = 0

# Tracking parameter for dynamic parking choice
last_seen_color = None

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

# Parking-specific sub-states
class ParkingRedStates(Enum):
	ALIGNING = 0            
	WALL_1_PAUSE = 1        
	BLACK_WALL_PD_APPROACH = 2 
	BACKWARD = 3        
	BACKWARD_TURN_1 = 4     
	BACKWARD_STRAIGHT = 5   
	BACKWARD_TURN_2 = 6     
	COMPLETED = 7           

class ParkingGreenStates(Enum):
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
	return None

# Advanced Edge Line-Fitting from parking2.py
def get_front_edge_angle(track_polygon, y_min=62, y_max=95):
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

state = States.INIT
OC2_state = OC2_States.INIT
try:
	print("IDLE")
	while True:
		if reset_OC:
			if run_OC1:
				angle = 0
				turn_time = 0
				run_time = 0
				num_of_turn = 0
				is_clockwise = True
				turning_point = right_turning_point
				state = States.FIRST_SECTOR
			if run_OC2:
				angle = 0
				speed = 0
				speed_var = 0
				is_clockwise = True
				lot_count = 0
				lot_count_flag = 1
				pillar_count = 0
				pillar_count_flag = 1
				pillar_count_temp = 0
				enter_flag = 0
				last_seen_color = None
				OC2_state = OC2_States.LEAVE
				print("leave") 
			reset_OC = False
			print("Resetted")
			continue
			
		elif run_OC1:
			esp.send_command("OC1")
			start_time = time.perf_counter()
			while run_OC1:
				end_time = time.perf_counter()
				reply = esp_replyNprint()
				if reply == "EOC1":
					print("EOC1")
					run_OC1 = False
					break
				_, _, track = get_latest_data()
	
				# OC1 FSM
				if state == States.FIRST_SECTOR:
					angle = (track["center_x"]-320)*abs(track["center_x"]-320)/80
					esp.send_command("12, " + str(angle) + ", -1, move forward")
					time.sleep(0.025)
					if get_track_distance(turn_indi_1[0], turn_indi_1[1])[0] == False and get_track_distance(turn_indi_2[0], turn_indi_2[1])[0] == True and track["center_x"] != 0:
						if turn_flag:
							turn_flag = 0
							if track["center_x"] < 320:
								is_clockwise = False
							else:
								is_clockwise = True
							print(is_clockwise)
							state = States.TURNING_STATE
							num_of_turn += 1
							print(num_of_turn)
							print("turning")
						else:
							turn_flag = 1
					else:
						turn_flag = 0
					continue
				
				elif state == States.WAIT_TURN_STATE:
					esp.send_command("8, 0, -1, wait turn")
					if get_track_distance(turning_point[0], turning_point[1])[0] == True:
						state = States.TURNING_STATE
						num_of_turn += 1
						print(num_of_turn)
						print("turning")
						time.sleep(0.25)
					continue
				
				elif state == States.TURNING_STATE:
					angle = (track["center_x"]-320)/0.4
					if angle > 100:
						angle = 100
					if angle <-100:
						angle = -100
					esp.send_command("12, " + str(angle) +", -1, turn")
					if get_track_distance(sector_indi[is_clockwise][0], sector_indi[is_clockwise][1])[0] == True:
						if turn_time <= 0.1:
							num_of_turn -= 1
							print(num_of_turn)
							if num_of_turn > 0:
								state = States.RUN_SECTOR_STATE
								print("sector")
							else:
								state = States.FIRST_SECTOR
								print("first")
							turn_time = 0
							angle = 0
							continue 
						buffer = angle/5
						for i in range (4):
							angle -= buffer
							esp.send_command("12, " + str(angle) +", -1, turn")
							time.sleep(0.005)
						if num_of_turn == 12:
							state = States.LAST_RUN
							print("last")
						else:
							state = States.RUN_SECTOR_STATE
							print("sector")
						turn_time = 0
					time.sleep(0.025)
					turn_time += 0.025
					continue
				
				elif state == States.DASH_AFTER_TURNING_STATE:
					esp.send_command("8, 0, -1, go forward")
					time.sleep(0.25)
					if num_of_turn == 12:
						state = States.LAST_RUN
						print("last")
					else:
						state = States.RUN_SECTOR_STATE
						print("sector")
					continue
				
				elif state == States.RUN_SECTOR_STATE:                    
					if track["center_x"] != 0:
						angle_temp = (track["center_x"]-320)*abs(track["center_x"]-320)/300
						if angle_temp < 50 and angle_temp > -50:
							angle = angle_temp
					esp.send_command("12, " + str(angle) + ", -1, move forward")
					time.sleep(0.025)
					if get_track_distance(turn_indi_1[0], turn_indi_1[1])[0] == False and get_track_distance(turn_indi_2[0], turn_indi_2[1])[0] == True and track["center_x"] != 0:
						if turn_flag: 
							state = States.TURNING_STATE
							num_of_turn += 1
							print(num_of_turn)
							print("turning")
						else:
							turn_flag = 1
					else:
						turn_flag = 0
					continue
	
				elif state == States.LAST_RUN:
					time.sleep(0.5)
					esp.send_command("0, 0, 0,motor stop")
					break

				time.sleep(0.005)
				
		elif run_OC2:
			esp.send_command("OC2")
			while run_OC2:
				reply = esp_replyNprint()
				if reply == "EOC2":
					run_OC2 = False
					break
				time.sleep(0.03)
				pillar, lot, track = get_latest_data()

				if lot["center_x"] > 0:
					if lot_count_flag:
						lot_count += 1
						print("lot " + str(lot_count))
						lot_count_flag = 0
						pillar_count_temp = pillar_count
					if lot["center_y"] > 180:
						enter_flag = 1
				elif (pillar_count - 2) > pillar_count_temp:
					lot_count_flag = 1
					enter_flag = 0
				
				if enter_flag and lot_count == 4 and pillar_count % 3 == 0:
					OC2_state = OC2_States.ENTER

				if OC2_state == OC2_States.LEAVE:
					if track["center_x"] < 320:
						is_clockwise = False
					esp.send_command("0, " + str(is_clockwise * 200 - 100) + ", -1, turn")
					time.sleep(0.1)
					esp.send_command("10, " + str(is_clockwise * 200 - 100) + ", -1, turn")
					time.sleep(0.9)
					esp.send_command("-8.75, " + str(is_clockwise * -130 + 65) + ", -1, turn")
					time.sleep(0.6)
					esp.send_command("0, 0, -1, stop")
					time.sleep(0.4)
					pillar, lot, track = get_latest_data()
					if pillar["color"] != None and abs(pillar["center_x"] - 320) > 160:
						esp.send_command("8, 0, -1, move")
						print(pillar["color"] + str(pillar["center_x"]))
						time.sleep(1)
					OC2_state = OC2_States.WHITE
					print("white")
					continue

				if OC2_state == OC2_States.WHITE:
					if pillar["color"] != None:
						OC2_state = OC2_States.PILLAR
						print("pillar")
						continue
					angle = (track["center_x"] - 350 + is_clockwise * 60) / 0.3
					if angle > 97.5:
						angle = 97.5
					if angle < -97.5:
						angle = -97.5
					speed_var = min(100, abs(angle)) / 133
					speed = 8 + speed_var
					esp.send_command(str(speed) + ", " + str(angle) + ", -1, P")
					continue

				if OC2_state == OC2_States.PILLAR:
					if pillar["color"] == None:
						OC2_state = OC2_States.WHITE
						print("white")
						continue
					
					# Dynamically save the color state
					last_seen_color = pillar["color"]
					
					angle = (pillar["center_x"] * 1.75 + ((pillar["color"] == "RED") * 2 - 1) * pillar["center_y"] - 525 - (pillar["color"] == "GREEN") * 70) / 2
					if angle > 97.5:
						angle = 97.5
					if angle < -97.5:
						angle = -97.5
					if pillar["center_y"] < 180:
						angle /= ((180 / pillar["center_y"]) ** 1.5)
					print(angle)
					speed_var = min(100, abs(angle)) / 133
					speed = 8 + speed_var
					esp.send_command(str(speed) + ", " + str(angle) + ", -1, P")
					if pillar["center_y"] > 270:
						if pillar_count_flag:
							pillar_count += 1
							print("pillar " + str(pillar_count))
							pillar_count_flag = 0
					elif pillar["center_y"] < 180:
						pillar_count_flag = 1
					continue
				
				if OC2_state == OC2_States.ENTER:
					print(f"[PARKING ACTIVATED] Final decision triggered. Last seen pillar color: {last_seen_color}")
					esp.send_command("0, 0, 0, stop")
					time.sleep(0.5)

					# ==================================================================
					# CONDITIONAL EXECUTION FOR RED PILLAR (parking.py)
					# ==================================================================
					if last_seen_color == "RED":
						print("[PARKING BRANCH] Initiating Red Wall (parking.py) Sequence...")
						PURPLE_TARGET_X = 470
						PURPLE_ALIGN_SPEED = 7.5
						PURPLE_STOP_THRESHOLD = 130
						PROBE_Y = 129.5                  
						PROBE_LEFT_X = 260             
						PROBE_RIGHT_X = 300            
						STOP_DISTANCE_THRESHOLD = 1.0  
						KP_RED = 20
						KD_RED = 1.2
						last_alignment_error = 0.0
						
						park_red_state = ParkingRedStates.ALIGNING
						target_done = False
						run_target_sent = False
						maneuver_start_time = time.time()

						while True:
							reply = esp_replyNprint()
							if reply == "Done Target":
								target_done = True

							if park_red_state == ParkingRedStates.ALIGNING:
								_, parking, _ = get_latest_data()
								purple_x = parking.get("center_x", 0)
								purple_width = parking.get("width", 0)
								purple_height = parking.get("height", 0)
								purple_area = (purple_width * purple_height) // 100
								print(f"[TRACKING RED WALL 1] X: {purple_x} | Area: {purple_area}")

								if purple_x != 0 and purple_area > 10:
									if purple_area >= PURPLE_STOP_THRESHOLD:
										send_command_logged("0, 0, 0, wall 1 stop")
										park_red_state = ParkingRedStates.WALL_1_PAUSE
										maneuver_start_time = time.time()
									else:
										error = purple_x - PURPLE_TARGET_X
										kp_purple = 1.9
										align_steering = int(clamp(error * kp_purple, -65, 65))
										send_command_logged(f"{PURPLE_ALIGN_SPEED}, {align_steering}, 0, purple align")
								else:
									send_command_logged("0, 0, 0, tracking lost holding")

							elif park_red_state == ParkingRedStates.WALL_1_PAUSE:
								if time.time() - maneuver_start_time < 1.5:
									send_command_logged("0, 0, 0, holding pause")
								else:
									park_red_state = ParkingRedStates.BLACK_WALL_PD_APPROACH
									last_alignment_error = 0.0

							elif park_red_state == ParkingRedStates.BLACK_WALL_PD_APPROACH:
								has_poly_l, dist_left = get_track_distance(PROBE_LEFT_X, PROBE_Y)
								has_poly_r, dist_right = get_track_distance(PROBE_RIGHT_X, PROBE_Y)
								dist_left = dist_left + 1.7
								print(f"[PROBE L/R] Dist L: {dist_left:.2f} | Dist R: {dist_right:.2f}")

								if dist_left < STOP_DISTANCE_THRESHOLD or dist_right < STOP_DISTANCE_THRESHOLD:
									send_command_logged("0, 0, 0, R")
									time.sleep(1.0)
									park_red_state = ParkingRedStates.BACKWARD
									run_target_sent = False
									target_done = False
									continue

								alignment_error = dist_left - dist_right
								derivative = alignment_error - last_alignment_error
								pd_steering = int(clamp((KP_RED * alignment_error) + (KD_RED * derivative), -55, 55))
								last_alignment_error = alignment_error

								send_command_logged(f"6, {pd_steering}, 0, pd front wall alignment")

							elif park_red_state == ParkingRedStates.BACKWARD:
								if not run_target_sent:
									send_command_logged("-9, 0, 1, forward target")
									run_target_sent = True
									target_done = False
								if not target_done:
									time.sleep(0.001)
									continue
								else:
									send_command_logged("-1, 0, 0, R")
									park_red_state = ParkingRedStates.BACKWARD_TURN_1
									run_target_sent = False
									target_done = False
									continue

							elif park_red_state == ParkingRedStates.BACKWARD_TURN_1:
								if not run_target_sent:
									steer = -100 if is_clockwise else 100
									send_command_logged(f"-9, {steer}, 45, forward target")
									run_target_sent = True
									target_done = False
								if not target_done:
									time.sleep(0.001)
									continue
								else:
									send_command_logged("-1, 0, 0, R")
									park_red_state = ParkingRedStates.BACKWARD_STRAIGHT
									run_target_sent = False
									target_done = False
									continue

							elif park_red_state == ParkingRedStates.BACKWARD_STRAIGHT:
								if not run_target_sent:
									send_command_logged("-9, 0, 3, forward target")
									run_target_sent = True
									target_done = False
								if not target_done:
									time.sleep(0.001)
									continue
								else:
									send_command_logged("-1, 0, 0, R")
									park_red_state = ParkingRedStates.BACKWARD_TURN_2
									run_target_sent = False
									target_done = False
									continue

							elif park_red_state == ParkingRedStates.BACKWARD_TURN_2:
								if not run_target_sent:
									steer = 100 if is_clockwise else -100
									send_command_logged(f"-9, {steer}, 55, forward target")
									run_target_sent = True
									target_done = False
								if not target_done:
									time.sleep(0.001)
									continue
								else:
									send_command_logged("-1, 0, 0, R")
									park_red_state = ParkingRedStates.COMPLETED
									run_target_sent = False
									target_done = False
									continue

							elif park_red_state == ParkingRedStates.COMPLETED:
								send_command_logged("0, 0, 0, parking complete")
								break
							time.sleep(0.02)

					# ==================================================================
					# CONDITIONAL EXECUTION FOR GREEN PILLAR (parking2.py)
					# ==================================================================
					elif last_seen_color == "GREEN":
						print("[PARKING BRANCH] Initiating Green Wall (parking2.py) Sequence...")
						
						# Optimized Scaling Variables from your updated setup
						PROBE_Y = 72
						PROBE_LEFT_X = 220
						PROBE_RIGHT_X = 248
						TOUCH_POINT_X = 169
						TOUCH_POINT_Y = 86
						STOP_DISTANCE_THRESHOLD = 1.0
						APPROACH_SPEED = 5

						KP_GREEN = 30
						KD_GREEN = 1.2
						KP_ANGLE = 2.8
						KD_ANGLE = 0.9
						last_alignment_error = 0.0

						park_green_state = ParkingGreenStates.BLACK_WALL_PD_APPROACH
						target_done = False
						run_target_sent = False

						while True:
							reply = esp_replyNprint()
							if reply == "Done Target":
								target_done = True

							if park_green_state == ParkingGreenStates.BLACK_WALL_PD_APPROACH:
								_, _, track = get_latest_data()
								touch_inside, touch_dist = get_track_distance(TOUCH_POINT_X, TOUCH_POINT_Y)
								edge_angle = get_front_edge_angle(track.get("polygon"))

								print(f"[TOUCH POINT] Inside Poly: {int(touch_inside)} | Distance: {touch_dist:.2f}")

								# FIX: Ensure a real wall line profile is actively resolved before acting on thresholds
								if edge_angle is not None and 0.01 < touch_dist < STOP_DISTANCE_THRESHOLD:
									send_command_logged("0, 0, 0, R")
									park_green_state = ParkingGreenStates.COMPLETED
									run_target_sent = False
									target_done = False
									continue

								if edge_angle is not None:
									print(f"[FRONT EDGE ANGLE] {edge_angle:+.2f} deg")
									alignment_error = edge_angle
									derivative = alignment_error - last_alignment_error
									pd_steering = int(clamp(-(KP_ANGLE * alignment_error) - (KD_ANGLE * derivative) - 12, -55, 55))
									last_alignment_error = alignment_error
									send_command_logged(f"{APPROACH_SPEED}, {pd_steering}, 0, front edge line align")
								else:
									has_poly_l, dist_left = get_track_distance(PROBE_LEFT_X, PROBE_Y)
									has_poly_r, dist_right = get_track_distance(PROBE_RIGHT_X, PROBE_Y)
									
									if dist_left is None or dist_right is None or dist_left == 0 or dist_right == 0:
										last_alignment_error = 0.0
										send_command_logged(f"{APPROACH_SPEED}, 0, 0, fallback probe search")
									else:
										dist_left = dist_left + 0.6
										
										if dist_left < STOP_DISTANCE_THRESHOLD and dist_right < STOP_DISTANCE_THRESHOLD:
											send_command_logged("0, 0, 0, R")
											park_green_state = ParkingGreenStates.COMPLETED
											run_target_sent = False
											target_done = False
											continue

										alignment_error = dist_left - dist_right
										derivative = alignment_error - last_alignment_error
										pd_steering = int(clamp((KP_GREEN * alignment_error) + (KD_GREEN * derivative), -55, 55))
										last_alignment_error = alignment_error
										send_command_logged(f"{APPROACH_SPEED}, {pd_steering}, 0, fallback probe align")

							elif park_green_state == ParkingGreenStates.COMPLETED:
								park_green_state = ParkingGreenStates.RIGHT_TURN_90
								run_target_sent = False
								target_done = False
								continue

							elif park_green_state == ParkingGreenStates.RIGHT_TURN_90:
								if not run_target_sent:
									send_command_logged("-8.5, 100, 61, forward target")
									run_target_sent = True
									target_done = False
								if not target_done:
									time.sleep(0.001)
									continue
								else:
									send_command_logged("0, 0, 0, R") 
									park_green_state = ParkingGreenStates.STEERING_REALIGN
									run_target_sent = False
									target_done = False
									continue

							elif park_green_state == ParkingGreenStates.STEERING_REALIGN:
								send_command_logged("0, -30, 0, friction break pulse")
								time.sleep(0.15)  
								send_command_logged("0, 0, 0, R") 
								park_green_state = ParkingGreenStates.BACKWARD_ENCODER_MOVE
								continue

							elif park_green_state == ParkingGreenStates.BACKWARD_ENCODER_MOVE:
								if not run_target_sent:
									send_command_logged("-8.5, 0, 72, forward target")
									run_target_sent = True
									target_done = False
								if not target_done:
									time.sleep(0.001)
									continue
								else:
									send_command_logged("0, 0, 0, R")
									park_green_state = ParkingGreenStates.FINAL_TURN_90
									run_target_sent = False
									target_done = False
									continue

							elif park_green_state == ParkingGreenStates.FINAL_TURN_90:
								if not run_target_sent:
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
									park_green_state = ParkingGreenStates.REAR_SAFETY_BACKOFF
									run_target_sent = False
									target_done = False
									continue

							elif park_green_state == ParkingGreenStates.REAR_SAFETY_BACKOFF:
								if not run_target_sent:
									send_command_logged("-8.5, -30, 16, forward target")
									run_target_sent = True
									target_done = False
								if not target_done:
									time.sleep(0.001)
									continue
								else:
									send_command_logged("0, 0, 0, R")
									park_green_state = ParkingGreenStates.FINAL_CORRECTION_TURN
									run_target_sent = False
									target_done = False
									continue

							elif park_green_state == ParkingGreenStates.FINAL_CORRECTION_TURN:
								if not run_target_sent:
									send_command_logged("-8.5, -100, 47, forward target")
									run_target_sent = True
									target_done = False
								if not target_done:
									time.sleep(0.001)
									continue
								else:
									print("[SHAKE RESET] Releasing servo linkage compression...")
									send_command_logged("0, 60, 0, dynamic shake right")
									time.sleep(0.18)
									send_command_logged("0, 0, 0, straight hold")
									time.sleep(0.10)
									send_command_logged("0, 0, 0, R")
									break
							time.sleep(0.02)
					
					else:
						print("[⚠️ ERROR] No valid color recorded before entry sector!")

					run_OC2 = 0
					reset_OC = True
					break

				time.sleep(0.005)
				
		else:
			recorded_time = end_time - start_time
			reply = esp_replyNprint()
			if reply == "ROC1":
				reset_OC = True
				run_OC1 = True
				continue
			elif reply == "ROC2":
				reset_OC = True
				run_OC2 = True
				continue
		time.sleep(0.05)

except KeyboardInterrupt:
	print("\nShutting down software control loop...")
	esp.disconnect()
	stop_vision_system()
	time.sleep(0.5)
