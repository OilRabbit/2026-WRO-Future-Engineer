import time
import datetime
import math
from esp_com.communication import ESP32Communicator
from camera.camera_utils import start_vision_system, start_web_server, get_latest_data, stop_vision_system, get_track_distance, configure_vision_pipeline
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

#Checkpoints (default as clockwise case)
angle = 0 #steering percentage
speed = 0
speed_var = 0
turn_flag = 0
turn_indi_1 = [200, 87.5] #check when to turn
turn_indi_2 = [200, 106.25] #check when to turn
sector_indi = [[150, 87.5], [250, 87.5]] #check when is sector
turn_time = 0
run_time = 0
lot_count = 0
lot_count_flag = 1
pillar_count = 0
pillar_count_flag = 1
pillar_count_temp = 0
enter_flag = 0
last_pillar_color = None
run_target_sent = False
target_done = False

front_turning_point = [200, 43.75]
left_turning_point = [25, 125] #check direction
right_turning_point = [375, 125] #check direction
turning_point = right_turning_point #check if the robot get to the position that should turn
track_left = [[50, 112.5], [325, 112.5]] #check if the robot is getting left from the ideal track
track_right = [[75, 112.5], [350, 112.5]] #check if the robot is getting right from the ideal track
ending_point = [200, 18.75] #check if the robot is at the ideal point to end

#States
class States(Enum):
	INIT = 0
	FIRST_SECTOR = 1 #Move forward until it knows the direction to run
	WAIT_TURN_STATE = 2 #Move forward for fixed distance to get to the ideal point to turn
	TURNING_STATE = 3 #Turn until it is parallel to the next path
	DASH_AFTER_TURNING_STATE = 4 #Move forward until it passed the corner sector
	RUN_SECTOR_STATE = 5 #Move forward until is time to turn
	LAST_RUN = 6 #Move forward to stop at the right place

class OC2_States(Enum):
	INIT = 0
	LEAVE = 1
	WHITE = 2
	PILLAR = 3
	ALIGN_0 = 4
	ENTER = 5

# Function for receiving msg from ESP and print the message with timestamp
def esp_replyNprint():
	timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
	reply = esp.read_message()
	if reply is not None:
		timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
		print(f"{timestamp} ESP: {reply}")
	return reply

def send_command_logged(cmd):
	try:
		esp.send_command(cmd)
	except Exception as e:
		print(f"[SERIAL WRITE FAULT] Failed to transmit packet: {e}")

def clamp(value, minimum, maximum):
	return max(minimum, min(maximum, value))

def wait_for_target_done(stop_reply="EOC2"):
	while True:
		reply = esp_replyNprint()
		if reply == stop_reply:
			return False
		if reply == "Done Target":
			return True
		time.sleep(0.001)

def run_parking_red():
	print("[PARKING] Using parking.py routine (last block RED)")
	PURPLE_TARGET_X = 293.75
	PURPLE_ALIGN_SPEED = 7.5
	PURPLE_STOP_THRESHOLD = 130
	PROBE_Y = 80.9375
	PROBE_LEFT_X = 162.5
	PROBE_RIGHT_X = 187.5
	STOP_DISTANCE_THRESHOLD = 0.625
	KP = 20
	KD = 1.2
	last_alignment_error = 0.0
	state = "ALIGNING"
	pause_start = 0.0

	while True:
		reply = esp_replyNprint()
		if reply == "EOC2":
			return False

		if state == "ALIGNING":
			_, parking, _ = get_latest_data()
			purple_x = parking.get("center_x", 0)
			purple_width = parking.get("width", 0)
			purple_height = parking.get("height", 0)
			purple_area = (purple_width * purple_height) // 100

			if purple_x != 0 and purple_area > 10:
				if purple_area >= PURPLE_STOP_THRESHOLD:
					send_command_logged("0, 0, 0, wall 1 stop")
					state = "WALL_1_PAUSE"
					pause_start = time.time()
				else:
					error = purple_x - PURPLE_TARGET_X
					align_steering = int(clamp(error * 1.9, -65, 65))
					send_command_logged(f"{PURPLE_ALIGN_SPEED}, {align_steering}, 0, purple align")
			else:
				send_command_logged("0, 0, 0, tracking lost holding")

		elif state == "WALL_1_PAUSE":
			if time.time() - pause_start < 1.5:
				send_command_logged("0, 0, 0, holding pause")
			else:
				state = "BLACK_WALL_PD_APPROACH"
				last_alignment_error = 0.0

		elif state == "BLACK_WALL_PD_APPROACH":
			_, dist_left = get_track_distance(PROBE_LEFT_X, PROBE_Y)
			_, dist_right = get_track_distance(PROBE_RIGHT_X, PROBE_Y)
			dist_left += 1.0625

			if dist_left < STOP_DISTANCE_THRESHOLD or dist_right < STOP_DISTANCE_THRESHOLD:
				send_command_logged("0, 0, 0, R")
				time.sleep(1.0)
				state = "BACKWARD"
				continue

			alignment_error = dist_left - dist_right
			derivative = alignment_error - last_alignment_error
			pd_steering = int(clamp((KP * alignment_error) + (KD * derivative), -55, 55))
			last_alignment_error = alignment_error
			send_command_logged(f"6, {pd_steering}, 0, pd front wall alignment")

		elif state == "BACKWARD":
			send_command_logged("-9, 0, 1, forward target")
			if not wait_for_target_done():
				return False
			send_command_logged("-1, 0, 0, R")
			state = "BACKWARD_TURN_1"

		elif state == "BACKWARD_TURN_1":
			steer = -100 if is_clockwise else 100
			send_command_logged(f"-9, {steer}, 45, forward target")
			if not wait_for_target_done():
				return False
			send_command_logged("-1, 0, 0, R")
			state = "BACKWARD_STRAIGHT"

		elif state == "BACKWARD_STRAIGHT":
			send_command_logged("-9, 0, 3, forward target")
			if not wait_for_target_done():
				return False
			send_command_logged("-1, 0, 0, R")
			state = "BACKWARD_TURN_2"

		elif state == "BACKWARD_TURN_2":
			steer = 100 if is_clockwise else -100
			send_command_logged(f"-9, {steer}, 55, forward target")
			if not wait_for_target_done():
				return False
			send_command_logged("-1, 0, 0, R")
			state = "COMPLETED"

		elif state == "COMPLETED":
			send_command_logged("0, 0, 0, parking complete")
			return True

		time.sleep(0.02)

def run_parking_other():
	print("[PARKING] Using parking2.py routine (last block not RED)")
	PROBE_Y_FORWARD = 83.75
	PROBE_LEFT_X_FORWARD = 200
	PROBE_RIGHT_X_FORWARD = 225
	TARGET_DIST = 0.625
	MIN_STOP_DIST = 0.5625
	MAX_STOP_DIST = 0.6875
	KP_SPEED = 10.0
	KP = 30
	KD = 1.2
	last_alignment_error = 0.0
	last_direction = None
	oscillation_count = 0
	state = "BLACK_WALL_PD_APPROACH"

	while True:
		reply = esp_replyNprint()
		if reply == "EOC2":
			return False

		if state == "BLACK_WALL_PD_APPROACH":
			_, dist_left = get_track_distance(PROBE_LEFT_X_FORWARD, PROBE_Y_FORWARD)
			_, dist_right = get_track_distance(PROBE_RIGHT_X_FORWARD, PROBE_Y_FORWARD)
			dist_left += 0.3125

			if (MIN_STOP_DIST <= dist_left <= MAX_STOP_DIST) and (MIN_STOP_DIST <= dist_right <= MAX_STOP_DIST):
				send_command_logged("0, 0, 0, R")
				state = "COMPLETED"
				continue

			mean_distance = (dist_left + dist_right) / 2.0
			dist_error = mean_distance - TARGET_DIST
			current_direction = 1 if dist_error >= 0 else -1
			if last_direction is not None and current_direction != last_direction:
				oscillation_count += 1
				if oscillation_count >= 2:
					send_command_logged("0, 0, 0, R")
					state = "COMPLETED"
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

		elif state == "COMPLETED":
			state = "RIGHT_TURN_90"

		elif state == "RIGHT_TURN_90":
			send_command_logged("-8.5, 100, 61, forward target")
			if not wait_for_target_done():
				return False
			send_command_logged("0, 0, 0, R")
			state = "STEERING_REALIGN"

		elif state == "STEERING_REALIGN":
			send_command_logged("0, -30, 0, friction break pulse")
			time.sleep(0.15)
			send_command_logged("0, 0, 0, R")
			state = "BACKWARD_ENCODER_MOVE"

		elif state == "BACKWARD_ENCODER_MOVE":
			send_command_logged("-8.5, 0, 72, forward target")
			if not wait_for_target_done():
				return False
			send_command_logged("0, 0, 0, R")
			state = "FINAL_TURN_90"

		elif state == "FINAL_TURN_90":
			send_command_logged("8.5, 100, 45, forward target")
			if not wait_for_target_done():
				return False
			send_command_logged("0, -30, 0, friction break pulse")
			time.sleep(0.15)
			send_command_logged("0, 0, 0, R")
			state = "REAR_SAFETY_BACKOFF"

		elif state == "REAR_SAFETY_BACKOFF":
			send_command_logged("-8.5, -30, 16, forward target")
			if not wait_for_target_done():
				return False
			send_command_logged("0, 0, 0, R")
			state = "FINAL_CORRECTION_TURN"

		elif state == "FINAL_CORRECTION_TURN":
			send_command_logged("-8.5, -100, 47, forward target")
			if not wait_for_target_done():
				return False
			send_command_logged("0, 60, 0, dynamic shake right")
			time.sleep(0.18)
			send_command_logged("0, 0, 0, straight hold")
			time.sleep(0.10)
			send_command_logged("0, 0, 0, R")
			return True

		time.sleep(0.02)

state = States.INIT
OC2_state = OC2_States.INIT
try:
	print("IDLE")
	while True:
		if reset_OC:
			# Add anything you need to reset before each OC run below this line #
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
				enter_flag = 0
				last_pillar_color = None
				run_target_sent = False
				target_done = False
				OC2_state = OC2_States.LEAVE
				print("leave") 
			# End of reset #
			reset_OC = False
			print("Resetted")
			# print("IDLE")
			continue
			
		# TODO: add timer
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
				#Format eg: esp.send_command("<speed>, <streering percentage>, <distance(-1 when not needed)>, <what to do>")
				#The first sector
				if state == States.FIRST_SECTOR:
					angle = (track["center_x"]-200)*abs(track["center_x"]-200)/50
					esp.send_command("12, " + str(angle) + ", -1, move forward")
					time.sleep(0.025)
					if get_track_distance(turn_indi_1[0], turn_indi_1[1])[0] == False and get_track_distance(turn_indi_2[0], turn_indi_2[1])[0] == True and track["center_x"] != 0:
						if turn_flag:
							turn_flag = 0
							if track["center_x"] < 200:
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

					esp.send_command("8, 0, -1, move forward")
					if get_track_distance(turn_indi_1[0], turn_indi_1[1])[0] == False:
						if track["center_x"] < 200 :
							is_clockwise = False
							turning_point = left_turning_point
						state = States.WAIT_TURN_STATE
						print(is_clockwise)
						print("wait")
					continue
				
				#Wait turn
				elif state == States.WAIT_TURN_STATE:
					esp.send_command("8, 0, -1, wait turn")
					if get_track_distance(turning_point[0], turning_point[1])[0] == True:
						state = States.TURNING_STATE
						num_of_turn += 1
						print(num_of_turn)
						print("turning")
						time.sleep(0.25)
					continue
				
				#Turn 
				elif state == States.TURNING_STATE:
					angle = (track["center_x"]-200)/0.25
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

					esp.send_command("8, " + str(is_clockwise*160-80) + ", -1, turn")
					time.sleep(2)
					state = States.DASH_AFTER_TURNING_STATE
					print("after")
					continue
				
				#Dash to pass the corner
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
				
				#Run sector and keep a certain distance from the inner barrier
				elif state == States.RUN_SECTOR_STATE:                    
					if track["center_x"] != 0:
						angle_temp = (track["center_x"]-200)*abs(track["center_x"]-200)/187.5
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

					if get_track_distance(track_right[is_clockwise][0], track_right[is_clockwise][1])[0] == is_clockwise:
						esp.send_command("8, 25, -1, move right")
						print("right")
					elif get_track_distance(track_left[is_clockwise][0], track_left[is_clockwise][1])[0] != is_clockwise:
						esp.send_command("8, -25, -1, move left")
						print("left")
					else:
						esp.send_command("8, 0, -1, go forward")
					if get_track_distance(turning_point[0], turning_point[1])[0] == True and get_track_distance(front_turning_point[0], front_turning_point[1])[0] == False:
						state = States.WAIT_TURN_STATE
						print("wait")
					continue
	
				#Last forward to stop
				elif state == States.LAST_RUN:
					time.sleep(0.5)
					esp.send_command("0, 0, 0,motor stop")
					break
					if track["center_x"] != 0:
						angle = (track["center_x"]-200)*abs(track["center_x"]-200)/37.5
					esp.send_command("10, " + str(angle) + ", -1, move forward")
					time.sleep(0.05)
					#Wait until the ending_point reach the wall in front of the robot
					if get_track_distance(ending_point[0], ending_point[1])[0] == False:
						esp.send_command("0, 0, 0, motor stop")
						end_time = time.perf_counter()
						print(end_time)
						break

				# End of OC1 FSM #
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

				# OC2 FSM #
				#left = [240, 120] 
				#right = [400, 120]
				#while 1:
					#dl = get_track_distance(left[0], left[1])[1]
					#dr = get_track_distance(right[0], right[1])[1]
					#esp.send_command(str((dl + dr >= 0) * 16 - 8) + ", " + str(dl + dr) + ", -1, move")
				if lot["center_x"] > 0:
					if lot_count_flag:
						lot_count += 1
						print("lot " + str(lot_count))
						lot_count_flag = 0
						pillar_count_temp = pillar_count
					if lot["center_y"] > 112.5:
						enter_flag = 1
				elif (pillar_count - 2) > pillar_count_temp:
					lot_count_flag = 1
					enter_flag = 0
				
				if lot_count == 4:
					if pillar_count % 3 == 0:
						OC2_state = OC2_States.ALIGN_0
						print("align_0")
					#elif pillar_count % 3 == 2:
						#OC2_state = OC2_States.ALIGN_0

				if OC2_state == OC2_States.LEAVE:
					if track["center_x"] < 200:
						is_clockwise = False
					esp.send_command("0, " + str(is_clockwise * 200 - 100) + ", -1, P")
					time.sleep(0.1)
					esp.send_command("6, " + str(is_clockwise * 200 - 100) + ", 55, P")
					while reply != "Done Target":
						time.sleep(0.01)
						reply = esp_replyNprint()
					reply = esp_replyNprint()
					esp.send_command("-1, 0, 0, R")
					esp.send_command("-6, " + str(is_clockwise * -80 + 40) + ", 20, P")
					while reply != "Done Target":
						time.sleep(0.01)
						reply = esp_replyNprint()
					reply = esp_replyNprint()
					esp.send_command("-1, 0, 0, R")
					esp.send_command("0, 0, -1, stop")
					time.sleep(0.4)
					pillar, lot, track = get_latest_data()
					if pillar["color"] != None and abs(pillar["center_x"] - 200) > 100:
						esp.send_command("6, 0, -1, move")
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
					angle = (track["center_x"] - 208 + is_clockwise * 16) / 0.1875
					if angle > 100:
						angle = 100
					if angle < -100:
						angle = -100
					speed_var = min(100, abs(angle)) / 100
					speed = 6 + speed_var
					esp.send_command(str(speed) + ", " + str(angle) + ", -1, P")
					continue

				if OC2_state == OC2_States.PILLAR:
					if pillar["color"] == None:
						OC2_state = OC2_States.WHITE
						print("white")
						continue
					angle = (pillar["center_x"] * 1.75 + ((pillar["color"] == "RED") * 2 - 1) * pillar["center_y"] - 328.125 - (pillar["color"] == "GREEN") * 43.75) / 1.25
					#if not get_track_distance(62.5 + is_clockwise * 275, 168.75)[0] and angle * (is_clockwise * 2 - 1) > 0:
						#angle = 20 - is_clockwise * 40
					if angle > 100:
						angle = 100
					if angle < -100:
						angle = -100
					if pillar["center_y"] < 112.5:
						angle /= ((112.5 / pillar["center_y"]) ** 1.6)
					print(angle)
					speed_var = min(100, abs(angle)) / 100
					speed = 6 + speed_var
					esp.send_command(str(speed) + ", " + str(angle) + ", -1, P")
					if pillar["center_y"] > 168.75:
						if pillar_count_flag:
							pillar_count += 1
							print("pillar " + str(pillar_count))
							pillar_count_flag = 0
							if pillar_count % 3 == 0:
								last_pillar_color = pillar["color"]
					elif pillar["center_y"] < 112.5:
						pillar_count_flag = 1
					continue

				if OC2_state == OC2_States.ALIGN_0:
					if lot["center_x"] > 0:
						angle = (lot["center_x"] - lot["center_y"] - 195) / 1.25
					else:
						angle = (track["center_x"] - 200) / 0.1875
					if angle > 95:
						angle = 95
					if angle < -95:
						angle = -95
					if lot["center_x"] > 0 and lot["center_y"] < 100:
						angle /= ((100 / lot["center_y"]) ** 4)
					print(angle)
					speed_var = min(95, abs(angle)) / 95
					speed = 5.5 + speed_var
					esp.send_command(str(speed) + ", " + str(angle) + ", -1, P")
					if lot["center_y"] > 105:
						OC2_state = OC2_States.ENTER
						print("enter")
						esp.send_command("0, 0, 0, stop")
						break
					continue

				if OC2_state == OC2_States.ENTER:
					print("last pillar color before stop:", last_pillar_color)
					send_command_logged("0, 0, 0, stop before parking")
					if last_pillar_color == "RED":
						parking_completed = run_parking_red()
					else:
						parking_completed = run_parking_other()
					if parking_completed:
						print("[PARKING] Completed successfully")
					else:
						print("[PARKING] Aborted by EOC2")
					run_OC2 = 0
					reset_OC = True
					break

				# End of OC2 FSM #
				time.sleep(0.005)
				
		else:
			# IDLE
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
	print("\nShutting down...")
	esp.disconnect()
	stop_vision_system()
	time.sleep(0.5)
