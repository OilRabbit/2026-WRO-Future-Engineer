import time
import datetime
import math
from esp_com.communication import ESP32Communicator
from camera.camera_utils import start_vision_system, start_web_server, get_latest_data, stop_vision_system, get_track_distance
from picamera2 import Picamera2 as picam2
from enum import Enum

# ESP communication and Picam init
print("======= Init =======")

esp = ESP32Communicator()
esp.connect()
camera = picam2()
full_sensor_res = camera.sensor_resolution
config = camera.create_video_configuration(main={"size": (640, 360), "format": "BGR888"},sensor={"output_size": full_sensor_res})
camera.configure(config)
camera.start()
print("Camera: Activated")

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
turn_flag = 0
turn_indi_1 = [320, 140] #check when to turn
turn_indi_2 = [320, 170] #check when to turn
sector_indi = [[240, 140], [400, 140]] #check when is sector
turn_time = 0
run_time = 0
lot_count = 0
lot_count_flag = 1
pillar_count = 0
pillar_count_flag = 1
pillar_count_temp = 0

front_turning_point = [320, 70]
left_turning_point = [40, 200] #check direction
right_turning_point = [600, 200] #check direction
turning_point = right_turning_point #check if the robot get to the position that should turn
track_left = [[80, 180], [520, 180]] #check if the robot is getting left from the ideal track
track_right = [[120, 180], [560, 180]] #check if the robot is getting right from the ideal track
ending_point = [320, 30] #check if the robot is at the ideal point to end

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
	ENTER = 4

# Function for receiving msg from ESP and print the message with timestamp
def esp_replyNprint():
	timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
	reply = esp.read_message()
	if reply is not None:
		timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
		print(f"{timestamp} ESP: {reply}")
	return reply

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
				lot_count = 0
				lot_count_flag = 1
				pillar_count = 0
				pillar_count_flag = 1
				state = States.FIRST_SECTOR
				OC2_state = OC2_States.LEAVE
				print("leave")
			if run_OC2:
				angle = 0
			# End of reset #
			reset_OC = False
			print("Resetted")
			# print("IDLE")
			continue
			
		# TODO: add timer
		elif run_OC2:
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

					esp.send_command("8, 0, -1, move forward")
					if get_track_distance(turn_indi_1[0], turn_indi_1[1])[0] == False:
						if track["center_x"] < 320 :
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
						angle = (track["center_x"]-320)*abs(track["center_x"]-320)/60
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
				
		elif run_OC1:
			esp.send_command("OC2")
			while run_OC1:
				reply = esp_replyNprint()
				if reply == "EOC2":
					run_OC2 = False
					break
				pillar, lot, track = get_latest_data()
				time.sleep(0.025)

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
				elif (pillar_count - 2) > pillar_count_temp:
					lot_count_flag = 1
				
				if lot_count == 4 and pillar_count % 3 == 0:
					OC2_state = OC2_States.ENTER

				if OC2_state == OC2_States.LEAVE:
					if track["center_x"] < 320:
						is_clockwise = False
					esp.send_command("6, " + str(is_clockwise * 200 - 100) + ", -1, turn")
					time.sleep(1.5)
					esp.send_command("-6, " + str(is_clockwise * -200 + 100) + ", -1, turn")
					time.sleep(0.5)
					#esp.send_command("6, " + str(is_clockwise * 200 - 100) + ", -1, turn")
					#time.sleep(0.75)
					#esp.send_command("-6, " + str(is_clockwise * -200 + 100) + ", -1, turn")
					#time.sleep(0.75)
					#esp.send_command("6, " + str(is_clockwise * 200 - 100) + ", -1, turn")
					#time.sleep(0.75)
					#esp.send_command("-6, " + str(is_clockwise * -200 + 100) + ", -1, turn")
					#time.sleep(0.75)
					OC2_state = OC2_States.WHITE
					print("white")
					continue

				if OC2_state == OC2_States.WHITE:
					if pillar["color"] != None:
						OC2_state = OC2_States.PILLAR
						print("pillar")
						continue
					angle = (track["center_x"] - 340 + is_clockwise * 40) / 0.3
					esp.send_command("9, " + str(angle) + ", -1, move")
					continue

				if OC2_state == OC2_States.PILLAR:
					if pillar["color"] == None:
						OC2_state = OC2_States.WHITE
						print("white")
						continue
					angle = (pillar["center_x"] * 2 + ((pillar["color"] == "RED") * 2 - 1) * pillar["center_y"] - 660) / 1.8
					esp.send_command("9, " + str(angle) + ", -1, move")
					if pillar["center_y"] > 320:
						if pillar_count_flag:
							pillar_count += 1
							print("pillar " + str(pillar_count))
							pillar_count_flag = 0
					else:
						pillar_count_flag = 1
					continue
				
				if OC2_state == OC2_States.ENTER:
					esp.send_command("0, 0, 0, stop")
					run_OC1 = 0
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
