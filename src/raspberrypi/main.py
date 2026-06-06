# testing
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
config = camera.create_preview_configuration(main={"size": (640, 360), "format": "BGR888"})
camera.configure(config)
camera.start()
print("Camera: Activated")

start_vision_system(camera, True)

live_streaming = True
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
turn_indi_1 = [320, 40] #check when to turn
turn_indi_2 = [320, 60] #check when to turn
sector_indi = [320, 60] #check when is sector
left_turning_point = [30, 180] #check direction
right_turning_point = [610, 180] #check direction
turning_point = right_turning_point #check if the robot get to the position that should turn
track_left = [[30, 180], [580, 180]] #check if the robot is getting left from the ideal track
track_right = [[60, 180], [610, 180]] #check if the robot is getting right from the ideal track
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

# Function for receiving msg from ESP and print the message with timestamp
def esp_replyNprint():
	timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
	reply = esp.read_message()
	if reply is not None:
		timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
		print(f"{timestamp} ESP: {reply}")
	return reply

state = States.INIT
try:
	print("IDLE")
	while True:
		if reset_OC:
			# Add anything you need to reset before each OC run below this line #
			if run_OC1:
				angle = 0
				num_of_turn = 0
				is_clockwise = True
				turning_point = right_turning_point
				state = States.FIRST_SECTOR
				print("FIRST_SECTOR") 
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
					#print("FIRST_SECTOR")
					#if track["center_x"] != 0:
						#angle_temp = (track["center_x"]-320)*abs(track["center_x"]-320)/60
						#if angle_temp < 10 and angle_temp > -10:
							#angle = angle_temp
					#esp.send_command("8, " + str(angle) + ", -1, move forward")
					#time.sleep(0.05)
					#if get_track_distance(turn_indi_1[0], turn_indi_1[1])[0] == False and get_track_distance(turn_indi_2[0], turn_indi_2[1])[0] == True and track["center_x"] != 0:
						#time.sleep(0.066)
						#if get_track_distance(turn_indi_1[0], turn_indi_1[1])[0] == False and get_track_distance(turn_indi_2[0], turn_indi_2[1])[0] == True and track["center_x"] != 0:
							#state = States.TURNING_STATE
							#num_of_turn += 1
							#print(num_of_turn)
							#print("turning state")

					#continue

					esp.send_command("8, 0, -1, move forward")
					if get_track_distance(turn_indi_1[0], turn_indi_1[1])[0] == False:
						if track["center_x"] < 320 :
							is_clockwise = False
							turning_point = left_turning_point
							print(is_clockwise)
						state = States.WAIT_TURN_STATE
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
					continue
				
				#Turn 
				elif state == States.TURNING_STATE:
					#angle = (track["center_x"]-320)/0.4
					#esp.send_command("8, " + str(angle) +", -1, turn")
					#print(angle)
					#if get_track_distance(sector_indi[0], sector_indi[1])[0] == True:
						#buffer = angle/5
						#for i in range (4):
							#angle -= buffer
							#esp.send_command("8, " + str(angle) +", -1, turn")
							#time.sleep(0.05)
						#if num_of_turn == 12:
							#state = States.LAST_RUN
							#print("last")
						#else:
							#state = States.RUN_SECTOR_STATE
							#print("sector")
					#time.sleep(0.05)
					#continue

					esp.send_command("8, " + str(is_clockwise*160-80) + ", 200, turn")
					state = States.DASH_AFTER_TURNING_STATE
					print("after")
					continue
				
				#Dash to pass the corner
				elif state == States.DASH_AFTER_TURNING_STATE:
					esp.send_command("8, 0, 100, go forward")
					if num_of_turn == 12:
						state = States.LAST_RUN
						print("last")
					else:
						state = States.RUN_SECTOR_STATE
						print("sector")
					continue
				
				#Run sector and keep a certain distance from the inner barrier
				elif state == States.RUN_SECTOR_STATE:                    
					#if track["center_x"] != 0:
						#angle_temp = (track["center_x"]-320)*abs(track["center_x"]-320)/60
						#if angle_temp < 10 and angle_temp > -10:
							#angle = angle_temp
					#esp.send_command("8, " + str(angle) + ", -1, move forward")
					#print(angle)
					#time.sleep(0.05)
					#if get_track_distance(turn_indi_1[0], turn_indi_1[1])[0] == False and get_track_distance(turn_indi_2[0], turn_indi_2[1])[0] == True and track["center_x"] != 0:
						#time.sleep(0.066)
						#if get_track_distance(turn_indi_1[0], turn_indi_1[1])[0] == False and get_track_distance(turn_indi_2[0], turn_indi_2[1])[0] == True and track["center_x"] != 0: 
							#state = States.TURNING_STATE
							#num_of_turn += 1
							#print(num_of_turn)
							#print("turning")
					#continue

					esp.send_command("8, 0, -1, go forward")
					if get_track_distance(track_right[is_clockwise][0], track_right[is_clockwise][1])[0] == is_clockwise:
						esp.send_command("8, 10, -1, move right")
					elif get_track_distance(track_left[is_clockwise][0], track_left[is_clockwise][1])[0] != is_clockwise:
						esp.send_command("8, -10, -1, move left")
					if get_track_distance(turning_point[0], turning_point[1])[0] == True and get_track_distance(turn_indi_1[0], turn_indi_1[1])[0] == False:
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
				
		elif run_OC2:
			esp.send_command("OC2")
			while run_OC2:
				reply = esp_replyNprint()
				if reply == "EOC2":
					run_OC2 = False
					break
				# OC2 FSM #
	
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
