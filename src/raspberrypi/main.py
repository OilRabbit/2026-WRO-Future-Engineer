# testing
import time
import datetime
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

start_vision_system(camera, False)

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
left_turning_point = [0, 180] #check direction
right_turning_point = [640, 180] #check direction
turning_point = right_turning_point #check if the robot get to the position that should turn
track_left = [600, 360] #check if the robot is getting left from the ideal track
track_right = [640, 360] #check if the robot is getting right from the ideal track
ending_point = [320, 100] #check if the robot is at the ideal point to end

#States
class States(Enum):
	INIT = 0
	FIRST_SECTOR = 1 #Move forward until it knows the direction to run
	WAIT_TURN_STATE = 2 #Move forward for fixed distance to get to the ideal point to turn
	TURNING_STATE = 3 #Turn until it is parallel to the next path
	DASH_AFTER_TURING_STATE = 4 #Move forward until it passed the corner sector
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
				state = States.FIRST_SECTOR
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
					print("FIRST_SECTOR")
					esp.send_command("10, 0, -1, go forward")
					if get_track_distance(left_turning_point[0], left_turning_point[1])[0] == False and get_track_distance(right_turning_point[0], right_turning_point[1])[0] == False:
						time.sleep(0.005)
						break
					if get_track_distance(left_turning_point[0], left_turning_point[1])[0] == True:
						is_clockwise = False
						turning_point = left_turning_point
						track_left = [0, 360]
						track_right = [40, 360]
						state = States.WAIT_TURN_STATE
					continue
				
				#Wait turn
				elif state == States.WAIT_TURN_STATE:
					esp.send_command("10, 0, 100, wait turn")
					state = States.TURNING_STATE
					continue
				
				#Turn 
				elif state == States.TURNING_STATE:
					num_of_turn += 1
					esp.send_command("10, 70, 200, turn")
					state = States.DASH_AFTER_TURNING_STATE
					continue
				
				#Dash to pass the corner
				elif state == States.DASH_AFTER_TURING_STATE:
					esp.send_command("10, 0, 100, go forward")
					if num_of_turn == 12:
						state = States.LAST_RUN
					else:
						state = States.RUN_SECTOR_STATE
					continue
				
				#Run sector and keep a certain distance from the inner barrier
				elif state == States.RUN_SECTOR_STATE:
					if (get_track_distance(track_right[0], track_right[1])[0] == True and is_clockwise) or (get_track_distance(track_left[0], track_left[1])[0] == True and not is_clockwise):
						esp.send_command("10, 10, -1, move right")
					elif (get_track_distance(track_left[0], track_left[1])[0] == False and is_clockwise) or (get_track_distance(track_right[0], track_right[1])[0] == False and not is_clockwise):
						esp.send_command("10, 10, -1, move left")
					if get_track_distance(turning_point[0], turning_point[1])[0] == True:
						state = States.WAIT_TURN_STATE
					continue
	
				#Last forward to stop
				elif state == States.LAST_RUN:
					esp.send_command("10, 0, -1, move forward")
					#Wait until the ending_point reach the wall in front of the robot
					if get_track_distance(ending_point[0], ending_point[1])[0] == True:
						time.sleep(0.005)
						break
					esp.send_command("0, 0, 0, motor stop")
					end_time = time.pref_counter()
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
