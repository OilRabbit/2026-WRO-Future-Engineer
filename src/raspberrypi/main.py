import time
import datetime
from esp_com.communication import ESP32Communicator
from camera.camera_utils import start_vision_system, start_web_server, get_latest_data, stop_vision_system
from picamera2 import Picamera2 as picam2

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
state

#Checkpoints (default as clockwise case)
# TODO: make them into list or tuple or two separate variables
left_turning_point = 0, 180 #check direction
right_turning_point = 640, 180 #check direction
turning_point = right_turning_point #check if the robot get to the position that should turn
track_left = 600, 360 #check if the robot is getting left from the ideal track
track_right = 640, 360 #check if the robot is getting right from the ideal track
ending_point = 320, 100 #check if the robot is at the ideal point to end

#States
# TODO: make them into enums
FIRST_SECTOR = "Move forward until it knows the direction to run"
WAIT_TURN_STATE = "Move forward for fixed distance to get to the ideal point to turn"
TURNING_STATE = "Turn until it is parallel to the next path"
DASH_AFTER_TURING_STATE = "Move forward until it passed the corner sector"
RUN_SECTOR_STATE = "Move forward until is time to turn"
LAST_RUN = "Move forward to stop at the right place"

# Function for receiving msg from ESP and print the message with timestamp
def esp_replyNprint():
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    if reply is not None:
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        print(f"{timestamp} ESP: {reply}")
    return reply

try:
    print("IDLE")
    while True:
        if reset_OC:
            # Add anything you need to reset before each OC run below this line #

            # End of reset #
            reset_OC = False
            print("Resetted")
            print("IDLE")
	    state = FIRST_SECTOR
            continue
            
    	# TODO: add timer
        elif run_OC1:
            esp.send_command("OC1")
            while run_OC1:
                reply = esp_replyNprint()
                if reply == "EOC1":
                    run_OC1 = False
                    break
                _, _, track = get_latest_data()
                """
                Example of checking if a given point lies inside the track, and return the distance to the nearest edge:
                is_inside, distance = get_track_distance(320, 180)
                if is_inside:
                    # Do sth
                """
                # OC1 FSM #
                """
                for example:
                ```
                if state == DETECT_STATE:
                    # Do sth
                elif state == WHATEVER:
                    # Do sth
                ...etc
                ```
                """
                	# TODO: distance set to -1 if not need specific degree run
		        #Format eg: esp.send_command("<what to do>, <speed>, <streering percentage>, <distance(0 when not needed)>")
		        #The last straight road
		        if state == First_SECTOR:
		            esp.send_command("go forward, 10, 0, -1")
			    while get_track_distance(left_turning_point)[0] == False && get_track_distance(right_turning_point)[0] == False:
			        time.sleep(0.005)
			    if get_track_distance(left_turning_point)[0] == True:
				is_clockwise = False
				turning_point = left_turning_point
				# TODO: change the following two variables type
				track_left = 0, 360
				track_right = 40, 360
			    state = WAIT_TURN_STATE
			    continue
		        #Wait turn
		        elif state == WAIT_TURN_STATE:
		            esp.send_command("wait turn, 10, 70, 100")
			    state = TURNING_STATE
			    continue
		        #Turn 
		        elif state == TURNING_STATE:
		        	num_of_turn++
		        	esp.send_command("turn, 10, 70, 200")
		        	state = DASH_AFTER_TURNING_STATE
				continue
			#Dash to pass the corner
			elif state == DASH_AFTER_TURING_STATE:
				esp.send_command("go forward, 10, 10, 100")
				if num_of_turn == 12:
					state = LAST_RUN
				else:
					state = RUN_SECTOR_STATE
				continue
		        #Run sector and keep a certain distance from the inner barrier
                	elif state == RUN_SECTOR_STATE:
			    if (get_track_distance(track_left)[0] == False && is_clockwise) || (get_track_distance(track_left)[0] == True && not is_clockwise):
                    	    	esp.send_command("move right, 10, 10, 0")
			    elif (get_track_distance(track_right)[0] == True && is_clockwise) || (get_track_distance(track_right)[0] == False && not is_clockwise):
                                esp.send_command("move left, 10, 10, 0")
			    if get_track_distance(turning_point)[0] == True:
				state = WAIT_TURN_STATE
		        #Last forward to stop
                	elif state == LAST_RUN:
                	    esp.send_command("move forward, 10, 0, 0")
			    #Wait until the ending_point reach the wall in front of the robot
			    while get_track_distance(ending_point)[0] == True:
				time.sleep(0.005)
			    esp.send_command("motor stop, 0, 0, 0")

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
