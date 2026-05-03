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
            continue
            
        elif run_OC1:
            esp.send_command("OC1")
            while run_OC1:
                reply = esp_replyNprint()
                if reply == "EOC1":
                    run_OC1 = False
                    break
                _, _, track = get_latest_data()
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
		if num_of_turn == 12 && '''the 12nd turn is ended and going along the wall''':
		    esp.send_command("last forward, 10, 0")
		elif '''suitable to turn left''':
		    esp.send_command("turn left, 10, 70")
		elif '''suitable to turn right''':
                    esp.send_command("turn right, 10, 70")
		elif '''gettig right from the ideal track''':
                    esp.send_command("move left, 10, 10")
		elif '''gettig right from the ideal track''':
                    esp.send_command("move right, 10, 10")
		else
		    esp.send_command("forward, 10, 0")
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
