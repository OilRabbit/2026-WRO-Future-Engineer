import serial
import time

# Change this to match what you found in 'ls /dev/ttyACM*'
# Usually it is /dev/ttyACM0
usb_port = '/dev/ttyACM1'

esp_serial = serial.Serial(usb_port, 115200, timeout = 1)

esp_connected = False
while not esp_connected:
    try:
        time.sleep(0.5)
        esp_serial.reset_input_buffer()
        print("Connected to ESP32 via USB at {}".format(usb_port))
        esp_connected = True
    except:
        print("Could not find ESP32. Is it connected to the Pi?")
        time.sleep(0.5)

run_OC1 = False
run_OC2 = False
reset_OC = True
cmd = ""
try:
    print("IDLE")
    while True:
        if reset_OC:
            reset_OC = False
            pass
        elif run_OC1:
            esp_serial.write(("hi1\n").encode('utf-8'))
            time.sleep(0.2)
            if esp_serial.in_waiting > 0:
                reply = esp_serial.readline().decode('utf-8', errors = 'ignore').strip()
                print("ESP: {}".format(reply))
                if reply == "EOC1":
                    run_OC1 = False
                    reset_OC = True
                    print("IDLE")
        elif run_OC2:
            esp_serial.write(("hi2\n").encode('utf-8'))
            time.sleep(0.2)
            if esp_serial.in_waiting > 0:
                reply = esp_serial.readline().decode('utf-8', errors = 'ignore').strip()
                print("ESP: {}".format(reply))
                if reply == "EOC2":
                    run_OC2 = False
                    reset_OC = True
                    print("IDLE")
        else:
            if esp_serial.in_waiting > 0:
                reply = esp_serial.readline().decode('utf-8', errors = 'ignore').strip()
                print("ESP: {}".format(reply))
                if reply == "ROC1":
                    run_OC1 = True
                elif reply == "ROC2":
                    run_OC2 = True
                else:
                    pass

except KeyboardInterrupt:
    esp_serial.close()
