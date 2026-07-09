import serial
import time

# Change this to match what you found in 'ls /dev/ttyACM*'
# Usually it is /dev/ttyACM0
usb_port = '/dev/ttyACM1' 

try:
    ser = serial.Serial(usb_port, 115200, timeout=1)
    time.sleep(2) # Wait for ESP32 to boot
    ser.reset_input_buffer()
    print(f"Connected to ESP32 via USB at {usb_port}")
except:
    print("Could not find ESP32. Is the cable a DATA cable?")
    exit()

try:
    while True:
        cmd = input("Send Command (USB): ")
        ser.write((cmd + "\n").encode('utf-8'))
        
        time.sleep(0.1)
        if ser.in_waiting > 0:
            reply = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"ESP32 says: {reply}")
except KeyboardInterrupt:
    ser.close()
