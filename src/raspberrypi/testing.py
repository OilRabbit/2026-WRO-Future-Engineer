import serial
import time

# Initialize Serial (USB-to-TTL or GPIO pins)
# On Pi, /dev/ttyS0 is usually the hardware serial port
ser = serial.Serial('/dev/serial0', 9600, timeout=1)

def send_control(steering, speed):
    # Format: "S[angle]V[speed]\n" -> e.g., "S90V100\n"
    # The '\n' acts as a terminator so the ESP32 knows the packet is done
    command = f"S{steering}V{speed}\n"
    ser.write(command.encode('utf-8'))

# Example loop
while True:
    # Your OpenCV logic would go here
    steering_val = 105  # Turn slightly right
    speed_val = 50      # Half speed

    print("Command sent!")

    send_control(steering_val, speed_val)
    time.sleep(0.02)    # 50Hz update rate
