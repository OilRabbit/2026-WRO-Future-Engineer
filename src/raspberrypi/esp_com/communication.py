import serial
import time

class ESP32Communicator:
    def __init__(self, port='/dev/ttyACM1', baudrate=115200, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.is_connected = False

    def connect(self):
        while not self.is_connected:
            try:
                self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
                time.sleep(0.5)
                self.serial_conn.reset_input_buffer()
                print(f"ESP32 connected at {self.port}")
                self.is_connected = True
            except serial.SerialException:
                print("Could not find ESP32")
                time.sleep(0.5)

    def send_command(self, cmd):
        if self.is_connected and self.serial_conn:
            self.serial_conn.write((f"{cmd}\n").encode('utf-8'))

    def read_message(self):
        if self.is_connected and self.serial_conn:
            if self.serial_conn.in_waiting > 0:
                reply = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                return reply
        return None

    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.is_connected = False
            print("ESP32 disconnected")
