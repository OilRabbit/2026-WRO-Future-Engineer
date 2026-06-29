import serial
import time

class ESP32Communicator:
    def __init__(self, port='/dev/ttyACM1', baudrate=115200, timeout=0.05):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_conn = None
        self.is_connected = False
        self._rx_buffer = ""

    def connect(self):
        while not self.is_connected:
            try:
                self.serial_conn = serial.Serial(
                    self.port,
                    self.baudrate,
                    timeout=self.timeout,
                    write_timeout=0.1,
                )
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
            waiting = self.serial_conn.in_waiting
            if waiting > 0:
                chunk = self.serial_conn.read(waiting).decode('utf-8', errors='ignore')
                self._rx_buffer += chunk.replace('\r', '')
                if '\n' in self._rx_buffer:
                    reply, self._rx_buffer = self._rx_buffer.split('\n', 1)
                    reply = reply.strip()
                    if reply:
                        return reply
        return None

    def disconnect(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.is_connected = False
            print("ESP32 disconnected")
