void setup() {
  // On the S3, 'Serial' is the USB-C Hardware CDC
  Serial.begin(115200); 
  delay(2000);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    // Echo it back to the Pi
    Serial.print("USB_ACK: ");
    Serial.println(command);
  }
}
