### SPI LCD (ST7789 HAT)

* SCLK (1st row, pin 12) → **GPIO40**
* MOSI (1st row, pin 10) → **GPIO41**
* CS (2nd row, pin 12) → **GPIO21**
* DC (2nd row, pin 11) → **GPIO42**
* RST (1st row, pin 7) → **GPIO1**
* BL (2nd row, pin 9) → **GPIO2** (drive HIGH to turn backlight on)
* VCC (1st row, pin 1) → 3V3
* GND (2nd row, pin 3) → GND

### HAT Buttons & Joystick  *(active-LOW, use `INPUT_PULLUP`)*

* KEY1 (2nd row, pin 20) → **GPIO12**
* KEY2 (2nd row, pin 19) → **GPIO13**
* KEY3 (2nd row, pin 18) → **GPIO14**
(UNUSED) * JOY UP → **GPIO15**
(UNUSED) * JOY DOWN → **GPIO16**
(UNUSED) * JOY LEFT → **GPIO17**
(UNUSED) * JOY RIGHT → **GPIO18**
(UNUSED) * JOY PRESS → **GPIO11**

### Qwiic IMU (ICM-20948, I²C @ 3.3 V), share with Pixy2

* SDA → **GPIO8** (purple)
* SCL → **GPIO9**
* VCC → 3V3, GND → GND

### Pixy 2.1 (UART)

* SDA (3rd row, pin 1) → **GPIO8**
* SCL (5th row, pin 1) → **GPIO9**
* VCC (1st row, pin 2) → 5V
* GND (3rd row, pin 2) → GND

### ToF  *(power both from 5 V)*

* ToF 1 (Left): **TX → GPIO10**, **RX → GPIO11**
* ToF 2 (Left): **TX → GPIO48**, **RX → GPIO47**

### BM50 brushless motor with encoder
* Brake (blue) → **GPIO5**
* PWM (green) → **GPIO6**
* Dir (brown) → **GPIO7**
* ENCPinA → **GPIO39** (yellow)
* ENCPinB → **GPIO17** (white)

### Servo

* Signal (orang) → **GPIO4** (LEDC @ 50 Hz)
* Power servo (red) from 5–6 V (not 3V3); GND (brown) common

### HC-SR04 x2  *(power both from 5 V; **ECHO must be level-shifted to 3.3 V**)*

* (UNUSED) Sensor A (Left): **TRIG → GPIO10**, **ECHO → GPIO11** (through resistor divider)
* (UNUSED) Sensor B (Right): **TRIG → GPIO47**, **ECHO → **GPIO48** (through resistor divider)
* (UNUSED) Sensor C (Front): **TRIG → GPIO15**, **ECHO → **GPIO16** (through resistor divider)

#### Notes / gotchas

* **Do not use**: GPIO0 (BOOT strap), **GPIO19/20** (USB D−/D+), **GPIO45/46** (input-only; avoid for TRIG/PWM).

