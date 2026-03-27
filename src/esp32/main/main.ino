#include "main.h"

// Create threads for different tasks
TaskHandle_t blinkledThread = NULL;
TaskHandle_t displayThread = NULL;
TaskHandle_t OCThread = NULL;
// TaskHandle_t IMUThread = NULL;
// TaskHandle_t SteeringThread = NULL;
// TaskHandle_t MotorEncThread = NULL;

void setup() {
  // Initialization
  // motor_preinit_safe();
  Serial.begin(115200);
  tft.init();
  tft.displayln(30, 0, 2, "Initializing...", TFT_WHITE, true);
  
  // Showing the init status of each components
  int line_iter = 2;
  #ifdef FENZY_MODE
    tft.displayLeftln(line_iter++, 2, "LED Init:", TFT_WHITE, false);
    tft.displayLeftln(line_iter++, 2, "Clk Init:", TFT_WHITE, false);
    tft.displayLeftln(line_iter++, 2, "Srl Init:", TFT_WHITE, false);
    tft.displayLeftln(line_iter++, 2, "Btn Init:", TFT_WHITE, false);
    // tft.displayLeftln(line_iter++, 2, "Imu Init:", TFT_WHITE, false);
    // tft.displayLeftln(line_iter++, 2, "Str Init:", TFT_WHITE, false);
    // tft.displayLeftln(line_iter++, 2, "Mtr Init:", TFT_WHITE, false);
  #endif

  Serial.println("ESP_INIT");
  
  line_iter = 2;
  led_init();
  #ifdef FENZY_MODE
    delay(100);
    tft.displayRightln(line_iter++, 2, "Done", TFT_GREEN, false);
  #endif

  internalClock.start();
  #ifdef FENZY_MODE
    delay(100);
    tft.displayRightln(line_iter++, 2, "Done", TFT_GREEN, false);
  #endif

  Serial.begin(115200);
  #ifdef FENZY_MODE
    delay(100);
    tft.displayRightln(line_iter++, 2, "Done", TFT_GREEN, false);
  #endif

  btn_init();
  #ifdef FENZY_MODE
    delay(100);
    tft.displayRightln(line_iter++, 2, "Done", TFT_GREEN, false);
  #endif

  // imu_init();
  // #ifdef FENZY_MODE
  //   delay(100);
  //   tft.displayRightln(line_iter++, 2, "Done", TFT_GREEN, false);
  // #endif

  // steeringInit();
  // #ifdef FENZY_MODE
  //   delay(100);
  //   tft.displayRightln(line_iter++, 2, "Done", TFT_GREEN, false);
  // #endif

  // motor_init();
  // #ifdef FENZY_MODE
  //   delay(100);
  //   tft.displayRightln(line_iter++, 2, "Done", TFT_GREEN, false);
  // #endif


  xTaskCreatePinnedToCore(
    blinkledLED_test,         // Task function
    "Blink LED",              // Task name
    10000,                    // Stack size (bytes)
    NULL,                     // Parameters
    1,                        // Priority
    &blinkledThread,          // Task handle
    1                         // Core 1
  );

  // xTaskCreatePinnedToCore(
  //   getYPRloop,               // Task function
  //   "Get IMU Data",           // Task name
  //   10000,                    // Stack size (bytes)
  //   NULL,                     // Parameters
  //   1,                        // Priority
  //   &IMUThread,               // Task handle
  //   1                         // Core 1
  // );

  // xTaskCreatePinnedToCore(
  //   steeringloop,             // Task function
  //   "Steering",               // Task name
  //   10000,                    // Stack size (bytes)
  //   NULL,                     // Parameters
  //   1,                        // Priority
  //   &SteeringThread,          // Task handle
  //   1                         // Core 1
  // );

  // xTaskCreatePinnedToCore(
  //   motor_encloop,            // Task function
  //   "Motor Encoder",          // Task name
  //   10000,                    // Stack size (bytes)
  //   NULL,                     // Parameters
  //   1,                        // Priority
  //   &MotorEncThread,          // Task handle
  //   1                         // Core 1
  // );

  #ifdef FENZY_MODE
    delay(500);
  #else
    delay(300);
  #endif

  tft.clear();

  xTaskCreatePinnedToCore(
    displayData,              // Task function
    "Display info",           // Task name
    10000,                    // Stack size (bytes)
    NULL,                     // Parameters
    1,                        // Priority
    &displayThread,           // Task handle
    1                         // Core 1
  );

  Serial.println("ESP_IDLE");

  xTaskCreatePinnedToCore(
    OCmain,                  // Task function
    "OC",                    // Task name
    10000,                    // Stack size (bytes)
    NULL,                     // Parameters
    1,                        // Priority
    &OCThread,               // Task handle
    1                         // Core 1
  );
}

void loop() {
  // Nothing as the threads handle everything
}