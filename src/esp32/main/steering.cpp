#include "steering.h"

/* Global variable for storing the steering angle percentage (-100% ~ 100%) */
float steering_percentage;

Servo steering_motor;

/**
 * @brief Initialize the servo motor
 * 
 */
void steeringInit(){
  steering_motor.attach(SERVO_PIN);
}

/**
 * @brief Resetting the servo motor to "zero-pos"
 * 
 */
void reset_steering(){
  steering_motor.write(90);
}

/**
 * @brief A function which control the steering angle of the servo motor by a given steering angle position
 * 
 * @param steering_percentage; int; Steering angle percentage (-100% ~ 100%)
 */
void steeringloop(void* parameters){
  while(1){
    int servo_angle = 0;
    if (steering_percentage > 100) servo_angle = int(91 + MAX_STEERING_ANGLE);
    else if (steering_percentage < -100) servo_angle = int(91 - MAX_STEERING_ANGLE);
    else servo_angle = int(91 + MAX_STEERING_ANGLE * steering_percentage / 100);
    steering_motor.write(servo_angle);
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

/**
 * @brief A function to put into display thread for showing the steering angle percentage
 * 
 * @param column; (enum) COLUMN; A enum defined in oled.h indicating which column the data should be displaced at
 * @param line_number; int; The line number where the data should be displaced at (0 ~ 3)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
 */
void showSteering(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour = TFT_WHITE, bool clearDisplay = false){
  String steering_text = "Str: " + String(int(steering_percentage)) + "%";
  if (column == TFT_LEFT_CLN){
    tft.clearln(TFT_LEFT_CLN, line_number);
    tft.displayLeftln(line_number, text_size, steering_text.c_str(), text_colour, false);
  } else {
    tft.clearln(TFT_RIGHT_CLN, line_number);
    tft.displayRightln(line_number, text_size, steering_text.c_str(), text_colour, false);
  }
}