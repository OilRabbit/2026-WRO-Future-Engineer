#include "OC.h"

// A global variables for OC1
bool run_OC1 = false;
bool run_OC2 = false;
long OC_starttime = 0; 
long OC_endtime = 0; 

// Functions for safely suspend and resume threads
static inline void safeSuspend(TaskHandle_t h){ if (h) vTaskSuspend(h); }
static inline void safeResume(TaskHandle_t h){ if (h) vTaskResume(h); }

int current_target_speed = 0;
int current_target_steering = 0;
int current_target_distance = -1; 
bool is_moving_distance = false;

void OC1_program() {
  String cmd = receiveNprint_msg(TFT_LEFT_CLN, 13, 2, TFT_WHITE, false);
  
  if (cmd.length() > 0 && cmd != "OC1" && cmd != "OC2") {
    
    int firstComma = cmd.indexOf(',');
    int secondComma = cmd.indexOf(',', firstComma + 1);
    int thirdComma = cmd.indexOf(',', secondComma + 1);

    if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
      String speedStr = cmd.substring(0, firstComma);
      String steeringStr = cmd.substring(firstComma + 1, secondComma);
      String distanceStr = cmd.substring(secondComma + 1, thirdComma);
      
      speedStr.trim();
      steeringStr.trim();
      distanceStr.trim();
      
      current_target_speed = speedStr.toInt();
      current_target_steering = steeringStr.toInt();
      current_target_distance = distanceStr.toInt();
      
      if (current_target_distance > 0) {
        reset_encoder();
        is_moving_distance = true;
      } else {
        is_moving_distance = false; 
      }
      
    } else {
      send_msg("X");
    }
  }
  
  steering_percentage = current_target_steering; 

  if (is_moving_distance) {
    if (abs(MOTOR_ENCODER_COUNT) < current_target_distance) {
      motor_move(current_target_speed);
    } else {
      motor_move(0);
      is_moving_distance = false; 
      current_target_distance = -1;
      // send_msg("DONE"); 
    }
  } else {
    motor_move(current_target_speed);
  }
}

void OC2_program(){
  String cmd = receiveNprint_msg(TFT_LEFT_CLN, 13, 2, TFT_WHITE, false);
}

/**
 * @brief The main thread function for OC1
 */
void OCmain(void *){
  while (1){
    if (is_btn_bumped(TFT_BTN1) && !run_OC2){
      run_OC1 = !run_OC1;
      if (run_OC1) send_msg("ROC1");
      else send_msg("EOC1");
    } else if (is_btn_bumped(TFT_BTN2) && !run_OC1){
      run_OC2 = !run_OC2;
      if (run_OC2) send_msg("ROC2");
      else send_msg("EOC2");
    } else if (is_btn_bumped(TFT_BTN3)){
      motor_stop(BRAKE);
      steering_percentage = 0;
    }

    if (run_OC1){
      OC1_program();
    } else if (run_OC2){
      OC2_program();
    } else {
      String cmd = receiveNprint_msg(TFT_LEFT_CLN, 11, 2, TFT_WHITE, false);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

/**
 * @brief Function to print the time and status of OC1 run on the LCD monitor
 * 
 * @param column; TFT_COLUMN; the column on the LCD where the info is being printed
 * @param line_number; int; the line number on the LCD where the info is being printed
 * @param text_size; int; text size of the info on the LCD (1 or 2)
 * @param text_colour; uint16_t; text colour printed on the LCD
 * @param clearDisplay; bool; (UNUSED) whether the display will be cleared before running
 * 
 */
void showOCTime(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour = TFT_WHITE, bool clearDisplay = false){
  static long total_ms = 0;
  static long seconds = 0;
  static long milliseconds = 0;
  total_ms = OC_endtime - OC_starttime;
  seconds = total_ms / 1000;
  milliseconds = total_ms % 1000;
  String OC_time_text = String(seconds) + "." + String(milliseconds);
  String OC1_onoff_text = run_OC1 ? "OC1 ON" : "OC1 OFF";
  String OC2_onoff_text = run_OC2 ? "OC2 ON" : "OC2 OFF";
  tft.clearln(TFT_LEFT_CLN, line_number);
  tft.clearln(TFT_RIGHT_CLN, line_number);
  tft.displayLeftln(line_number, text_size, OC_time_text.c_str(), text_colour, false);
  tft.displayRightln(line_number++, text_size, OC1_onoff_text.c_str(), run_OC1 ? TFT_GREEN : TFT_RED, false);
  tft.clearln(TFT_LEFT_CLN, line_number);
  tft.clearln(TFT_RIGHT_CLN, line_number);
  tft.displayLeftln(line_number, text_size, OC_time_text.c_str(), text_colour, false);
  tft.displayRightln(line_number, text_size, OC2_onoff_text.c_str(), run_OC2 ? TFT_GREEN : TFT_RED, false);
}
