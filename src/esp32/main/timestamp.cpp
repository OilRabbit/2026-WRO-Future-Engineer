#include "timestamp.h"

/* Global variable of the internal clock. For time reading purpose only. Never reset while running. */
Timer internalClock;

/**
 * @brief A function to put into display thread for showing the time elapsed from the start of the smart car
 * 
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
 */
void showInternalClock(){
  tft.clearln(TFT_LEFT_CLN, 0);
  static long total_ms = 0;
  static long seconds = 0;
  static long milliseconds = 0;
  total_ms = internalClock.read();
  seconds = total_ms / 1000;
  milliseconds = total_ms % 1000;
  String curr_time_text = String(seconds) + "." + String(milliseconds);
  tft.displayLeftln(0, 2, curr_time_text.c_str(), TFT_WHITE, false);
}