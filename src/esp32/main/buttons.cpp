#include "HardwareSerial.h"
#include "buttons.h"
#include "imu.h"
#include "steering.h"
#include "motor.h"

static inline uint8_t pinOf(TFT_BTNS b) { return static_cast<uint8_t>(b); }

// Button init function
void btn_init(){
  pinMode(pinOf(TFT_BTN1), INPUT_PULLUP);
  pinMode(pinOf(TFT_BTN2), INPUT_PULLUP);
  pinMode(pinOf(TFT_BTN3), INPUT_PULLUP);
  // pinMode(pinOf(TFT_JS_BTN), INPUT_PULLUP);
  // pinMode(pinOf(TFT_JS_UP), INPUT_PULLUP);
  // pinMode(pinOf(TFT_JS_DOWN), INPUT_PULLUP);
  // pinMode(pinOf(TFT_JS_LEFT), INPUT_PULLUP);
  // pinMode(pinOf(TFT_JS_RIGHT), INPUT_PULLUP);
}

/**
 * @brief Check if the specific button is being pressed and hold
 * 
 * @return bool; true if being pressed and hold
 */
bool is_btn_pressed(TFT_BTNS btn){
  return (digitalRead(pinOf(btn)) == LOW);
}

/**
 * @brief Check if the specific button is being pressed and released once
 * 
 * @return bool; true if being pressed and released once
 */
bool is_btn_bumped(TFT_BTNS btn){
  const uint8_t pin = pinOf(btn);
  static uint64_t wasPressedMask = 0;
  const uint64_t bit = 1ULL << pin;
  const bool nowPressed = (digitalRead(pin) == LOW);
  const bool edge = nowPressed && !(wasPressedMask & bit);

  if (nowPressed) wasPressedMask |= bit;
  else            wasPressedMask &= ~bit;

  return edge;
}

/**
 * @brief A function to put into display thread for showing the button state
 * 
 * @param column; (enum) COLUMN; A enum defined in oled.h indicating which column the data should be displaced at
 * @param line_number; int; The line number where the data should be displaced at (0 ~ 3)
 * @param size; int; The size of the text being displaced (1 ~ 2)
 * @param clearDisplay; bool; Set true to clear the whole OLED display everytime before displaying the battery percentage
 */
void showbtnState(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour = TFT_WHITE, bool clearDisplay = false){
  String btn_text = String("Bt1:") + (is_btn_pressed(TFT_BTN1) ? "Pre" : "Rel");
  if (is_btn_pressed(TFT_BTN3)){
    reset_encoder();
    tft.clear();
  } 
  if (column == TFT_LEFT_CLN){
    tft.clearln(TFT_LEFT_CLN, line_number);
    tft.displayLeftln(line_number, text_size, btn_text.c_str(), text_colour, false);
  } else {
    tft.clearln(TFT_RIGHT_CLN, line_number);
    tft.displayRightln(line_number, text_size, btn_text.c_str(), text_colour, false);
  }
}