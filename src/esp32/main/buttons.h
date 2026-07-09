#ifndef BUTTONS_H
#define BUTTONS_H

#include <Arduino.h>
#include "tft.h"

// Button pins of the LCD monitor
typedef enum {
  TFT_JS_BTN = 11,
  TFT_BTN1 = 12,
  TFT_BTN2 = 13,
  TFT_BTN3 = 14,
  TFT_JS_UP = 15,
  TFT_JS_DOWN = 16,
  TFT_JS_LEFT = 17,
  TFT_JS_RIGHT = 18,
} TFT_BTNS;

void btn_init();
bool is_btn_pressed(TFT_BTNS btn);
bool is_btn_bumped(TFT_BTNS btn);
void showbtnState(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour, bool clearDisplay);

#endif
