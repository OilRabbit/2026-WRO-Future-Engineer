#ifndef SERIAL_H
#define SERIAL_H

#include <Arduino.h>
#include "Timer.h"
#include "timestamp.h"
#include <algorithm>
#include "main.h"
#include "buttons.h"
#include "tft.h"

void send_msg(String message);
String receiveNprint_msg(TFT_COLUMN column, int line_number, int textsize, uint16_t text_colour, bool clearDisplay);

#endif