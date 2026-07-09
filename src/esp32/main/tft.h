#ifndef TFT_H
#define TFT_H

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include "timestamp.h"
#include <string>

// Colour code for text colour on the LCD
#define TFT_BLACK 0x0000
#define TFT_WHITE 0xFFFF
#define TFT_RED 0xF800
#define TFT_GREEN 0x07E0
#define TFT_BLUE 0x001F
#define TFT_CYAN 0x07FF
#define TFT_MAGENTA 0xF81F
#define TFT_YELLOW 0xFFE0
#define TFT_ORANGE 0xFC00

// Column of the LCD
typedef enum {
  TFT_LEFT_CLN,
  TFT_RIGHT_CLN
} TFT_COLUMN;

// Class of the LCD for LCD's functions
class TFT
{
  public:
    void init();
    void textconfig(uint16_t text_colour, int textsize, int xpos, int ypos);
    void clear();
    void clearln(TFT_COLUMN column, int line_number);
    void setTextColour(uint16_t text_colour);
    void display(int xpos, int ypos, int textsize, const char text[], uint16_t text_colour, bool clearDisplay);
    void displayln(int xpos, int line_number, int textsize, const char text[], uint16_t text_colour, bool clearDisplay);
    void displayLeft(int ypos, int textsize, const char text[], uint16_t text_colour, bool clearDisplay);
    void displayLeftln(int line_number, int textsize, const char text[], uint16_t text_colour, bool clearDisplay);
    void displayRight(int ypos, int textsize, const char text[], uint16_t text_colour, bool clearDisplay);
    void displayRightln(int line_number, int textsize, const char text[], uint16_t text_colour, bool clearDisplay);
};

void displayData(void *parameters);

extern TFT tft;

#endif
