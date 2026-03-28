#include "tft.h"
#include "buttons.h"
#include "OC.h"
#include "imu.h"
#include "steering.h"
#include "motor.h"

// Pins connected to the LCD hat
#define TFT_SCLK 40   // HAT SCLK -> ESP32-S3 GPIO40
#define TFT_MOSI 41   // HAT MOSI -> ESP32-S3 GPIO41
#define TFT_CS   21
#define TFT_DC   42
#define TFT_RST  1
#define TFT_BL   2    // backlight (active HIGH)

#define TFT_HEIGHT 240 //pixels
#define TFT_WIDTH 240 //pixels
#define TEXT_HEIGHT 16
#define NUM_ROWS TFT_HEIGHT / TEXT_HEIGHT

Adafruit_ST7789 tft_raw(TFT_CS, TFT_DC, TFT_RST);
TFT tft;

// Init function for the LCD
void TFT::init(){
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);          // turn backlight ON
  SPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);  // MISO = -1 (write-only LCD)
  tft_raw.init(TFT_HEIGHT, TFT_WIDTH);
  tft_raw.setRotation(3);
  tft_raw.setTextSize(2);
}

// Text configuration when printing on the LCD
void TFT::textconfig(uint16_t text_colour, int textsize, int xpos, int ypos){
  tft_raw.setTextColor(text_colour);
  tft_raw.setTextSize(textsize);
  tft_raw.setCursor(xpos, ypos);
}

// Clear the LCD
void TFT::clear(){
  tft_raw.fillScreen(TFT_BLACK);
}

// Clear a specific line on the LCD
void TFT::clearln(TFT_COLUMN column, int line_number){
  tft_raw.fillRect(column * 120, line_number * TEXT_HEIGHT, TFT_WIDTH / 2, TEXT_HEIGHT, TFT_BLACK);
}

// Set the text colour being printed on the LCD
void TFT::setTextColour(uint16_t text_colour){
  tft_raw.setTextColor(text_colour);
}

// Show text on the LCD
void TFT::display(int xpos, int ypos, int textsize, const char text[], uint16_t text_colour, bool clearDisplay){
  if(clearDisplay) clear();
  tft_raw.setCursor(xpos, ypos);
  setTextColour(text_colour);
  tft_raw.setTextSize(textsize);
  tft_raw.printf(text);
}

// Show text on a specific line on the LCD
void TFT::displayln(int xpos, int line_number, int textsize, const char text[], uint16_t text_colour, bool clearDisplay){
  if(clearDisplay) clear();
  setTextColour(text_colour);
  tft_raw.setCursor(xpos, line_number * TEXT_HEIGHT);
  tft_raw.setTextSize(textsize);
  tft_raw.printf(text);
}

// Show text on the left column of the LCD
void TFT::displayLeft(int ypos, int textsize, const char text[], uint16_t text_colour, bool clearDisplay){
  if(clearDisplay) clear();
  display(0, ypos, textsize, text, text_colour, clearDisplay);
}

// Show text on the left column on a specific line of the LCD
void TFT::displayLeftln(int line_number, int textsize, const char text[], uint16_t text_colour, bool clearDisplay){
  if(clearDisplay) clear();
  displayln(0, line_number, textsize, text, text_colour, clearDisplay);
}

// Show text on the right column of the LCD
void TFT::displayRight(int ypos, int textsize, const char text[], uint16_t text_colour, bool clearDisplay){
  if(clearDisplay) clear();
  display(TFT_WIDTH / 2, ypos, textsize, text, text_colour, clearDisplay);
}

// Show text on the right column on a specific line of the LCD
void TFT::displayRightln(int line_number, int textsize, const char text[], uint16_t text_colour, bool clearDisplay){
  if(clearDisplay) clear();
  displayln(TFT_WIDTH / 2, line_number, textsize, text, text_colour, clearDisplay);
}

// A function to display everying from different threads on the LCD
void displayData(void *parameters){
  while(1){
    int ln = 2;
    // Put your display functions here. The first one MUST clear display, while those after that MUST NOT clear the display
    showInternalClock();
    showOCTime(TFT_LEFT_CLN, ln++, 2, TFT_WHITE, false);
    // showbtnState(TFT_LEFT_CLN, ln++, 2, TFT_WHITE, false);
    // showIMU(TFT_LEFT_CLN, ln++, 2, TFT_WHITE, false);
    // showSteering(TFT_LEFT_CLN, ln++, 2, TFT_WHITE, false);
    // showEncoder(TFT_LEFT_CLN, ln++, 2, TFT_WHITE, false);
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}