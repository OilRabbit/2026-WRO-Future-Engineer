#include "LED.h"

#define RGB_PIN 38
#define NUM_LEDS 1

Adafruit_NeoPixel rgb(NUM_LEDS, RGB_PIN, NEO_GRB + NEO_KHZ800);

void led_init(){
  rgb.begin();
}

void led_off(){
  rgb.clear();
  rgb.show();
}

void led_white(uint8_t brightness = 32){
  rgb.setBrightness(brightness);
  rgb.setPixelColor(0, rgb.Color(255, 255, 255)); 
  rgb.show();
}

void led_red(uint8_t brightness = 32){
  rgb.setBrightness(brightness);
  rgb.setPixelColor(0, rgb.Color(255, 0, 0)); 
  rgb.show();
}

void led_green(uint8_t brightness = 32){
  rgb.setBrightness(brightness);
  rgb.setPixelColor(0, rgb.Color(0, 255, 0)); 
  rgb.show();
}

void led_blue(uint8_t brightness = 32){
  rgb.setBrightness(brightness);
  rgb.setPixelColor(0, rgb.Color(0, 0, 255)); 
  rgb.show();
}

void led_flash(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness = 32){
  rgb.setBrightness(brightness);
  rgb.setPixelColor(0, rgb.Color(0, 0, 255)); 
  rgb.show();
}

// For testing only
void blinkledLED_test(void *parameters){
  led_init();
  while(1){
    led_blue(5);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    led_off();
    vTaskDelay(500 / portTICK_PERIOD_MS);
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}