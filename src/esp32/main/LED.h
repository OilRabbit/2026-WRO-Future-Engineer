#ifndef LED_H
#define LED_H

#include <Adafruit_NeoPixel.h>

extern Adafruit_NeoPixel rgb;

void led_init();
void led_off();
void led_white(uint8_t brightness);
void led_red(uint8_t brightness);
void led_green(uint8_t brightness);
void led_blue(uint8_t brightness);
void led_flash(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness);
void blinkledLED_test(void *parameters); // For testing only

#endif
