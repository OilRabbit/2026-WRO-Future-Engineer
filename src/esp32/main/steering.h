#ifndef STEERING_H
#define STEERING_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "tft.h"

#define SERVO_PIN 4

#define MAX_STEERING_ANGLE 20

// Clockwise is +ve
extern float steering_percentage; 
extern Servo steering_motor;

void steeringInit();
void reset_steering();
void steeringloop(void* parameters);
void showSteering(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour, bool clearDisplay);

#endif