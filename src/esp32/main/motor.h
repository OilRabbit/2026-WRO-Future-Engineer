#pragma once
#include <Arduino.h>
#include <Ticker.h>
#include <math.h>
#include "tft.h"

// Pins of the motor and encoder connected
#define PIN_M_BRAKE   5    // LOW = brake, HIGH = run
#define PIN_M_PWM     6    // LEDC PWM pin
#define PIN_M_DIR     7    // DIR pin
#define PIN_ENC_A     39   // encoder A
#define PIN_ENC_B     17   // encoder B

// Encoder setting
#ifndef ENCODER_CPR
#define ENCODER_CPR   600   // counts per mechanical revolution
#endif

// PWM setting
#define PWM_FREQ_HZ   20000     // 20 kHz
#define PWM_BITS      8
#define PWM_MAX_DUTY  ((1u << PWM_BITS) - 1)
extern const bool PWM_ACTIVE_LOW;   // defined in motor.cpp

// Encoder value from the distance travelled by the car
#define ENC_PER_CM 1.44

// Stop modes for the motor
typedef enum{
  COAST = 0, 
  BRAKE = 1 
} BRAKE_TYPE;

// Global variables storing the motor encoder value
extern volatile long MOTOR_ENCODER_COUNT;   // live value (PCNT reading)
extern long          MOTOR_ENCODER_VALUE;   // last snapshotted value by read_encoder()

// Safe preinit function for the motor
static inline void motor_preinit_safe() {
  pinMode(PIN_M_BRAKE, OUTPUT);
  digitalWrite(PIN_M_BRAKE, LOW);                               // assert brake

  pinMode(PIN_M_PWM, OUTPUT);
  digitalWrite(PIN_M_PWM, /*OFF*/ PWM_ACTIVE_LOW ? HIGH : LOW); // force PWM OFF level

  pinMode(PIN_M_DIR, OUTPUT);
  digitalWrite(PIN_M_DIR, LOW);
}

void motor_init();                              
void read_encoder();                            
void reset_encoder();                           
void motor_move(int speed_percentage);          
void motor_stop(BRAKE_TYPE brake_method);       
void motor_on_degree(int deg, int speed_percentage, BRAKE_TYPE brake_method);
bool motor_degree_accel(float dist, float accel_dist, float decel_dist, float init_speed, float max_speed);
void motor_encloop(void* parameters);
void showEncoder(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour, bool clearDisplay);