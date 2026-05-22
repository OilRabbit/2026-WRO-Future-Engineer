#ifndef MAIN_H
#define MAIN_H

#define FENZY_MODE 1

#include <string.h>
#include "LED.h"
#include "tft.h"
#include "timestamp.h"
#include "OC.h"
#include "buttons.h"
#include "imu.h"
#include "steering.h"
#include "motor.h"

extern TaskHandle_t blinkledThread;
extern TaskHandle_t displayThread;
extern TaskHandle_t OCThread;
extern TaskHandle_t SteeringThread;
extern TaskHandle_t MotorEncThread;

#endif