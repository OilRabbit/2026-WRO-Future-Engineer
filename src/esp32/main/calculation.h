#ifndef CALCULATION_H
#define CALCULATION_H

#include <Arduino.h>
#include <algorithm>
#include <cmath>

#define E 2.718281828

int sign(float num);
float deg2rad(float deg);
float rad2deg(float rad);
float min_val(float v1, float v2);
float max_val(float v1, float v2);

#endif