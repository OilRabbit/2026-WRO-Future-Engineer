#include <cmath>
#include "calculation.h"

/**
 * @brief To get the sign of a variable
 * @param num; float; the variable to get the sign
 *
 * @return int; 1, -1 or 0
*/
int sign(float num){
  if (num > 0) return 1;
  else if (num < 0) return -1;
  else return 0;
}

// Converting an angle from degree to radian
float deg2rad(float deg){
  return deg * M_PI / 180;
}

// Converting an angle from radian to degree
float rad2deg(float rad){
  return rad * 180 / M_PI;
}

// Get the smaller value among two input values
float min_val(float v1, float v2){
  if (v1 < v2) return v1;
  else return v2;
}

// Get the larger value among two input values
float max_val(float v1, float v2){
  if (v1 > v2) return v1;
  else return v2;
}