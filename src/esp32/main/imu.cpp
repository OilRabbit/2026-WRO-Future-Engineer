#include "imu.h"

// Wiring of the IMU 
#define I2C_SDA   9
#define I2C_SCL   8
#define AD0_VAL   1      // 1 => 0x69 (SparkFun default), 0 => 0x68
#define WIRE_PORT Wire
#define SERIAL_PORT Serial

// IMU global variable
ICM_20948_I2C icm;

// Public YPR values
volatile double imu_yaw   = 0.0;
volatile double imu_pitch = 0.0;
volatile double imu_roll  = 0.0;

// Internal unwrapping state
static volatile double yaw_unwrapped       = 0.0;
static volatile double yaw_zero_unwrapped  = 0.0;
static bool   have_prev_yaw = false;
static double prev_yaw_deg  = 0.0;

// Helper functions
static inline double clamp01(double v){
  return v < 0.0 ? 0.0 : (v > 1.0 ? 1.0 : v);
}

// Return smallest signed difference a-b in (-180, 180]
static inline double angdiff_deg(double a_deg, double b_deg){
  double d = fmod(a_deg - b_deg + 540.0, 360.0) - 180.0;
  if (d <= -180.0) d += 360.0;
  return d;
}

// Drain FIFO and return the *latest* yaw (deg in [-180,180])
static bool get_yaw_deg_latest(double& yaw_deg){
  icm_20948_DMP_data_t data;
  bool got = false;

  do {
    icm.readDMPdataFromFIFO(&data);

    if ((icm.status == ICM_20948_Stat_Ok) ||
        (icm.status == ICM_20948_Stat_FIFOMoreDataAvail))
    {
      if (data.header & DMP_header_bitmap_Quat6){
        // Vendor vector part, Q30 fixed-point
        const double Q30 = 1073741824.0;
        const double q1v = (double)data.Quat6.Data.Q1 / Q30; // vendor x
        const double q2v = (double)data.Quat6.Data.Q2 / Q30; // vendor y
        const double q3v = (double)data.Quat6.Data.Q3 / Q30; // vendor z

        const double w2  = clamp01(1.0 - (q1v*q1v + q2v*q2v + q3v*q3v));
        const double q0  = sqrt(w2);                         // scalar

        // ---- Axis mapping (A). If yaw is flipped on your rig, try mapping B below ----
        double qw = q0, qx = q2v, qy = q1v, qz = -q3v;      // Mapping A
        // double qw = q0, qx = q1v, qy = q2v, qz = q3v;    // Mapping B (alternative)

        // Z-yaw (deg) in [-180, 180]
        const double t3 = 2.0 * (qw*qz + qx*qy);
        const double t4 = 1.0 - 2.0 * (qy*qy + qz*qz);
        yaw_deg = atan2(t3, t4) * 180.0 / PI;
        got = true;
      }
    }
  } while (icm.status == ICM_20948_Stat_FIFOMoreDataAvail);

  if (!got) vTaskDelay(pdMS_TO_TICKS(5));
  return got;
}

// Init function for the IMU
void imu_init(){
  WIRE_PORT.begin(I2C_SDA, I2C_SCL, 400000);
  WIRE_PORT.setClock(400000);

  bool initialized = false;
  while (!initialized){
    icm.begin(WIRE_PORT, AD0_VAL);
    if (icm.status != ICM_20948_Stat_Ok){
      delay(250);
    } else {
      initialized = true;
    }
  }

  bool ok = true;
  ok &= (icm.initializeDMP() == ICM_20948_Stat_Ok);
  ok &= (icm.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);
  ok &= (icm.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // max rate
  ok &= (icm.enableFIFO() == ICM_20948_Stat_Ok);
  ok &= (icm.enableDMP() == ICM_20948_Stat_Ok);
  ok &= (icm.resetDMP() == ICM_20948_Stat_Ok);
  ok &= (icm.resetFIFO() == ICM_20948_Stat_Ok);
  if (!ok) {
    while (1) { delay(100); }
  }

  have_prev_yaw      = false;
  yaw_unwrapped      = 0.0;
  yaw_zero_unwrapped = 0.0;
  imu_yaw = imu_pitch = imu_roll = 0.0;
}

/**
 * @brief The main thread function for refreshing IMU values 
 */
void getYPRloop(void *){
  for (;;) {
    double y_deg;
    if (!get_yaw_deg_latest(y_deg)) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    if (!have_prev_yaw){
      prev_yaw_deg       = y_deg;
      yaw_unwrapped      = 0.0;
      yaw_zero_unwrapped = 0.0;   // start with current yaw as 0
      have_prev_yaw      = true;
      imu_yaw = 0.0;
      imu_pitch = 0.0;
      imu_roll  = 0.0;
      continue;
    }

    // Robust unwrap (works even if Δ>180° due to skipped samples)
    double d = angdiff_deg(y_deg, prev_yaw_deg);
    yaw_unwrapped += d;
    prev_yaw_deg = y_deg;

    // Apply zero offset
    imu_yaw = yaw_unwrapped - yaw_zero_unwrapped;

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

/**
 * @brief Resetting the IMU yaw angle
 */
void imu_resetYaw(){
  yaw_zero_unwrapped = yaw_unwrapped;
}

/**
 * @brief Function to print the (yaw) angle from the IMU on the LCD monitor
 * 
 * @param column; TFT_COLUMN; the column on the LCD where the info is being printed
 * @param line_number; int; the line number on the LCD where the info is being printed
 * @param text_size; int; text size of the info on the LCD (1 or 2)
 * @param text_colour; uint16_t; text colour printed on the LCD
 * @param clearDisplay; bool; (UNUSED) whether the display will be cleared before running
 * 
 */
void showIMU(TFT_COLUMN column, int line_number, int text_size, uint16_t text_colour = TFT_WHITE, bool clearDisplay = false){
  String yaw_text = String("Y:") + String(imu_yaw);
  // String pitch_text = String("Pit:") + String(imu_pitch);
  // String roll_text = String("Rol:") + String(imu_roll);
  if (column == TFT_LEFT_CLN){
    tft.clearln(TFT_LEFT_CLN, line_number);
    tft.displayLeftln(line_number, text_size, yaw_text.c_str(), text_colour, false);
    // tft.clearln(TFT_LEFT_CLN, line_number + 1);
    // tft.displayLeftln(line_number + 1, text_size, pitch_text.c_str(), text_colour, false);
    // tft.clearln(TFT_LEFT_CLN, line_number + 2);
    // tft.displayLeftln(line_number + 2, text_size, roll_text.c_str(), text_colour, false);
  } else {
    tft.clearln(TFT_RIGHT_CLN, line_number);
    tft.displayRightln(line_number, text_size, yaw_text.c_str(), text_colour, false);
    // tft.clearln(TFT_RIGHT_CLN, line_number + 1);
    // tft.displayRightln(line_number + 1, text_size, pitch_text.c_str(), text_colour, false);
    // tft.clearln(TFT_RIGHT_CLN, line_number + 2);
    // tft.displayRightln(line_number + 2, text_size, roll_text.c_str(), text_colour, false);
  }
}