#ifndef ST_ACCELGYROCONFIG_H
#define ST_ACCELGYROCONFIG_H

#include <Arduino.h>


// How is put your device X and Y reference system?
// X axis to rigth side
const double X_AXIS_RIGHT = 0;
// X axis to forward
const double X_AXIS_FORWARD = 90;
// X axis to left side
const double X_AXIS_LEFT = 180;
// X axis to backward
const double X_AXIS_BACKWARD = 270;


const word IMU_MPU6050_ADDRS = 0x68;

// Gyroscope configuration constants
// Gyroscope Sensitivity
const uint8_t GYRO_FULL_SCALE_RANGE_0250 = 0X00; // 205 º/sec
const uint8_t GYRO_FULL_SCALE_RANGE_0500 = 0X01; // 500 º/sec
const uint8_t GYRO_FULL_SCALE_RANGE_1000 = 0X02; // 1000 º/sec
const uint8_t GYRO_FULL_SCALE_RANGE_2000 = 0X03; // 2000 º/sec
// Gyroscope Sensitivity Scale Factor (º/s)
const double GYRO_SENSITIVITY_SCALE_FACTOR_0250 = 131.0;
const double GYRO_SENSITIVITY_SCALE_FACTOR_0500 = 65.5;
const double GYRO_SENSITIVITY_SCALE_FACTOR_1000 = 32.8;
const double GYRO_SENSITIVITY_SCALE_FACTOR_2000 = 16.4;

// Accelerometer configuration constants
// Accelerometer Sensitivity
const uint8_t ACC_FULL_SCALE_RANGE_02 = 0X00; // pm 2 g
const uint8_t ACC_FULL_SCALE_RANGE_04 = 0X01; // pm 4 g
const uint8_t ACC_FULL_SCALE_RANGE_08 = 0X02; // pm 8 g
const uint8_t ACC_FULL_SCALE_RANGE_16 = 0X03; // pm 16 g
// Accelerometer Sensitivity Scale Factor (/g)
const double ACC_SENSITIVITY_SCALE_FACTOR_02 = 16384;
const double ACC_SENSITIVITY_SCALE_FACTOR_04 = 8192;
const double ACC_SENSITIVITY_SCALE_FACTOR_08 = 4096;
const double ACC_SENSITIVITY_SCALE_FACTOR_16 = 2084;
#endif //ST_ACCELGYROCONFIG_H
