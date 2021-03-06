#ifndef ST_ACCELGYROCONFIG_H
#define ST_ACCELGYROCONFIG_H

#include <Arduino.h>


// How is put your device X and Y reference system?
typedef enum 
{ 
    ACC_GYRO_X_AXIS_FORWARD = 0, // X axis to forward
    ACC_GYRO_X_AXIS_LEFT = 90 , // X axis to left side
    ACC_GYRO_X_AXIS_BACKWARD = 180, // X axis to backward
    ACC_GYRO_X_AXIS_RIGHT = 270 // X axis to rigth side
} accGyroXposition;



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
// Gyroscope Sensitivity Scale Factor (rad/s)
const double GYRO_SENSITIVITY_SCALE_FACTOR_0250_RADS = GYRO_SENSITIVITY_SCALE_FACTOR_0250 * RAD_TO_DEG;
const double GYRO_SENSITIVITY_SCALE_FACTOR_0500_RADS = GYRO_SENSITIVITY_SCALE_FACTOR_0500 * RAD_TO_DEG;
const double GYRO_SENSITIVITY_SCALE_FACTOR_1000_RADS = GYRO_SENSITIVITY_SCALE_FACTOR_1000 * RAD_TO_DEG;
const double GYRO_SENSITIVITY_SCALE_FACTOR_2000_RADS = GYRO_SENSITIVITY_SCALE_FACTOR_2000 * RAD_TO_DEG;

// Accelerometer configuration constants
// Accelerometer Sensitivity
const uint8_t ACC_FULL_SCALE_RANGE_02 = 0X00; // pm 2 g
const uint8_t ACC_FULL_SCALE_RANGE_04 = 0X01; // pm 4 g
const uint8_t ACC_FULL_SCALE_RANGE_08 = 0X02; // pm 8 g
const uint8_t ACC_FULL_SCALE_RANGE_16 = 0X03; // pm 16 g
// Accelerometer Sensitivity Scale Factor (/g)
const double ACC_SENSITIVITY_SCALE_FACTOR_02 = 16384.0L;
const double ACC_SENSITIVITY_SCALE_FACTOR_04 = 8192.0L;
const double ACC_SENSITIVITY_SCALE_FACTOR_08 = 4096.0L;
const double ACC_SENSITIVITY_SCALE_FACTOR_16 = 2084.0L;
#endif //ST_ACCELGYROCONFIG_H
