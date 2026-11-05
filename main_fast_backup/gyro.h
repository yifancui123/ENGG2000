// ===== gyro.h - MPU6050 body rate and angle for MQ Sentinel =====
// Measures turning about the vertical (the suspension line), in any
// mounting: gyroCalibrate finds "up" from gravity. No library, only Wire.
//
// Use:
//   setup():  gyroBegin();  gyroCalibrate(2000);   // satellite STILL
//   loop():   gyroUpdate();                        // EVERY loop
//             gyroRate() / gyroAngle()
//
// Wiring: VCC -> 5V, GND -> GND, SDA -> A4, SCL -> A5.
// Mount it firmly near the middle of the body, any way round (flat is
// best). Calibrate with the satellite hanging still, upright.
// Sign: + = anticlockwise seen from above (same as turnBy).

#ifndef GYRO_H
#define GYRO_H

#include <Arduino.h>

bool gyroBegin();                     // false if no MPU6050 answers
void gyroCalibrate(unsigned long ms); // bias + "up" direction, while still
void gyroUpdate();                    // read + integrate, every 5 ms
float gyroRate();                     // body rate, deg/s
float gyroAngle();                    // body angle since reset, deg
void gyroResetAngle();                // angle = 0

#endif
