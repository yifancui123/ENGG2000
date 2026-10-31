// ===== gyro.cpp - MPU6050 rate about the vertical, bias and angle =====
// Any mounting works (flat, on the wall, tilted): gyroCalibrate uses the
// accelerometer to find "up" (gravity) and the rate is the gyro turning
// about that axis, i.e. about the suspension line.

#include "gyro.h"
#include <Wire.h>

static const byte MPU_ADDR = 0x68;      // AD0 low
static const float LSB_PER_DPS = 65.5;  // +-500 deg/s range
// + must be anticlockwise from above. Should be right for any mounting;
// if turnBy(90) makes gyroAngle() go to about -90, change this to -1.
static const int GYRO_SIGN = 1;
static const unsigned long READ_US = 5000;

static float bias[3] = { 0, 0, 0 };     // raw gyro counts while still
static float up[3] = { 0, 0, 1 };       // unit vector "up" in chip axes
static float rate = 0;                  // deg/s
static float angle = 0;                 // deg
static unsigned long lastRead = 0;

static void writeReg(byte reg, byte value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// reads n int16 values starting at reg, false if the read failed
static bool readRegs(byte reg, int16_t *out, byte n) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (byte)(2 * n));
  if (Wire.available() < 2 * n) return false;
  for (byte i = 0; i < n; i++) {
    int16_t hi = Wire.read();
    int16_t lo = Wire.read();
    out[i] = (hi << 8) | lo;
  }
  return true;
}

bool gyroBegin() {
  Wire.begin();
  Wire.setClock(400000);
  Wire.setWireTimeout(3000, true);      // don't hang if a wire is loose

  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;

  writeReg(0x6B, 0x00);                 // wake up
  writeReg(0x1A, 0x03);                 // low-pass ~44 Hz (motor vibration)
  writeReg(0x1B, 0x08);                 // +-500 deg/s
  delay(100);
  lastRead = micros();
  return true;
}

void gyroCalibrate(unsigned long ms) {
  float gSum[3] = { 0, 0, 0 };
  float aSum[3] = { 0, 0, 0 };
  long n = 0;
  int16_t raw[7];                       // accel x,y,z, temp, gyro x,y,z
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (readRegs(0x3B, raw, 7)) {
      for (byte i = 0; i < 3; i++) {
        aSum[i] += raw[i];
        gSum[i] += raw[4 + i];
      }
      n++;
    }
    delay(2);
  }
  if (n > 0) {
    float len = sqrt(aSum[0] * aSum[0] + aSum[1] * aSum[1] + aSum[2] * aSum[2]);
    for (byte i = 0; i < 3; i++) {
      bias[i] = gSum[i] / n;
      if (len > 0) up[i] = aSum[i] / len;  // at rest the accel reads +1 g up
    }
  }
  angle = 0;
  rate = 0;
  lastRead = micros();
}

void gyroUpdate() {
  unsigned long now = micros();
  if (now - lastRead < READ_US) return;
  float dt = (now - lastRead) / 1000000.0;
  lastRead = now;

  int16_t g[3];
  if (!readRegs(0x43, g, 3)) return;    // keep the last rate
  float about = 0;                      // rate about "up", raw counts
  for (byte i = 0; i < 3; i++) about += (g[i] - bias[i]) * up[i];
  rate = GYRO_SIGN * about / LSB_PER_DPS;
  angle += rate * dt;
}

float gyroRate() {
  return rate;
}

float gyroAngle() {
  return angle;
}

void gyroResetAngle() {
  angle = 0;
}
