// ===== MQ Sentinel - Analog(TSSP) sensor + motor drive, combined =====
// Sensor code = the team's "working analog sensor" module (analog_sensor.cpp/.h),
// pins UNCHANGED: power-cycle TSSP, shared power D8, OUT A0 (sensorNum=1), 25/25.
// Motor code = motor_control.cpp/.h (PWM D5, DIR D6, encoder D2/D3, sleep D7).
//
// No pin conflicts: sensor uses D8 + A0; motor uses D2/D3/D5/D6/D7.
//
// This base reads the sensor strengths and prints them with the encoder position,
// while (optionally) spinning the motor so you can watch the reading change as
// the body rotates. Build your aim logic on top of this.

#include "analog_sensor.h"   // setupSensor(), readStrengths(), (power D8 / OUT A0)
#include "motor_control.h"   // motorForward/Reverse/Stop, setupMotor(), pos

const int SCAN_SPEED = 80;   // gentle spin speed (set 0 / comment out to stay still)

void setup() {
  Serial.begin(115200);
  setupSensor();   // TSSP power + OUT pins (from analog_sensor.cpp — pins unchanged)
  setupMotor();    // motor pins + encoder interrupt
}

void loop() {
  // --- SENSOR: read all strengths (index 0=front..3=left; unwired slots = 0) ---
  unsigned long s[4];
  readStrengths(s);

  // --- MOTOR: gentle scan spin so sensor + motor run together.
  //     Comment this line out if you want the body to stay still. ---
  motorForward(SCAN_SPEED);

  // --- REPORT: strengths + encoder position ---
  Serial.print("Front:");   Serial.print(s[0]);
  Serial.print("\tRight:"); Serial.print(s[1]);
  Serial.print("\tBack:");  Serial.print(s[2]);
  Serial.print("\tLeft:");  Serial.print(s[3]);
  Serial.print("\tpos:");   Serial.println(pos);
}
