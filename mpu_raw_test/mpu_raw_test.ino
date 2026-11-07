// MPU6050 raw test - not part of the project
#include <Wire.h>
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x80);   // full reset
  Wire.endTransmission();
  delay(200);                                                          // wait for reset
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x01);   // wake, gyro clock
  Serial.print(F("wake: ")); Serial.println(Wire.endTransmission());  // 0 = OK
  delay(100);                                                          // let sensors start
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.endTransmission(false);  // read back
  Wire.requestFrom(0x68, 2);
  Serial.print(F("PWR1: 0x")); Serial.print(Wire.read(), HEX);        // should be 0x1
  Serial.print(F("  PWR2: 0x")); Serial.println(Wire.read(), HEX);    // should be 0x0
  Wire.beginTransmission(0x68); Wire.write(0x75); Wire.endTransmission(false);
  Wire.requestFrom(0x68, 1);
  Serial.print(F("WHO_AM_I: 0x")); Serial.println(Wire.read(), HEX);  // 0x68 = real MPU6050
}
void loop() {
  Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission(false);
  byte n = Wire.requestFrom(0x68, 14);
  int16_t v[7];
  for (byte i = 0; i < 7; i++) { int16_t hi = Wire.read(); int16_t lo = Wire.read(); v[i] = (hi << 8) | lo; }
  Serial.print(F("bytes ")); Serial.print(n);
  Serial.print(F("  acc ")); Serial.print(v[0]); Serial.print(' '); Serial.print(v[1]); Serial.print(' '); Serial.print(v[2]);
  Serial.print(F("  gyro ")); Serial.print(v[4]); Serial.print(' '); Serial.print(v[5]); Serial.print(' '); Serial.println(v[6]);
  delay(200);
}
