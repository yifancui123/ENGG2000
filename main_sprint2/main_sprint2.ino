// MQ Sentinel - Sprint 2 main.ino
// Loop: motor burst -> stop -> read the sensor -> fire if the rule says so.

#include "analog_sensor.h"
#include "motor_control.h"

//---------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  setupMotor();   // motor pins, driver enable, encoder interrupt
  setupSensor();  // power pin + sensor pins

  delay(STARTUP_DELAY_MS);
}

//---------------------------------------------------------------------------------------------------------
void loop() {
    motorForward(100);
    delay(100);
    motorStop();
    delay(100);
}
