
#include "ir_sensor.h"
#include "motor_control.h"

const unsigned long laserTimer = 2000;

// Set to true when testing A3, A4 and A5.
// Set to false to run the original satellite program.
const bool SENSOR_TEST_MODE = true;

// Laser pin
const int LASER_PIN = 12;

// Original IR receivers
int sensorPins[4] = {8, 9, 10, 11};
int sensorLength = 4;

// Directional sensor test function
void testDirectionalSensors();

enum State {
  scanning,
  aligning,
  firing
};

State laserState = scanning;

// --------------------------------------------------
void setup() {
  Serial.begin(9600);

  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);

  pinMode(LED_BUILTIN, OUTPUT);

  // New directional sensors
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);

  // Original IR receivers
  for (int i = 0; i < sensorLength; i++) {
    pinMode(sensorPins[i], INPUT_PULLUP);
  }

  // Don't activate the motor during sensor testing
  if (!SENSOR_TEST_MODE) {
    setupMotor();
  }

  Serial.println("MQ Sentinel started");

  if (SENSOR_TEST_MODE) {
    Serial.println("A3-A5 sensor test mode");
  }
}

// --------------------------------------------------
void loop() {

  // Test sensors without rotating or firing
  if (SENSOR_TEST_MODE) {
    testDirectionalSensors();
    delay(200);
    return;
  }

  // Original satellite operation
  switch (laserState) {

    case scanning:
      handleScanning();
      break;

    case aligning:
      handleAligning();
      break;

    case firing:
      handleFiring();
      break;
  }
}

// --------------------------------------------------
void fireLaser() {
  digitalWrite(LASER_PIN, HIGH);
  delay(laserTimer);
  digitalWrite(LASER_PIN, LOW);
}

// --------------------------------------------------
void handleScanning() {
  int hitSensor = -1;

  for (int i = 0; i < sensorLength; i++) {
    if (sawBeacon(sensorPins[i])) {
      hitSensor = i;
      break;
    }
  }

  if (hitSensor == -1) {
    Serial.println("No beacon - keep scanning");
    rotateTo(45);
  } else {
    beaconDirection(hitSensor);
    laserState = aligning;
  }
}

// --------------------------------------------------
void beaconDirection(int hitSensor) {

  if (hitSensor == 0) {
    Serial.println("Beacon in FRONT - fire laser!");
  }
  else if (hitSensor == 1) {
    Serial.println("Beacon on RIGHT - rotate right");
    rotateTo(-90);
  }
  else if (hitSensor == 2) {
    Serial.println("Beacon BEHIND - rotate around");
    rotateTo(180);
  }
  else if (hitSensor == 3) {
    Serial.println("Beacon on LEFT - rotate left");
    rotateTo(90);
  }
}

// --------------------------------------------------
void handleAligning() {
  // Alignment logic not implemented yet
  laserState = firing;
}

// --------------------------------------------------
void handleFiring() {
  fireLaser();
  laserState = scanning;
}
