#include "ir_sensor.h"
#include "motor_control.h"

//const unsigned long laserTimer = 2000;

//laser pin
const int LASER_PIN = 12;  // change to whatever pin is actually connected

//ir receiver
const int SENSOR_PIN = 8;

//Spin Speed
const int SLOW_SPEED = 80;
const int SCAN_SPEED = 150;

//How long the laser stays ON once the beacon triggers it (5s for now)
const unsigned long LASER_ON_MS = 5000;

//when the laser was turned on
unsigned long laserStart = 0;

//laser status (initially off)
bool laserOn = false;

//last time we print "searching beacon"
unsigned long lastSearching = 0;

//---------------------------------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  pinMode(LASER_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT_PULLUP);

  setupMotor();  // handles motorPWM/motorDirection/encoder pinMode + attachInterrupt, defined in motor_control.cpp

  digitalWrite(LASER_PIN, LOW);
  digitalWrite(LED_BUILTIN, LOW);

  //wait 30 seconds after power-up before spinning (with countdown)
  for (int s = 30; s > 0; s--) {
    Serial.print("Starting in ");
    Serial.print(s);
    Serial.println(" s...");
    delay(1000);
  }

  //start spinning slowly in ONE direction and never stop
  motorForward(SCAN_SPEED);
}
//---------------------------------------------------------------------------------------------------------
void loop() {
  bool seeBeacon = sawBeacon(SENSOR_PIN);

  //fast speed while searching; slow speed after detecting beacon
  if (laserOn){
    motorForward(SLOW_SPEED);
  } else {
    motorForward(SCAN_SPEED);
  }

  if (seeBeacon && !laserOn) {
    digitalWrite(LASER_PIN, HIGH);
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println("Beacon found - laser ON");
    laserOn = true;
    laserStart = millis();
  }

  //turn the laser OFF after 5s
  if (laserOn && millis() - laserStart >= LASER_ON_MS) {
    digitalWrite(LASER_PIN, LOW);
    digitalWrite(LED_BUILTIN, LOW);
    Serial.println("Timer expired - laser OFF");
    laserOn = false;
  }

   //while the laser is off, print a searching message once per second
  if (!laserOn && millis() - lastSearching >= 1000) {
    Serial.println("Searching beacon...");
    lastSearching = millis();
  }
}
