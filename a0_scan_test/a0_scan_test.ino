// ===== a0_scan_test - the robot measures A0 against the gyro angle =====
// Hang the robot, beacon 1 m away at A0's height, 12 V battery on.
// Power on and keep it STILL for 2 s (gyro calibration).
// Serial Monitor 115200, New Line. Commands:
//   a number, e.g. 3 or -2 : turn to that gyro angle (to aim the laser)
//   l : laser on/off (to see where it points; switch it OFF before g)
//   z : call this angle 0 (do it when the laser is on the bullseye)
//   g : scan -15 .. +15 deg in 1 deg steps, then back +15 .. -15,
//       then return to 0
//   s : stop (brake)
// At each step it turns, waits for steady READY, then logs 3 frames that
// started after that:  SCAN,dir,target,gyro,A0,A1,A2,A3
// At the end it prints the mean A0 per step and, for each direction, the
// first and last angle where A0 >= FIRE_LEVEL, and their middle.
// Library files are copies from main_see_fire (commit 81548ab).

#include "analog_sensor.h"
#include "motor.h"
#include "gyro.h"
#include "steady.h"

const int LASER_PIN = 12;
const int SCAN_FROM = -15;                     // deg
const int SCAN_TO = 15;                        // deg
const int STEPS = SCAN_TO - SCAN_FROM + 1;     // 31, 1 deg apart
const byte FRAMES_PER_STEP = 3;
const unsigned long FIRE_LEVEL = 119200;       // candidate fire threshold

enum Mode { IDLE, MOVING, SCANNING };
Mode mode = IDLE;
bool laserOn = false;

int dir = 1;                     // +1 = up (-15 -> +15), -1 = down
int stepIdx = 0;
byte framesLeft = 0;
bool waitingStill = false;       // a turn is running
unsigned long stillSinceMs = 0;  // only frames that start after this count
unsigned long sumA0 = 0;
float sumGyro = 0;

unsigned long meanA0[2][STEPS];  // [0] = up, [1] = down
float meanGyro[2][STEPS];

int stepTarget(int idx) {
  return dir == 1 ? SCAN_FROM + idx : SCAN_TO - idx;
}

void goTo(float angle) {
  steadyStop();
  steadyTurnBy(angle - gyroAngle());
  waitingStill = true;
}

void startStep() {
  goTo(stepTarget(stepIdx));
  framesLeft = FRAMES_PER_STEP;
  sumA0 = 0;
  sumGyro = 0;
}

void printSummary() {
  Serial.println(F("STEP,dir,target,gyro_mean,A0_mean"));
  for (int d = 0; d < 2; d++) {
    for (int i = 0; i < STEPS; i++) {
      Serial.print(F("STEP,"));
      Serial.print(d == 0 ? F("up,") : F("down,"));
      Serial.print(d == 0 ? SCAN_FROM + i : SCAN_TO - i);
      Serial.print(',');
      Serial.print(meanGyro[d][i], 1);
      Serial.print(',');
      Serial.println(meanA0[d][i]);
    }
  }
  for (int d = 0; d < 2; d++) {
    int first = -1, last = -1;
    for (int i = 0; i < STEPS; i++) {
      if (meanA0[d][i] >= FIRE_LEVEL) {
        if (first < 0) first = i;
        last = i;
      }
    }
    Serial.print(d == 0 ? F("up:   ") : F("down: "));
    if (first < 0) {
      Serial.println(F("A0 never reached FIRE_LEVEL"));
      continue;
    }
    float a = meanGyro[d][first], b = meanGyro[d][last];
    Serial.print(F("A0 >= FIRE_LEVEL from "));
    Serial.print(min(a, b), 1);
    Serial.print(F(" to "));
    Serial.print(max(a, b), 1);
    Serial.print(F(" deg, middle "));
    Serial.println((a + b) / 2, 1);
  }
}

void readSerial() {
  if (!Serial.available()) return;
  char c = Serial.peek();
  if (c == 'l') {
    Serial.read();
    laserOn = !laserOn;
    digitalWrite(LASER_PIN, laserOn ? HIGH : LOW);
    Serial.println(laserOn ? F("laser on") : F("laser off"));
  } else if (c == 'z') {
    Serial.read();
    gyroResetAngle();
    steadyStop();
    steadyStartAt(0);
    Serial.println(F("angle = 0"));
  } else if (c == 'g') {
    Serial.read();
    if (laserOn) Serial.println(F("note: laser is on"));
    Serial.println(F("scan start"));
    mode = SCANNING;
    dir = 1;
    stepIdx = 0;
    startStep();
  } else if (c == 's') {
    Serial.read();
    mode = IDLE;
    waitingStill = false;
    steadyStop();
    motorStop();
    Serial.println(F("stopped"));
  } else if (isDigit(c) || c == '-' || c == '+') {
    float a = Serial.parseFloat();
    Serial.print(F("going to "));
    Serial.println(a, 1);
    mode = MOVING;
    goTo(a);
  } else {
    Serial.read();                      // newlines etc.
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
  setupSensor();
  motorBegin();
  if (!gyroBegin()) {
    Serial.println(F("No gyro, check A4/A5"));
    while (true) {}
  }
  Serial.println(F("calibrating, keep still"));
  gyroCalibrate(2000);
  steadyStart();
  Serial.println(F("ready: angle, l = laser, z = zero, g = scan, s = stop"));
}

void loop() {
  motorUpdate();
  gyroUpdate();
  steadyUpdate();
  sensorsUpdate();
  readSerial();

  if (waitingStill && steadyReady()) {
    waitingStill = false;
    stillSinceMs = millis();
    if (mode == MOVING) {
      mode = IDLE;
      Serial.print(F("at "));
      Serial.println(gyroAngle(), 1);
    }
  }

  if (mode != SCANNING || waitingStill || !sensorsNewFrame()) return;
  const SensorFrame &f = sensorFrame();
  if (f.onStartMs < stillSinceMs) return;   // started before it was still

  Serial.print(F("SCAN,"));
  Serial.print(dir == 1 ? F("up,") : F("down,"));
  Serial.print(stepTarget(stepIdx));
  Serial.print(',');
  Serial.print(gyroAngle(), 1);
  for (int i = 0; i < sensorNum; i++) {
    Serial.print(',');
    Serial.print(f.strength[i]);
  }
  Serial.println();

  sumA0 += f.strength[0];
  sumGyro += gyroAngle();
  if (--framesLeft > 0) return;

  int d = dir == 1 ? 0 : 1;
  meanA0[d][stepIdx] = sumA0 / FRAMES_PER_STEP;
  meanGyro[d][stepIdx] = sumGyro / FRAMES_PER_STEP;

  if (++stepIdx < STEPS) {
    startStep();
  } else if (dir == 1) {
    dir = -1;                           // now back down, +15 -> -15
    stepIdx = 0;
    startStep();
  } else {
    printSummary();
    Serial.println(F("scan done, back to 0"));
    mode = MOVING;
    goTo(0);
  }
}
