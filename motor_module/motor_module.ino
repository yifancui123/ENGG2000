// ===== motor_module - test sketch for motor / gyro / steady =====
// Serial Monitor at 115200, line ending "New Line". Motor clamped or hanging.
//   90     -> turn 90 deg anticlockwise (left), -45 -> 45 deg clockwise
//             with the gyro + STEADY_ON: steadyTurnBy (turn, cancel drift,
//             hold the target) and it prints READY, then the error 2 s later
//             without the gyro: plain turnBy
//   s      -> stop (brake)
//   h-2    -> motorHold(-2) wheel keeps turning at a steady speed to cancel
//             a +2 deg/s body drift (h0 = hold the wheel still, actively)
// While a turn runs it logs:  ms,target,count,pwm[,rate,angle]  (.csv)
// At the end:  DONE / TIMEOUT / STOPPED, error = X counts (Y deg)
//
// With USE_GYRO = true (MPU6050 on A4/A5) also:
//   g      -> gyro live print on/off:  rate deg/s, angle deg
//   z      -> gyro angle = 0
//   c      -> recalibrate gyro (keep the satellite STILL for 2 s)
//   p      -> print the saved AUTO_TEST results (kept after power off)
//
// AUTO_90 / AUTO_45 = true: no Serial needed (battery, USB unplugged).
// After a 15 s countdown (LED blinks) it turns one full circle in steps
// (90: 4 turns, 45: 8 turns), 5 s pause after each, then the LED stays on.
//
// AUTO_SWEEP = true: finds the hold rate with no Serial. After the 15 s
// countdown, for each rate in SWEEP_RATES: LED blinks fast (run number:
// 1 blink = 1st rate, 2 blinks = 2nd ...), 3 s settle, turnBy(90),
// hold at that rate for 5 s, brake. Film the pointer and pick the run
// that moved least after the turn. LED stays on at the end.
//
// AUTO_TEST = true (needs the gyro): the full subsystem test, battery only.
// 15 s countdown, LED on 3 s = gyro calibration (keep STILL), then every
// turn in TEST_TURNS. For each turn, on the gyro:
//   STEADY_ON = true  -> steadyTurnBy (the "after" run)
//   STEADY_ON = false -> turnBy + brake only (the "before" run)
// then it watches a 2 s laser window. Everything is saved in EEPROM: plug
// USB back in, Serial Monitor, type p, paste the table.
// LED stays on at the end, fast blinking = no gyro found.
//
// AUTO_HOLD = true (needs the gyro): same start as AUTO_TEST, then ONE turn
// of HOLD_TURN deg and holds it for HOLD_MS, saving the error every 0.5 s.
// p prints it the same way. Shows if the hold drifts away over 10 s.
//
// Only ONE auto mode true at a time. All false = normal Serial test.

#include <EEPROM.h>
#include "motor.h"
#include "gyro.h"
#include "steady.h"

const bool AUTO_90 = false;
const bool AUTO_45 = false;
const bool AUTO_SWEEP = false;
const bool AUTO_TEST = false;
const bool AUTO_HOLD = true;
const bool USE_GYRO = true;
const bool STEADY_ON = true;
// After each AUTO_90/45 turn, hold at this rate (deg/s, minus the drift you
// saw) for the 5 s pause instead of braking. 0 = brake like before.
const float HOLD_RATE = 0;
const float SWEEP_RATES[] = { 0, -2, -4, -6, 2, 4, 6 };
const float TEST_TURNS[] = { 90, 90, -90, -90, 45, -45, 10, -10, 5, -5, 2, -2, 1, -1 };
const float HOLD_TURN = 90;       // AUTO_HOLD turn, deg
const unsigned long HOLD_MS = 10000;
const float BULLSEYE = 2.3;       // deg, 8 cm at 1 m
const int led = 13;

// one AUTO_TEST turn, saved in EEPROM
struct TurnLog {
  float cmd;        // commanded turn, deg
  float turned;     // body angle turned when the encoder turn ended (gyro), deg
  float drift;      // drift after the turn, deg/s
  float corr;       // correction steady made to reach the target, deg
  float err0;       // off target when READY (or when the turn ended), deg
  float err2s;      // off target 2 s later, deg
  float maxErr;     // worst |off target| during those 2 s, deg
  float tReady;     // command -> READY (or turn end), s; -1 = never READY
};
const int TEST_N = sizeof(TEST_TURNS) / sizeof(TEST_TURNS[0]);
const int LOG_ADDR = 0;           // byte 0: steady on/off, byte 1: count, then logs
const int CURVE_ADDR = 512;       // AUTO_HOLD: byte count, then error every 0.5 s
const int CURVE_MAX = 60;         // up to 30 s

bool gyroOk = false;

static void countdown() {
  for (int i = 0; i < 30; i++) {            // 15 s countdown
    digitalWrite(led, i % 2 == 0);
    delay(500);
  }
  digitalWrite(led, LOW);
}

static void holdFor(float rate, unsigned long ms) {
  motorHold(rate);
  unsigned long t0 = millis();
  while (millis() - t0 < ms) motorUpdate();
  motorStop();
}

static void updateAll() {
  motorUpdate();
  if (gyroOk) gyroUpdate();
  steadyUpdate();
}

static void waitFor(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) updateAll();
}

// countdown, calibrate, clear the logs (AUTO_TEST and AUTO_HOLD)
static void startTest() {
  if (!gyroOk) {
    while (true) {                          // no gyro: blink fast forever
      digitalWrite(led, !digitalRead(led));
      delay(100);
    }
  }
  countdown();
  digitalWrite(led, HIGH);                  // LED on = calibrating, STILL
  gyroCalibrate(3000);
  digitalWrite(led, LOW);

  EEPROM.update(LOG_ADDR, STEADY_ON ? 1 : 0);
  EEPROM.update(LOG_ADDR + 1, 0);
  EEPROM.update(CURVE_ADDR, 0);
}

// one turn up to READY (or the turn end), fills the first log fields,
// returns the target gyro angle
static float turnAndLog(TurnLog &log) {
  float start = gyroAngle();
  float target = start + log.cmd;
  unsigned long t0 = millis();

  if (STEADY_ON) {
    steadyTurnBy(log.cmd);
    while (steadyState() == STEADY_TURNING) updateAll();
    log.turned = gyroAngle() - start;
    while (!steadyReady() && millis() - t0 < 6000) updateAll();
    if (steadyReady()) log.tReady = (millis() - t0) / 1000.0;
    log.drift = steadyDrift();
    log.corr = steadyCorrection();
  } else {
    turnBy(log.cmd);
    while (motorBusy()) updateAll();
    log.turned = gyroAngle() - start;
    log.tReady = (millis() - t0) / 1000.0;
  }
  log.err0 = gyroAngle() - target;
  return target;
}

static void runHoldTest() {
  startTest();
  TurnLog log = { HOLD_TURN, 0, 0, 0, 0, 0, 0, -1 };
  float target = turnAndLog(log);

  // hold for HOLD_MS, error saved every 0.5 s
  float a0 = gyroAngle();
  unsigned long w0 = millis();
  unsigned long nextSample = 500;
  byte n = 0;
  while (millis() - w0 < HOLD_MS) {
    updateAll();
    float off = gyroAngle() - target;
    if (fabs(off) > log.maxErr) log.maxErr = fabs(off);
    if (millis() - w0 >= 2000 && log.err2s == 0) log.err2s = off;
    if (millis() - w0 >= nextSample && n < CURVE_MAX) {
      EEPROM.put(CURVE_ADDR + 1 + n * sizeof(float), off);
      n++;
      nextSample += 500;
    }
  }
  if (!STEADY_ON) log.drift = (gyroAngle() - a0) / (HOLD_MS / 1000.0);

  EEPROM.put(LOG_ADDR + 2, log);
  EEPROM.update(LOG_ADDR + 1, 1);
  EEPROM.update(CURVE_ADDR, n);

  steadyStop();
  motorStop();
  digitalWrite(led, HIGH);
}

static void runFullTest() {
  startTest();

  for (int i = 0; i < TEST_N; i++) {
    TurnLog log = { TEST_TURNS[i], 0, 0, 0, 0, 0, 0, -1 };
    float target = turnAndLog(log);

    // the 2 s laser window
    float a0 = gyroAngle();
    unsigned long w0 = millis();
    while (millis() - w0 < 2000) {
      updateAll();
      float off = fabs(gyroAngle() - target);
      if (off > log.maxErr) log.maxErr = off;
    }
    log.err2s = gyroAngle() - target;
    if (!STEADY_ON) log.drift = (gyroAngle() - a0) / 2.0;

    EEPROM.put(LOG_ADDR + 2 + i * sizeof(TurnLog), log);
    EEPROM.update(LOG_ADDR + 1, i + 1);

    waitFor(2000);                          // rest before the next turn
    steadyStop();
  }
  motorStop();
  digitalWrite(led, HIGH);
}

static void printTestLog() {
  int n = EEPROM.read(LOG_ADDR + 1);
  if (n > TEST_N) n = 0;
  Serial.print("AUTO_TEST results, steady ");
  Serial.print(EEPROM.read(LOG_ADDR) == 1 ? "ON" : "OFF (brake only)");
  Serial.print(", ");
  Serial.print(n);
  Serial.println(" turns");
  Serial.println("n,cmd,turned,drift_dps,corr,err0,err2s,maxErr,tReady_s");

  float sumAbs2s = 0;
  float sumTurnErr = 0;
  int inBull = 0;
  for (int i = 0; i < n; i++) {
    TurnLog log;
    EEPROM.get(LOG_ADDR + 2 + i * sizeof(TurnLog), log);
    Serial.print(i + 1);
    float v[8] = { log.cmd, log.turned, log.drift, log.corr,
                   log.err0, log.err2s, log.maxErr, log.tReady };
    for (int k = 0; k < 8; k++) {
      Serial.print(",");
      Serial.print(v[k], 2);
    }
    Serial.println();
    sumAbs2s += fabs(log.err2s);
    sumTurnErr += fabs(log.turned - log.cmd);
    if (log.maxErr <= BULLSEYE) inBull++;
  }
  if (n > 0) {
    Serial.print("mean |turn error| (encoder turn) = ");
    Serial.print(sumTurnErr / n, 2);
    Serial.println(" deg");
    Serial.print("mean |err after 2 s| = ");
    Serial.print(sumAbs2s / n, 2);
    Serial.println(" deg");
    Serial.print("inside +-2.3 deg for the whole 2 s: ");
    Serial.print(inBull);
    Serial.print(" of ");
    Serial.println(n);
  }

  int c = EEPROM.read(CURVE_ADDR);
  if (c > 0 && c <= CURVE_MAX) {
    Serial.println("AUTO_HOLD: off target during the hold (maxErr = whole hold)");
    Serial.println("t_s,err_deg");
    for (int i = 0; i < c; i++) {
      float off;
      EEPROM.get(CURVE_ADDR + 1 + i * sizeof(float), off);
      Serial.print((i + 1) * 0.5, 1);
      Serial.print(",");
      Serial.println(off, 2);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  pinMode(led, OUTPUT);
  motorBegin();

  if (USE_GYRO || AUTO_TEST || AUTO_HOLD) {
    gyroOk = gyroBegin();
    if (gyroOk && !AUTO_TEST && !AUTO_HOLD) gyroCalibrate(2000);
  }

  if (AUTO_TEST) {
    runFullTest();
  } else if (AUTO_HOLD) {
    runHoldTest();
  } else if (AUTO_SWEEP) {
    countdown();
    int runs = sizeof(SWEEP_RATES) / sizeof(SWEEP_RATES[0]);
    for (int run = 0; run < runs; run++) {
      for (int b = 0; b <= run; b++) {      // run number: run+1 fast blinks
        digitalWrite(led, HIGH);
        delay(150);
        digitalWrite(led, LOW);
        delay(250);
      }
      delay(3000);                          // let it settle
      turnBy(90);
      while (motorBusy()) motorUpdate();
      if (SWEEP_RATES[run] != 0) {
        holdFor(SWEEP_RATES[run], 5000);
      } else {
        delay(5000);
      }
    }
    digitalWrite(led, HIGH);
  } else if (AUTO_90 || AUTO_45) {
    float step = AUTO_90 ? 90 : 45;
    int turns = 360 / step;                 // one full circle
    countdown();
    for (int turn = 0; turn < turns; turn++) {
      turnBy(step);
      while (motorBusy()) motorUpdate();
      if (HOLD_RATE != 0) {
        holdFor(HOLD_RATE, 5000);
      } else {
        delay(5000);
      }
    }
    digitalWrite(led, HIGH);
  }

  if (USE_GYRO || AUTO_TEST || AUTO_HOLD) {
    Serial.println(gyroOk ? "gyro OK" : "gyro NOT found (check A4/A5)");
  }
  Serial.println("ready: type degrees (e.g. 90, -45) or s");
}

void loop() {
  updateAll();

  static bool gyroPrint = false;

  if (Serial.available()) {
    char c = Serial.peek();
    if (c == 's') {
      Serial.read();
      steadyStop();
      motorStop();
    } else if (c == 'h') {
      Serial.read();
      float rate = Serial.parseFloat();
      Serial.print("hold ");
      Serial.print(rate);
      Serial.println(" deg/s (s to stop)");
      steadyStop();
      motorHold(rate);
    } else if (c == 'g') {
      Serial.read();
      gyroPrint = !gyroPrint;
    } else if (c == 'z') {
      Serial.read();
      gyroResetAngle();
    } else if (c == 'c') {
      Serial.read();
      if (gyroOk) {
        Serial.println("calibrating, keep still...");
        gyroCalibrate(2000);
        Serial.println("done");
      }
    } else if (c == 'p') {
      Serial.read();
      printTestLog();
    } else if (isDigit(c) || c == '-') {
      float deg = Serial.parseFloat();
      steadyStop();
      if (gyroOk && STEADY_ON) {
        Serial.print("steadyTurnBy ");
        Serial.println(deg);
        steadyTurnBy(deg);
      } else {
        Serial.print("turnBy ");
        Serial.println(deg);
        turnBy(deg);
      }
    } else {
      Serial.read();
    }
  }

  // log while moving
  static unsigned long lastPrint = 0;
  if ((motorBusy() || motorHolding()) && millis() - lastPrint >= 10) {
    lastPrint = millis();
    Serial.print(millis());
    Serial.print(",");
    Serial.print(motorTarget());
    Serial.print(",");
    Serial.print(motorCount());
    Serial.print(",");
    Serial.print(motorPwm());
    if (gyroOk) {
      Serial.print(",");
      Serial.print(gyroRate(), 2);
      Serial.print(",");
      Serial.print(gyroAngle(), 2);
    }
    Serial.println();
  }

  // gyro live print
  static unsigned long lastGyroPrint = 0;
  if (gyroPrint && gyroOk && !motorBusy() && !motorHolding() &&
      millis() - lastGyroPrint >= 50) {
    lastGyroPrint = millis();
    Serial.print(gyroRate(), 2);
    Serial.print(" deg/s, ");
    Serial.print(gyroAngle(), 2);
    Serial.println(" deg");
  }

  // report once when a move ends
  static MotorResult lastResult = MOTOR_IDLE;
  MotorResult r = motorResult();
  if (r != lastResult) {
    lastResult = r;
    if (r != MOTOR_BUSY && r != MOTOR_HOLDING) {
      long err = motorTarget() - motorCount();
      if (r == MOTOR_DONE) Serial.print("DONE");
      if (r == MOTOR_TIMEOUT) Serial.print("TIMEOUT");
      if (r == MOTOR_STOPPED) Serial.print("STOPPED");
      Serial.print(", error = ");
      Serial.print(err);
      Serial.print(" counts (");
      Serial.print(err / 55.6, 2);
      Serial.println(" deg of body)");
    }
  }

  // steady hold: report READY, then how far off it is 2 s later
  static bool wasReady = false;
  static unsigned long readyAt = 0;
  static float maxOff = 0;
  static bool reported2s = false;
  if (steadyReady()) {
    float off = gyroAngle() - steadyHoldAngle();
    if (!wasReady) {
      readyAt = millis();
      maxOff = 0;
      reported2s = false;
      Serial.print("READY: drift ");
      Serial.print(steadyDrift(), 2);
      Serial.print(" deg/s cancelled, corrected ");
      Serial.print(steadyCorrection(), 2);
      Serial.print(" deg, now off by ");
      Serial.print(off, 2);
      Serial.println(" deg");
    }
    if (fabs(off) > maxOff) maxOff = fabs(off);
    if (!reported2s && millis() - readyAt >= 2000) {
      reported2s = true;
      Serial.print("2 s later: off by ");
      Serial.print(off, 2);
      Serial.print(" deg, worst ");
      Serial.print(maxOff, 2);
      Serial.println(maxOff <= BULLSEYE ? " deg -> inside bullseye" : " deg -> OUTSIDE bullseye");
    }
  }
  wasReady = steadyReady();
}
