// ===== gyro_pid_fire - fast multi-target search + shoot, smooth gyro PID =====
// One file, nothing else needed.
//
// Sensors (90 deg apart, all VS on D4): A0 FRONT (in the tube), A1 RIGHT,
// A2 BACK, A3 LEFT. Every reading is used straight away, also while turning.
//
// Every reading, in this order:
//   1. A0 strength >= FIRE_US        -> FIRE: freeze where it is, laser ON
//   2. while firing, A0 drops below FIRE_OFF_US -> laser OFF, SWEEP around here
//   3. A0 >= NEAR_US (beacon near the front) -> never GOTO; SCAN starts a SWEEP here
//   4. a side sensor sees one (>= SIDE_MIN_US) -> GOTO: turn so A0 faces it
//      (the strongest side if several; worked out from where the body really is)
//   5. A0 sees something weak        -> slow the search down so the tube can't skip it
// States:
//   SCAN   turn slowly one way (reverses at +-MAX_ANGLE) - looking for beacons
//   GOTO   turning to where a side sensor saw one; then SWEEP there
//   SWEEP  swing +-SWEEP_HALF around a spot (tube = narrow view); nothing -> SCAN
//   FIRE   laser on while A0 stays strong; the PID keeps it pointing there
// Beacons turn on and off (up to 15): it never stops looking.
//
// Control (no braking, no pulses): gyro PID every 5 ms, angle error -> body
// speed (+ feed-forward of the moving target), the wheel driven at the speed
// that gives that body speed (encoder PD follows a moving wheel target).
//
// Wiring: 4 x TSSP VS -> D4, OUT front A0, right A1, back A2, left A3.
//   laser D12. motor: encoder D2/D3, EN D5, PH D6, nSLEEP D7, PMODE -> GND.
//   gyro MPU6050: SDA A4, SCL A5.
// Power on HANGING and STILL (3 s hands off, LED on 2 s = gyro calibration).
// SENSOR_TEST = true: sensors + laser only (motor off), prints every reading.

#include <Wire.h>

const bool SENSOR_TEST = false;
const bool DEBUG = true;

// ---------------- pins ----------------
const int motorPWM = 5;          // driver EN
const int motorDirection = 6;    // driver PH
const int sleepPin = 7;          // driver nSLEEP
const int encoderA = 2;
const int encoderB = 3;
const int SENSOR_POWER = 4;
const int LASER_PIN = 12;
const int LED = 13;
enum SensorId { FRONT, RIGHT, BACK, LEFT, N_SENSORS };
const uint8_t SENSOR_PIN[N_SENSORS] = { A0, A1, A2, A3 };

// ---------------- gyro PID (angle -> body speed) ----------------
const float ANGLE_KP = 2.5;      // deg/s of body speed per deg of error
const float ANGLE_KI = 0.4;      // slowly cancels drift / string twist
const float ANGLE_KD = 0.25;     // damping from the gyro rate (stops overshoot)
const float MAX_RATE = 45;       // deg/s, top body speed
const float MAX_I = 15;          // deg/s, cap on the integral part
const float I_ZONE = 10;         // deg, only integrate this close to the target (no windup on big turns)
const int GYRO_SIGN = 1;         // + must be anticlockwise; if it runs away, set -1

// ---------------- wheel speed (encoder PD follows a moving target) ----------------
const float R_RATIO = 0.14;                          // body angle / wheel angle
const float COUNTS_PER_DEG = 2800.0 / 360.0 / R_RATIO;   // ~55.6 counts per body degree
const int TURN_SIGN = -1;        // + wheel counts turn the body clockwise
const float wheelKp = 1.0;
const float wheelKd = 0.05;
const int MAX_PWM = 150;
const unsigned long LOOP_US = 5000;   // 5 ms control loop

// ---------------- search ----------------
const float SCAN_RATE = 20;      // deg/s, normal search speed
const float SLOW_RATE = 6;       // deg/s, when A0 sees something weak
const float SWEEP_HALF = 45;     // deg either side of the spot (a side sensor only says +-45)
const float SWEEP_RATE = 15;     // deg/s
const byte SWEEPS = 1;           // swings before going back to SCAN: +45 then -45 = 135 deg, 9 s
                                 // (the next beacon is usually somewhere else: don't linger)
const float ARRIVED_DEG = 10;    // GOTO close enough -> SWEEP
const float PINGPONG_DEG = 45;   // GOTO this far (not 90) when the last GOTO came from the other side
const float GOTO_RATE = 40;      // deg/s, the GOTO target moves to the goal this fast
                                 // (a jump would trip the runaway check: 180 deg > RUNAWAY_DEG)
const float MAX_ANGLE = 360;     // deg either way, so the wire can't wind up

// ---------------- sensors and firing ----------------
const unsigned long OFF_MS = 300;         // sensors off (resets AGC). 380 if readings get weak
const unsigned long ON_US = 120000UL;     // max detection window
const unsigned long NO_SIGNAL_US = 30000; // not LOW by now = not seeing it (ends the window early)
const unsigned long SIDE_MIN_US = 2000;   // side strength = sees one (blips ~1200)
const unsigned long SEEN_US = 2000;       // A0 sees something (weak)
const unsigned long FIRE_US = 40000;      // A0 strength to shoot (max 120000)
const unsigned long FIRE_OFF_US = 25000;  // stop shooting below this (hysteresis)
// A0 at or above this: the beacon is near the front (8 Nov: A0 falls about 6,000 per deg
// off its 6 deg flat top, so 20,000 is roughly +-20 deg). The side sensors leak 85-107k
// there and would win step 3, turning 90 deg away: sweep around here instead.
const unsigned long NEAR_US = 20000;
const float RUNAWAY_DEG = 150;            // this far off the target = something's wrong: stop

// ---------------- gyro ----------------
const byte MPU_ADDR = 0x68;
const float LSB_PER_DPS = 65.5;

// ================= state =================
enum State { SCAN, GOTO, SWEEP, FIRE };
State state = SCAN;

volatile long pos = 0;
volatile byte lastAB = 0;
const int8_t QDEC[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };

float gyroBias[3] = { 0, 0, 0 };
float gyroUp[3] = { 0, 0, 1 };
float bodyAngle = 0;             // deg, + = anticlockwise
float bodyRate = 0;              // deg/s

float targetAngle = 0;           // where the body should point, deg
float targetRate = 0;            // how fast the target moves (scan / sweep), deg/s
float gotoGoal = 0;              // GOTO: where the target is heading, deg
int lastGotoSide = -1;           // side sensor that started the last GOTO, -1 = none since FIRE / SCAN
float integral = 0;
float wheelTarget = 0;
long lastCount = 0;
unsigned long lastLoop = 0;
bool running = false;

int scanDir = 1;
float scanRate = SCAN_RATE;
float sweepCentre = 0;
int sweepDir = 1;
byte sweepEnds = 0;

bool sensorsOn = false;
unsigned long sensorPhaseMs = 0;
unsigned long sensorOnUs = 0;
bool pulseLow[N_SENSORS];
bool pulseEnded[N_SENSORS];
unsigned long lowUs[N_SENSORS];
unsigned long highUs[N_SENSORS];
unsigned long strength[N_SENSORS];
bool frameReady = false;
float frameAngle = 0;            // body angle when the reading was taken

bool laserOn = false;

// ================= motor =================
void encoderISR() {
  int a = digitalRead(encoderA);
  int b = digitalRead(encoderB);
  int ab = a << 1 | b;
  pos += QDEC[(lastAB << 2) | ab];
  lastAB = ab;
}

long getCount() {
  noInterrupts();
  long c = pos;
  interrupts();
  return c;
}

void motorOff() {
  analogWrite(motorPWM, 0);
  digitalWrite(motorDirection, LOW);
}

void drive(int u) {
  if (u == 0) {
    motorOff();
    return;
  }
  digitalWrite(motorDirection, u > 0 ? HIGH : LOW);
  analogWrite(motorPWM, abs(u));
}

// ================= gyro =================
void writeReg(byte reg, byte value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool readRegs(byte reg, int16_t *out, byte n) {
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
  Wire.setWireTimeout(3000, true);
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;
  writeReg(0x6B, 0x00);
  writeReg(0x1A, 0x03);
  writeReg(0x1B, 0x08);
  delay(100);
  return true;
}

void gyroCalibrate(unsigned long ms) {
  float g[3] = { 0, 0, 0 };
  float a[3] = { 0, 0, 0 };
  long n = 0;
  int16_t raw[7];
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    if (readRegs(0x3B, raw, 7)) {
      for (byte i = 0; i < 3; i++) {
        a[i] += raw[i];
        g[i] += raw[4 + i];
      }
      n++;
    }
    delay(2);
  }
  if (n > 0) {
    float len = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    for (byte i = 0; i < 3; i++) {
      gyroBias[i] = g[i] / n;
      if (len > 0) gyroUp[i] = a[i] / len;
    }
  }
  bodyAngle = 0;
}

void gyroRead(float dt) {
  int16_t g[3];
  if (!readRegs(0x43, g, 3)) return;
  float about = 0;
  for (byte i = 0; i < 3; i++) about += (g[i] - gyroBias[i]) * gyroUp[i];
  bodyRate = GYRO_SIGN * about / LSB_PER_DPS;
  bodyAngle += bodyRate * dt;
}

// ================= moving the target (scan / sweep) =================
void moveTarget(float dt) {
  if (state == SCAN) {
    if (scanDir * targetAngle >= MAX_ANGLE) scanDir = -scanDir;   // don't wind the wire up
    targetRate = scanDir * scanRate;
  } else if (state == SWEEP) {
    float off = targetAngle - sweepCentre;
    if (sweepDir * off >= SWEEP_HALF) {
      sweepDir = -sweepDir;
      if (++sweepEnds >= 2 * SWEEPS) {          // swept it, nothing: back to the search
        setState(SCAN);
        return;
      }
    }
    targetRate = sweepDir * SWEEP_RATE;
  } else if (state == GOTO) {
    // ramp the target to the goal so the error stays small (a jump trips RUNAWAY_DEG)
    float left = gotoGoal - targetAngle;
    if (fabs(left) <= GOTO_RATE * dt) {
      targetAngle = gotoGoal;
      targetRate = 0;
      return;
    }
    targetRate = left > 0 ? GOTO_RATE : -GOTO_RATE;
  } else {
    targetRate = 0;                            // FIRE: fixed target
  }
  targetAngle += targetRate * dt;
}

// ================= the control loop (every 5 ms) =================
void control() {
  unsigned long now = micros();
  if (now - lastLoop < LOOP_US) return;
  float dt = (now - lastLoop) / 1000000.0;
  lastLoop = now;

  gyroRead(dt);
  if (!running) return;
  moveTarget(dt);

  // 1. gyro PID: how fast the body should turn (+ feed-forward of a moving target)
  float err = targetAngle - bodyAngle;
  if (fabs(err) > RUNAWAY_DEG) {               // wrong GYRO_SIGN or a dead gyro: stop safely
    running = false;
    motorOff();
    digitalWrite(LASER_PIN, LOW);
    Serial.println(F("STOPPED: body went the wrong way - check GYRO_SIGN / gyro wiring"));
    return;
  }
  if (fabs(err) < I_ZONE) integral = constrain(integral + ANGLE_KI * err * dt, -MAX_I, MAX_I);
  float want = targetRate + ANGLE_KP * err + integral - ANGLE_KD * (bodyRate - targetRate);
  want = constrain(want, -MAX_RATE, MAX_RATE);

  // 2. wheel speed that turns the body at that rate (follow a moving wheel target)
  float wheelSpeed = TURN_SIGN * want * COUNTS_PER_DEG;    // counts/s
  wheelTarget += wheelSpeed * dt;
  long count = getCount();
  float speed = (count - lastCount) / dt;
  lastCount = count;
  float lag = wheelTarget - count;
  if (fabs(lag) > 400) {                         // wheel can't keep up: don't wind up
    wheelTarget = count + (lag > 0 ? 400 : -400);
    lag = wheelTarget - count;
  }
  float u = wheelKp * lag + wheelKd * (wheelSpeed - speed);
  drive((int)constrain(u, -MAX_PWM, MAX_PWM));

  // arrived: the ramp has reached the goal and the body is close to it
  if (state == GOTO && targetAngle == gotoGoal && fabs(err) < ARRIVED_DEG) startSweep(targetAngle, 1);
}

// ================= states =================
void setState(State s) {
  static const char *const NAMES[] = { "SCAN", "GOTO", "SWEEP", "FIRE" };
  if (s == state) return;
  state = s;
  if (s == SCAN) scanRate = SCAN_RATE;
  if (s == SCAN || s == FIRE) lastGotoSide = -1;
  if (DEBUG) Serial.println(NAMES[s]);
}

void startSweep(float centre, int dir) {   // dir: first swing +1 anticlockwise / -1
  sweepCentre = centre;
  sweepDir = dir;
  sweepEnds = 0;
  setState(SWEEP);
}

void goToSide(float offset) {
  float t = frameAngle + offset;                 // from where the body really was
  if (t > MAX_ANGLE) t -= 360;                   // same heading, other way round
  if (t < -MAX_ANGLE) t += 360;
  gotoGoal = t;                                  // the target ramps there in moveTarget()
  setState(GOTO);
  if (DEBUG) {
    Serial.print(F("  -> go to "));
    Serial.println(t, 1);
  }
}

void laser(bool on) {
  if (on != laserOn && DEBUG) Serial.println(on ? F("LASER ON") : F("LASER OFF"));
  laserOn = on;
  digitalWrite(LASER_PIN, on ? HIGH : LOW);
}

// ================= sensors (fast: an empty window ends early) =================
void sensorsUpdate() {
  if (!sensorsOn) {
    if (millis() - sensorPhaseMs < OFF_MS) return;
    for (byte i = 0; i < N_SENSORS; i++) {
      pulseLow[i] = false;
      pulseEnded[i] = false;
    }
    digitalWrite(SENSOR_POWER, HIGH);
    for (byte i = 0; i < N_SENSORS; i++) pinMode(SENSOR_PIN[i], INPUT_PULLUP);
    sensorsOn = true;
    sensorOnUs = micros();
    return;
  }
  unsigned long now = micros();
  unsigned long t = now - sensorOnUs;
  bool allDone = true;
  for (byte i = 0; i < N_SENSORS; i++) {
    if (pulseEnded[i]) continue;
    bool low = digitalRead(SENSOR_PIN[i]) == LOW;
    if (low && !pulseLow[i]) {
      pulseLow[i] = true;
      lowUs[i] = now;
    } else if (!low && pulseLow[i]) {
      pulseEnded[i] = true;
      highUs[i] = now;
    }
    bool decided = pulseEnded[i] || (!pulseLow[i] && t > NO_SIGNAL_US);
    if (!decided) allDone = false;
  }
  if (!allDone && t < ON_US) return;

  for (byte i = 0; i < N_SENSORS; i++) {
    unsigned long endUs = pulseEnded[i] ? highUs[i] : now;
    strength[i] = pulseLow[i] ? endUs - lowUs[i] : 0;
  }
  for (byte i = 0; i < N_SENSORS; i++) pinMode(SENSOR_PIN[i], INPUT);
  digitalWrite(SENSOR_POWER, LOW);
  sensorsOn = false;
  sensorPhaseMs = millis();
  frameAngle = bodyAngle;
  frameReady = true;
}

void printFrame() {
  Serial.print(F("A0:"));
  Serial.print(strength[FRONT]);
  Serial.print(F(" A1:"));
  Serial.print(strength[RIGHT]);
  Serial.print(F(" A2:"));
  Serial.print(strength[BACK]);
  Serial.print(F(" A3:"));
  Serial.print(strength[LEFT]);
  if (!SENSOR_TEST) {
    Serial.print(F("  target "));
    Serial.print(targetAngle, 1);
    Serial.print(F(" gyro "));
    Serial.print(bodyAngle, 1);
  }
  Serial.println();
}

// ================= decide (every reading) =================
void decide() {
  unsigned long a0 = strength[FRONT];

  // 1. main sensor strong: shoot right here
  if (a0 >= FIRE_US) {
    if (state != FIRE) {
      targetAngle = bodyAngle;                   // freeze on the beacon
      setState(FIRE);
    }
    laser(true);
    return;
  }

  // 2. was firing and lost it: look around this spot
  if (state == FIRE) {
    if (a0 >= FIRE_OFF_US) return;               // still close enough: keep firing
    laser(false);
    startSweep(targetAngle, 1);
  }

  // 3. A0 sees it fairly well: the beacon is near the front. Never turn 90 here
  //    (a leaking side sensor would win): sweep around here, towards the brighter side.
  if (a0 >= NEAR_US) {
    if (state == SCAN) startSweep(bodyAngle, strength[LEFT] >= strength[RIGHT] ? 1 : -1);
    return;
  }

  // 4. a side sensor sees one: turn so A0 faces it (not while already turning there)
  int side = -1;
  for (byte i = RIGHT; i <= LEFT; i++) {
    if (strength[i] >= SIDE_MIN_US && (side < 0 || strength[i] > strength[side])) side = i;
  }
  if (side >= 0 && state != GOTO) {
    // just came from the opposite side: the beacon is between the two (~45 deg),
    // so go half way instead of ping-ponging left-right 90 deg forever
    bool back = (side == RIGHT && lastGotoSide == LEFT) || (side == LEFT && lastGotoSide == RIGHT);
    float turn = back ? PINGPONG_DEG : 90;
    if (side == RIGHT) goToSide(-turn);
    else if (side == LEFT) goToSide(turn);
    else goToSide(180);
    lastGotoSide = side;
    return;
  }

  // 5. A0 sees something weak while searching: slow down so the tube doesn't skip it
  if (state == SCAN) scanRate = a0 >= SEEN_US ? SLOW_RATE : SCAN_RATE;
}

// ================= setup / loop =================
void setup() {
  Serial.begin(115200);
  pinMode(LASER_PIN, OUTPUT);
  digitalWrite(LASER_PIN, LOW);
  pinMode(LED, OUTPUT);
  pinMode(SENSOR_POWER, OUTPUT);
  digitalWrite(SENSOR_POWER, LOW);
  for (byte i = 0; i < N_SENSORS; i++) pinMode(SENSOR_PIN[i], INPUT);
  sensorPhaseMs = millis();

  if (SENSOR_TEST) {
    Serial.println(F("SENSOR_TEST: strengths A0 front, A1 right, A2 back, A3 left - laser when A0 is strong"));
    return;
  }

  pinMode(sleepPin, OUTPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDirection, OUTPUT);
  motorOff();
  digitalWrite(sleepPin, HIGH);
  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  lastAB = digitalRead(encoderA) << 1 | digitalRead(encoderB);
  attachInterrupt(digitalPinToInterrupt(encoderA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), encoderISR, CHANGE);

  if (!gyroBegin()) {
    Serial.println(F("No gyro, check A4/A5"));
    while (true) {                        // no gyro: LED blinks fast
      digitalWrite(LED, !digitalRead(LED));
      delay(100);
    }
  }
  delay(3000);                            // hands off, let it stop swinging
  digitalWrite(LED, HIGH);
  gyroCalibrate(2000);                    // HANGING and STILL
  digitalWrite(LED, LOW);

  targetAngle = 0;
  lastCount = getCount();
  wheelTarget = lastCount;
  lastLoop = micros();
  running = true;
  Serial.println(F("gyro_pid_fire started - SCAN"));
}

void loop() {
  sensorsUpdate();

  if (SENSOR_TEST) {
    if (frameReady) {
      frameReady = false;
      printFrame();
      digitalWrite(LASER_PIN, strength[FRONT] >= FIRE_US ? HIGH : LOW);
    }
    return;
  }

  control();

  if (!frameReady) return;
  frameReady = false;
  if (DEBUG) printFrame();
  decide();
}
