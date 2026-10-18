// ===== 1. PINS + PD CONSTANTS =====
// copy your consts: motorPWM, motorDirection, encoderA, encoderB, sleep,
// kp, kd, MAX_PWM, MIN_PWM, DONE_TOL, LOOP_US
// (drop turnSpeed and countsPerDegree, they aren't needed)

// ===== 2. ENCODER VARIABLES =====
// volatile long pos        -> the count
// volatile byte lastAB     -> previous state of A and B

// ===== 3. PD VARIABLES =====
// long target              -> where we want to be (counts)
// long lastPos             -> count from the previous loop (for speed)
// unsigned long lastLoop   -> micros() of the previous loop
// unsigned long moveStart  -> millis() when the move began (for timeout)
// bool moving              -> true while a move is running
// int lastPwm              -> so the .ino can log it

// ===== 4. ENCODER READER (the interrupt) =====
// read A and B, compare with lastAB, pos++ or pos--, save lastAB

// ===== 5. getCount() =====
// noInterrupts(); copy pos; interrupts(); return the copy

// ===== 6. YOUR 3 MOTOR FUNCTIONS (paste unchanged) =====
// motorForward, motorReverse, motorStop

// ===== 7. drive(int u) =====
// u > 0  -> motorForward(u)
// u < 0  -> motorReverse(-u)
// u == 0 -> motorStop()

// ===== 8. setupMotor() =====
// pinModes (same as before), sleep HIGH
// set lastAB from the real pins
// attachInterrupt on encoderA AND encoderB, both CHANGE

// ===== 9. moveBy(long counts) =====
// target = getCount() + counts; lastPos = getCount();
// moveStart = millis(); lastLoop = micros(); moving = true;

// ===== 10. motorUpdate()  <- the PD, called every loop() =====
// if not moving -> return
// if less than LOOP_US since lastLoop -> return; else lastLoop += LOOP_US
// error = target - count
// speed = (count - lastPos) / 0.005   (counts per second), lastPos = count
// DONE?  |error| <= DONE_TOL and |speed| small -> drive(0), moving = false, return
// SAFETY? more than 3000 ms since moveStart -> drive(0), moving = false, return
// u = kp*error - kd*speed
// deadband: if |u| < MIN_PWM -> u = MIN_PWM with the sign of u
// clamp u to -MAX_PWM..+MAX_PWM
// drive(u), lastPwm = u

// ===== 11. isAtTarget() =====
// return !moving

const int motorPWM = 5;
const int motorDirection = 6;
const int sleep = 7;

// Encoder constants
const int encoderA = 2;
const int encoderB = 3;
volatile long pos = 0;
volatile byte lastAB = 0;
const int8_t QDEC[16] = { 0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0 };

// PD constants
const float kp = 1.0;
const float kd = 0.05;
const int MAX_PWM = 100;
const int MIN_PWM = 17;
const int DONE_TOL = 10;
const unsigned long LOOP_US = 5000;

// PD state
long target = 0;
long lastPos = 0;
unsigned long lastLoop = 0;
unsigned long moveStart = 0;
bool moving = false;
int lastPwm = 0;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50);
  pinMode(sleep, OUTPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(motorDirection, OUTPUT);
  digitalWrite(sleep, HIGH);

  pinMode(encoderA, INPUT);
  pinMode(encoderB, INPUT);
  int a = digitalRead(encoderA);
  int b = digitalRead(encoderB);
  lastAB = a << 1 | b;
  attachInterrupt(digitalPinToInterrupt(encoderA), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderB), encoderISR, CHANGE);
  Serial.println("ready: f / r");
}

void loop() {
  if (Serial.available()) {
    char c = Serial.peek();

    if (c == 'f' || c == 'r') {
      Serial.read();
      if (c == 'f') {
        drive(80);
        delay(300);
        motorStop();
        delay(500);
        Serial.print("count: ");
        Serial.println(getCount());
      } else if (c == 'r') {
        drive(-80);
        delay(300);
        motorStop();
        delay(500);
        Serial.print("count: ");
        Serial.println(getCount());
      }
    } else if (c == 's') {
      Serial.read();
      drive(0);
      moving = false;
      Serial.println("STOP");
    } else if (isDigit(c) || c == '-') {
      long n = Serial.parseInt();
      Serial.print("move by ");
      Serial.println(n);
      moveBy(n);
    } else {
      Serial.read();
    }
  }

  motorUpdate();

  static unsigned long lastPrint = 0;
  if (moving && millis() - lastPrint >= 10) {
    lastPrint = millis();
    Serial.print(millis());
    Serial.print(",");
    Serial.print(target);
    Serial.print(",");
    Serial.print(getCount());
    Serial.print(",");
    Serial.println(lastPwm);
  }
}

void moveBy(long counts) {
  long now = getCount();
  target = now + counts;
  lastPos = now;
  moveStart = millis();
  lastLoop = micros();
  moving = true;
}

void motorUpdate() {
  // A. nothing to do if no move is running
  if (!moving) return;

  // B. only run every 5ms (fixed dt, D needs this)
  if (micros() - lastLoop < LOOP_US) return;
  lastLoop += LOOP_US;

  // C. measure: position, error, speed
  long count = getCount();
  long error = target - count;              // how far still to go
  float speed = (count - lastPos) / 0.005;  // counts per second
  lastPos = count;

  // D. done? close enough and nearly stopped -> brake and finish
  if (abs(error) <= DONE_TOL && fabs(speed) < 200) {
    drive(0);
    lastPwm = 0;
    moving = false;
    Serial.print("DONE, error = ");
    Serial.println(error);
    return;
  }

  // E. safety: taking too long -> brake and give up
  if (millis() - moveStart > 3000) {
    drive(0);
    lastPwm = 0;
    moving = false;
    Serial.println("TIMEOUT");
    return;
  }

  // F. correct: the PD law, then deadbandd and clamp
  float u = kp * error - kd * speed;

  if (u > 0 && u < MIN_PWM) u = MIN_PWM;
  if (u < 0 && u > -MIN_PWM) u = -MIN_PWM;

  if (u > MAX_PWM) u = MAX_PWM;
  if (u < -MAX_PWM) u = -MAX_PWM;

  lastPwm = (int)u;
  drive(lastPwm);
}

bool isAtTarget() {
  return !moving;
}

void drive(int u) {
  if (u > 0) {
    motorForward(u);
  } else if (u < 0) {
    motorReverse(-u);
  } else {
    motorStop();
  }
}

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

void motorForward(int speedValue) {
  digitalWrite(motorDirection, HIGH);
  analogWrite(motorPWM, speedValue);
}

void motorReverse(int speedValue) {
  digitalWrite(motorDirection, LOW);
  analogWrite(motorPWM, speedValue);
}

void motorStop() {
  analogWrite(motorPWM, 0);
}
