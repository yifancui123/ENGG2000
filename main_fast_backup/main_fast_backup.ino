// BACKUP of main: same code, but the fine step grows while the beacon stays on
// the same side, so a far target takes fewer turns. Use it if main is too slow.
//
// States:
// SEARCH
//  nothing in front: turn from the side/back sensors, else scan SCAN_STEP and look again, reversing at +-MAX_ANGLE
// COARSE    
// a coarse turn was made, same as SEARCH, but repeats get smaller
// FINE_AIM  
// front sensors see it, turn by fineDeg() until locked()
// FIRE      
// laser on for FIRE_MS while steady keeps holding, then back to SEARCH to wait for the next target (see REFIRE_MS)
//
// - Nothing here blocks the thread. 
// - A frame only counts if its ON window started after the body went still.
// - The laser is only on while steady is holding, the brief scores nothing if it leaves the target while on, and the lower zone if it crosses one.
//
// ALSO: set SENSOR_TEST true to print the sensors with the motors off.

#include "motor.h"
#include "gyro.h"
#include "steady.h"

static const bool SENSOR_TEST = false;  // true = print sensors, no motor or gyro

// - pins -
static const int FINE_POWER = 4;        // VS of the TSSP group
static const int COARSE_POWER = 8;      // VS of the digital group
static const int LASER_PIN = 12;

enum SensorId { FRONT_L, FRONT_R, SIDE_L, SIDE_R, BACK, SENSOR_COUNT };
// A0/A1 give the direction; D9-D11 only say whether the beacon is on that side
static const uint8_t SENSOR_PIN[SENSOR_COUNT] = { A0, A1, 9, 10, 11 };

// - sensor timing -
static const unsigned long OFF_MS = 380;        // resets the AGC while off
static const unsigned long ON_US = 120000UL;    // length of thee detection window

// - Fine aiming - 
static const int FINE_SIGN = 1;                 // +1 T or tube mount, -1 slit mount
static const float FINE_STEP = 3;               // deg, first turn and after a side change (below the ~4.6 deg overlap)
static const float FINE_STEP_MAX = 9;           // deg, biggest step while it stays on one side
static const unsigned long SEEN_MIN_US = 2000;  // at or above this = beacon seen (blips reach ~1200)
static const byte MAX_FINE_TURNS = 10;          // then give up and look elsewhere

// - search and fire -
static const float SCAN_STEP = 45;              // deg, when nothing is seen
static const float MAX_ANGLE = 360;             // deg either way, so the wire can't wind up
static const unsigned long FIRE_MS = 2500;      // brief needs 2 s unbroken, 0.5 s spare for clock error
static const unsigned long REFIRE_MS = 3000;    // hold this long for the next target, then hit this one again

// Types go above the first function: Arduino puts its generated prototypes there.
struct SensorFrame {
    unsigned long strength[SENSOR_COUNT];   // LOW pulse width in us, 0 = nothing seen
    unsigned long onStartMs;                // when this frame's ON window began
};

struct Pulse {
    bool low;                               // went LOW
    bool ended;                             // came back HIGH
    unsigned long lowUs;
    unsigned long highUs;
};

enum State { SEARCH, COARSE, FINE_AIM, FIRE };

// - sensor state -
static Pulse pulse[SENSOR_COUNT];
static SensorFrame frame;
static bool frameReady = false;
static bool sensorsOn = false;
static unsigned long offStartMs = 0;
static unsigned long onStartUs = 0;
static unsigned long onStartMs = 0;

// - main state -
static State state = SEARCH;
static bool still = false;
static unsigned long stillSinceMs = 0;
static int scanDir = 1;
static int coarseDiv = 1;
static byte fineTurns = 0;
static float fineStep = FINE_STEP;
static int fineSide = 0;                // +1 left, -1 right, 0 = no fine turn yet
static unsigned long fireStartMs = 0;
static unsigned long refireAfterMs = 0;

// - sensors -
// A new OFF/ON cycle every 500 ms for all five sensors together. Reads the same
// thing as Yifan's readPair(), but in small steps so loop() keeps running.

static void sensorsPower(bool on) {
    digitalWrite(FINE_POWER, on ? HIGH : LOW);
    digitalWrite(COARSE_POWER, on ? HIGH : LOW);
    sensorsOn = on;
}

static void sensorsOff() {
    sensorsPower(false);
    offStartMs = millis();
}

static void sensorsBegin() {
    pinMode(FINE_POWER, OUTPUT);
    pinMode(COARSE_POWER, OUTPUT);
    for (byte i = 0; i < SENSOR_COUNT; i++) {
        pinMode(SENSOR_PIN[i], INPUT);      // not INPUT_PULLUP: it can feed a powered-off sensor
    }
    sensorsOff();
}

static void pollPulses(unsigned long now) {
    for (byte i = 0; i < SENSOR_COUNT; i++) {
        Pulse &p = pulse[i];
        if (p.ended) continue;              // first pulse only
        bool low = digitalRead(SENSOR_PIN[i]) == LOW;
        if (low && !p.low) {
            p.low = true;
            p.lowUs = now;
        } else if (!low && p.low) {
            p.ended = true;
            p.highUs = now;
        }
    }
}

static void finishFrame() {
    unsigned long endUs = onStartUs + ON_US;
    for (byte i = 0; i < SENSOR_COUNT; i++) {
        const Pulse &p = pulse[i];
        frame.strength[i] = p.low ? (p.ended ? p.highUs : endUs) - p.lowUs : 0;
    }
    frame.onStartMs = onStartMs;
    frameReady = true;
    sensorsOff();
}

static void sensorsUpdate() {
    if (!sensorsOn) {
        if (millis() - offStartMs < OFF_MS) return;
        memset(pulse, 0, sizeof pulse);
        sensorsPower(true);
        onStartUs = micros();
        onStartMs = millis();
        return;
    }
    unsigned long now = micros();
    if (now - onStartUs < ON_US) pollPulses(now);
    else finishFrame();
}

static bool sensorsNewFrame() {
    bool ready = frameReady;
    frameReady = false;
    return ready;
}

// - reading a frame -

static bool seen(const SensorFrame &f, SensorId id) {
    return f.strength[id] >= SEEN_MIN_US;
}

static bool frontSees(const SensorFrame &f) {
    return seen(f, FRONT_L) || seen(f, FRONT_R);
}

// e = (L-R)/(L+R): distance-independent, 0 when both are equal
static float fineError(const SensorFrame &f) {
    float l = f.strength[FRONT_L];
    float r = f.strength[FRONT_R];
    return l + r > 0 ? (l - r) / (l + r) : 0;
}

// + = beacon to the left, the same sign as steadyTurnBy(). Only one front sensor
// sees it here. Same side as last time: step grows. Side changed: it went past
// the overlap, so back to the small step.
static float fineDeg(const SensorFrame &f) {
    int side = seen(f, FRONT_L) ? 1 : -1;
    fineStep = side == fineSide ? min(fineStep + FINE_STEP, FINE_STEP_MAX) : FINE_STEP;
    fineSide = side;
    return FINE_SIGN * side * fineStep;
}

// Both front views only overlap near the laser line, so both seen = on target.
// A saturated reading still counts as seen.
static bool locked(const SensorFrame &f) {
    return seen(f, FRONT_L) && seen(f, FRONT_R);
}

// Bits: 1 = left, 2 = right, 4 = back. Left + right alone (index 3) is a guess
// for the team to check: front sees nothing, so turn round.
static int coarseTurn(const SensorFrame &f) {
    static const int TURN[8] = { 0, 90, -90, 180, 180, 135, -135, 180 };
    byte bits = seen(f, SIDE_L)
              | seen(f, SIDE_R) << 1
              | seen(f, BACK) << 2;
    return TURN[bits];
}

static void sensorsPrint(const SensorFrame &f) {
    long l = f.strength[FRONT_L];
    long r = f.strength[FRONT_R];
    float e = fineError(f);

    Serial.print(F("A0(L)="));   Serial.print(l);
    Serial.print(F("  A1(R)=")); Serial.print(r);
    Serial.print(F("  diff="));  Serial.print(l - r);
    Serial.print(F("  e="));     Serial.print(e, 4);

    Serial.print(locked(f)         ? F("  -> CENTERED")
               : seen(f, FRONT_L)  ? F("  -> LEFT")
               : seen(f, FRONT_R)  ? F("  -> RIGHT")
               :                     F("  -> NO SIGNAL"));

    Serial.print(F("  SL="));   Serial.print(f.strength[SIDE_L]);
    Serial.print(F("  SR="));   Serial.print(f.strength[SIDE_R]);
    Serial.print(F("  B="));    Serial.print(f.strength[BACK]);
    Serial.print(F("  turn=")); Serial.println(coarseTurn(f));
}

// - turning -
static void setState(State s) {
    static const char *const NAMES[] = { "SEARCH", "COARSE", "FINE_AIM", "FIRE" };
    state = s;
    Serial.println(NAMES[s]);
}

static void startTurn(float deg) {
    steadyStop();
    steadyTurnBy(deg);
    still = false;
}

static float keepWireUnwound(float deg) {
    // Same heading, but the other way round if this turn would pass MAX_ANGLE
    float end = gyroAngle() + deg;
    if (end > MAX_ANGLE) return deg - 360;
    if (end < -MAX_ANGLE) return deg + 360;
    return deg;
}

static void scanStep() {
    coarseDiv = 1;
    if (fabs(gyroAngle() + scanDir * SCAN_STEP) > MAX_ANGLE) scanDir = -scanDir;
    startTurn(scanDir * SCAN_STEP);
    setState(SEARCH);
}

static void aim(const SensorFrame &f);

// SEARCH and COARSE: front sensors first, then side and back, then scan
static void look(const SensorFrame &f) {
    if (frontSees(f)) {
        coarseDiv = 1;
        fineTurns = 0;
        fineSide = 0;
        setState(FINE_AIM);
        aim(f);
        return;
    }
    refireAfterMs = 0;                      // that beacon has gone, so the next can fire at once
    int deg = coarseTurn(f);
    if (deg == 0) {
        scanStep();
        return;
    }
    // The side windows are about 118 deg wide, so a full 90 can land the beacon
    // in the opposite window and bounce between them for ever. Each coarse turn
    // in a row goes half as far.
    startTurn(keepWireUnwound((float)deg / coarseDiv));
    if (coarseDiv < 4) coarseDiv *= 2;
    setState(COARSE);
}

static void aim(const SensorFrame &f) {
    if (locked(f)) {
        if (millis() < refireAfterMs) return;   // already hit this one
        digitalWrite(LASER_PIN, HIGH);
        fireStartMs = millis();
        setState(FIRE);
    } else if (!frontSees(f)) {
        setState(SEARCH);
        look(f);
    } else if (fineTurns >= MAX_FINE_TURNS) {
        scanStep();                         // never got strong enough: a reflection?
    } else {
        fineTurns++;
        startTurn(fineDeg(f));
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(LASER_PIN, OUTPUT);
    digitalWrite(LASER_PIN, LOW);
    sensorsBegin();

    if (!SENSOR_TEST) {
        motorBegin();
        if (!gyroBegin()) {
            Serial.println(F("No gyro, check A4/A5"));
            while (true) {}
        }
        gyroCalibrate(2000);                // satellite hanging STILL
        steadyStart();                      // first decision comes once this is READY
    }
    Serial.println(F("MQ Sentinel started"));
}

void loop() {
    if (SENSOR_TEST) {
        sensorsUpdate();
        if (sensorsNewFrame()) sensorsPrint(frame);
        return;
    }

    motorUpdate();
    gyroUpdate();
    steadyUpdate(); // all three, EVERY loop
    sensorsUpdate();

    if (!still && steadyReady()) {
        still = true;
        stillSinceMs = millis();
    }

    if (state == FIRE && millis() - fireStartMs >= FIRE_MS) {
        digitalWrite(LASER_PIN, LOW);
        stillSinceMs = millis(); // next decision needs a frame from after the shot
        refireAfterMs = millis() + REFIRE_MS;
        setState(SEARCH);
    }

    if (!sensorsNewFrame() || state == FIRE) return;
    if (!still || frame.onStartMs < stillSinceMs) return; // taken while turning

    if (state == FINE_AIM) aim(frame);
    else look(frame);
}
