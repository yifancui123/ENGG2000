# Motor module (reaction wheel turning)

Turns the satellite body by a given number of degrees. A PD controller holds the
reaction wheel position using the encoder (4x quadrature, 2800 counts per wheel rev).

## Files
| File | What it is |
|---|---|
| `motor.h` | The functions you call |
| `motor.cpp` | The PD controller and encoder (no `setup()`/`loop()`) |
| `gyro.h` / `gyro.cpp` | MPU6050 gyro: body rate (deg/s) and angle (deg), z axis only. **Optional** - only when the gyro is fitted. |
| `steady.h` / `steady.cpp` | Measure-once steady hold after a turn (needs the gyro) - see below |
| `motor_module.ino` | Test sketch: type degrees in the Serial Monitor |

**To use it in the final sketch:** copy `motor.h` and `motor.cpp` into the final
sketch folder and add `#include "motor.h"`. Do **not** also copy the old
`main_sprint2/motor_control.cpp/.h` - Arduino compiles every `.cpp` in the
folder, so both would clash. Checked: it compiles together with
`analog_sensor.cpp/.h`.

## Functions
| Function | What it does |
|---|---|
| `motorBegin()` | Pins + encoder interrupts. Call once in `setup()`. |
| `turnBy(deg)` | Turn the body by `deg` degrees. **+ = anticlockwise (left), - = clockwise (right)**, seen from above. Returns straight away - the turn runs in `motorUpdate()`. |
| `motorUpdate()` | Runs the PD every 5 ms. **Call it every `loop()`.** |
| `motorBusy()` | `true` while a turn is running |
| `motorResult()` | `MOTOR_DONE`, `MOTOR_TIMEOUT`, `MOTOR_STOPPED`, `MOTOR_BUSY` or `MOTOR_IDLE` |
| `motorHold(degPerSec)` | After a turn: keep the wheel at a steady speed instead of braking, to cancel body drift. Use **minus** the drift (body drifts +2 deg/s -> `motorHold(-2)`). Runs until `turnBy()` or `motorStop()`. `motorBusy()` is false while holding, so sensors can be read. |
| `motorHolding()` | `true` while `motorHold` is running |
| `motorHoldNudge(deg)` | While holding: turn the body by `deg` on top of the hold speed (used by `steady`) |
| `motorStop()` | Brake now and cancel the turn or hold |
| `moveWheelBy(counts)` | Raw wheel move in counts (testing) |
| `motorCount()`, `motorTarget()`, `motorPwm()` | For logging |

## Rules for the logic code
1. **Call `motorUpdate()` every loop, and keep `loop()` fast.** While something
   blocks (`delay()`, a long `while`), the PD is not running.
2. **Never read the sensors while `motorBusy()` is true.** `readStrengths()`
   takes ~0.5 s per sensor with `delay()` inside, so the PD would stop mid-turn.
   Readings are only accurate when the body is still anyway.
3. Start a new turn only when `motorBusy()` is false.

```cpp
#include "motor.h"
#include "analog_sensor.h"

void setup() {
  motorBegin();
  setupSensor();
}

void loop() {
  motorUpdate();
  if (!motorBusy()) {
    // read sensors, work out the angle error in degrees
    // if (fabs(errDeg) > 1.5) turnBy(errDeg);
  }
}
```

## Turning + steady hold with the gyro (use this in main.ino)
After each turn the body keeps drifting (suspension wire, leftover spin), and
the encoder turn is only as accurate as `R_RATIO`. The gyro sees both.
**`steadyTurnBy(deg)`** does the whole job:

0. **TURNING** - `turnBy(deg)` on the encoder; target = gyro angle now + deg
1. **SETTLE** 0.1 s - turn finished, wheel braked
2. **MEASURE** 0.3 s - gyro averages the body drift rate
3. **CANCEL** 0.15 s - wheel set to the steady speed that cancels it (`motorHold(-drift)`)
4. **RETURN** up to 1.5 s - turn onto the **target gyro angle** (`motorHoldNudge`): fixes the turn error and the angle lost while measuring
5. **READY** - holding on the target: laser on, count the 2 s

`steadyStop()` + the next `steadyTurnBy()` clears the speed; the next target
measures again. The speed is measured only once, so if the drift changes during
the hold (wire winding up) it is not corrected - that's the next version.

| Function | What it does |
|---|---|
| `steadyTurnBy(deg)` | Turn by `deg` (+ = anticlockwise) and hold exactly there. **The main one.** |
| `steadyUpdate()` | Runs the steps. Call every loop with `motorUpdate()` and `gyroUpdate()`. |
| `steadyReady()` | `true` when holding on the target - fire the laser |
| `steadyActive()` | `true` from `steadyTurnBy` until `steadyStop` |
| `steadyStop()` | Clear the state before the next target (the wheel keeps its speed until the next turn) |
| `steadyStart()` / `steadyStartAt(angle)` | Hold where it is / a given gyro angle, if you turned with plain `turnBy` |
| `steadyHoldAngle()`, `steadyDrift()`, `steadyCorrection()` | For logging |

**Rule for the new sensor/integration code:** no `delay()` or blocking loops.
`motorUpdate()`, `gyroUpdate()` and `steadyUpdate()` must run every 5-10 ms,
also while reading the sensors - otherwise the gyro loses the angle and the
hold stops correcting. Write the sensor read as small `millis()` steps.

```cpp
#include "motor.h"
#include "gyro.h"
#include "steady.h"

void setup() {
  motorBegin();
  gyroBegin();
  gyroCalibrate(2000);              // satellite STILL for 2 s
}

void loop() {
  motorUpdate();
  gyroUpdate();
  steadyUpdate();                   // all three, every loop

  if (!steadyActive()) {
    // sensors say the target is errDeg away (+ = left)
    // steadyTurnBy(errDeg);
  }
  if (steadyReady()) {
    // on target and still: fire laser / count 2 s / IR check
    // next target:  steadyStop(); then steadyTurnBy(nextDeg);
  }
}
```

## Pins
| Nano | To |
|---|---|
| D2 | Encoder A |
| D3 | Encoder B |
| D5 | Driver EN |
| D6 | Driver PH |
| D7 | Driver nSLEEP |
| 5V / GND | Encoder VCC / GND |
| A4 / A5 | Gyro SDA / SCL (if fitted), gyro VCC -> 5V, GND -> GND |

Driver **PMODE must be tied to GND** (PH/EN mode). Battery + to driver VIN and
Nano VIN, all GNDs joined. Free for the sensors: D4, A0-A3, A6, A7
(**keep A4/A5 for the gyro**).

Gyro: mount flat near the middle of the body, chip facing up. Check the sign:
`turnBy(90)` should make `gyroAngle()` go to about +90. If it goes to -90, set
`GYRO_SIGN = -1` in `gyro.cpp`.

## Calibration (top of `motor.cpp`)
| Constant | Value | Meaning |
|---|---|---|
| `R_RATIO` | 0.14 | body angle / wheel angle. Checked on the hanging satellite: 4 x 90 deg turns ended close to the start (2026-10-23). |
| `COUNTS_PER_BODY_DEG` | ~55.6 | wheel counts per degree of body turn |
| `TURN_SIGN` | -1 | a + wheel move turns the body clockwise, so it's flipped to make `turnBy(+)` anticlockwise. **Confirmed on the satellite (2026-10-29).** |
| `kp`, `kd` | 1.0, 0.05 | PD gains (from simulation, tested on the bench) |
| `MAX_PWM` | 100 | top speed of the wheel during a turn |
| `DONE_TOL` | 10 counts | ~0.2 deg of body - turn counts as finished inside this |

If turns end short, `R_RATIO` is too big - make it smaller (and the other way).
Example: 4 x 90 deg ends 10 deg short of 360 -> new r = 0.14 x 350 / 360.

Timeout: 1 s + 2 ms per 4 counts (a 90 deg turn gets 3.5 s).

## Limits to know
- The encoder measures the wheel relative to the body, so the motor code
  **cannot see the body drifting** after a turn. Holding on target needs the
  gyro (`steady`) or the IR sensors (re-read and correct with small `turnBy` calls).
- Gyro and steady code compile but are **not tested on hardware yet**.
- Small turns (1-5 deg) not tested yet on the hanging satellite - swivel
  friction may stop the body moving for very small turns.
