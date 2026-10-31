# Motor module (reaction wheel turning + gyro hold)

Turns the satellite body by a given number of degrees with the reaction
wheel, then holds it on the target with the gyro so the laser can stay on
the bullseye.

- **motor**: PD controller on the wheel encoder (4x quadrature, 2800 counts
  per wheel rev). Turns the body by degrees.
- **gyro**: MPU6050, measures how fast and how far the body turns.
- **steady**: after a turn, holds the body on the target angle with the gyro.

## Files
| File | What it is | Copy into main? |
|---|---|---|
| `motor.h` / `motor.cpp` | Wheel PD and encoder | **Yes** |
| `gyro.h` / `gyro.cpp` | MPU6050: body rate (deg/s) and angle (deg) | **Yes** |
| `steady.h` / `steady.cpp` | Turn + hold on target with the gyro | **Yes** |
| `motor_module.ino` | Test sketch only (Serial + battery-only tests) | **No** |

**To use it in main.ino:** copy the 6 `.h`/`.cpp` files into the main sketch
folder and add the three `#include`s. Do **not** copy `motor_module.ino`: it
has its own `setup()`/`loop()` and would clash. Do not copy the old
`main_sprint2/motor_control.cpp/.h` either: Arduino compiles every `.cpp` in
the folder, so they would clash.

**Ownership:** the motor/gyro/steady files are owned by the motor subsystem.
If main.ino needs a change in them, ask instead of editing, so we don't get
git conflicts or two different copies. Function names and meanings in the
`.h` files will not change; tuning inside the `.cpp` files may.

## Use this in main.ino
```cpp
#include "motor.h"
#include "gyro.h"
#include "steady.h"

void setup() {
  motorBegin();
  gyroBegin();                      // false = gyro not found (check A4/A5)
  gyroCalibrate(2000);              // satellite hanging STILL for 2 s
}

void loop() {
  motorUpdate();
  gyroUpdate();
  steadyUpdate();                   // all three, EVERY loop

  if (!steadyActive()) {
    // sensors say the target is errDeg away (+ = anticlockwise / left)
    // steadyTurnBy(errDeg);
  }
  if (steadyReady()) {
    // on target and holding: laser on, count the 2 s
    // next target:  steadyStop(); then steadyTurnBy(nextDeg);
  }
}
```

## Rules for main.ino
1. **Call `motorUpdate()`, `gyroUpdate()` and `steadyUpdate()` every loop,
   at least every 5-10 ms.** If they stop, the PD stops, the gyro loses the
   angle and the hold stops correcting.
2. **No `delay()` or long blocking loops** anywhere in main.ino, including
   the sensor reads. Write them as small `millis()` steps.
3. Start a new turn only when `steadyActive()` is false (or after
   `steadyStop()`).
4. At power-on the satellite must be **hanging and still for 2 s** while
   `gyroCalibrate()` runs: it measures the gyro bias and which way is "up".
5. Keep **A4/A5 for the gyro**. Free for the sensors: D4, A0-A3, A6, A7
   (A6/A7 are analog input only).

## Functions
### steady (the ones main.ino needs)
| Function | What it does |
|---|---|
| `steadyTurnBy(deg)` | Turn by `deg` (**+ = anticlockwise**, seen from above) and hold exactly there. **The main one.** |
| `steadyUpdate()` | Runs the steps. Call every loop. |
| `steadyReady()` | `true` when holding on the target: fire the laser |
| `steadyActive()` | `true` from `steadyTurnBy` until `steadyStop` |
| `steadyStop()` | Clear the state before the next target |
| `steadyStart()` / `steadyStartAt(angle)` | Hold where it is / at a given gyro angle |
| `steadyHoldAngle()`, `steadyDrift()`, `steadyCorrection()` | For logging |

What `steadyTurnBy` does:
0. **TURNING** - `turnBy(deg)` on the encoder; target = gyro angle now + deg
1. **SETTLE** 0.1 s - turn finished, wheel braked
2. **MEASURE** 0.3 s - gyro measures the body drift rate
3. **CANCEL** 0.15 s - wheel set to the speed that cancels that drift
4. **RETURN** up to 1.5 s - turn back onto the target
5. **READY** - holding on the target

With `CONTINUOUS = true` (top of `steady.cpp`, the default) steps 4 and 5
keep correcting the wheel speed from the gyro every 20 ms (angle + rate,
with the wheel speed as the integral), so a suspension wire that twists
back is fought for the whole hold. `false` = measure once, one speed, one
nudge. Limit: if something keeps pushing for long, the wheel reaches top
speed (~64 rpm) and can't hold any more.

### gyro
| Function | What it does |
|---|---|
| `gyroBegin()` | Start the MPU6050. Returns `false` if it doesn't answer. |
| `gyroCalibrate(ms)` | Bias + "up" direction. Satellite STILL. |
| `gyroUpdate()` | Read + integrate every 5 ms. Call every loop. |
| `gyroRate()` | Body turn rate, deg/s (+ = anticlockwise) |
| `gyroAngle()` | Body angle since calibrate/reset, deg |
| `gyroResetAngle()` | Angle = 0 |

Works in **any mounting** (flat, on the wall, tilted): calibration finds
"up" from gravity and the gyro measures turning about that axis.

### motor
| Function | What it does |
|---|---|
| `motorBegin()` | Pins + encoder interrupts. Call once in `setup()`. |
| `turnBy(deg)` | Turn the body by `deg` on the encoder only (no gyro hold) |
| `motorUpdate()` | Runs the PD every 5 ms. Call every loop. |
| `motorBusy()` | `true` while a turn is running |
| `motorResult()` | `MOTOR_DONE`, `MOTOR_TIMEOUT`, `MOTOR_STOPPED`, `MOTOR_BUSY`, `MOTOR_HOLDING` or `MOTOR_IDLE` |
| `motorStop()` | Brake now and cancel the turn or hold |
| `motorHold(degPerSec)` / `motorHoldSpeed()` / `motorHoldNudge(deg)` / `motorHolding()` | Wheel hold speed (used by steady) |
| `moveWheelBy(counts)`, `motorCount()`, `motorTarget()`, `motorPwm()` | Testing / logging |

## Pins
| Nano | To |
|---|---|
| D2 | Encoder A |
| D3 | Encoder B |
| D5 | Driver EN |
| D6 | Driver PH |
| D7 | Driver nSLEEP |
| 5V / GND | Encoder VCC / GND |
| A4 | Gyro SDA |
| A5 | Gyro SCL |
| 5V / GND | Gyro VCC / GND |

Driver **PMODE must be tied to GND** (PH/EN mode). Battery + to driver VIN
and Nano VIN, all GNDs joined. Gyro AD0, INT, XDA, XCL: leave unconnected.

## Calibration
| Constant | File | Value | Meaning |
|---|---|---|---|
| `R_RATIO` | motor.cpp | 0.14 | body angle / wheel angle (hang test, 2026-10-23) |
| `TURN_SIGN` | motor.cpp | -1 | makes `turnBy(+)` anticlockwise. **Confirmed on the satellite.** |
| `kp`, `kd` | motor.cpp | 1.0, 0.05 | wheel PD gains |
| `MAX_PWM` | motor.cpp | 100 | top speed during a turn (~70 rpm wheel, ~60 deg/s body) |
| `GYRO_SIGN` | gyro.cpp | +1 | anticlockwise = +. **Confirmed.** |
| `CONTINUOUS` | steady.cpp | true | gyro hold keeps correcting |
| `ANGLE_GAIN`, `RATE_GAIN` | steady.cpp | 3.0, 0.15 | hold gains. Wobbles: RATE_GAIN 0.08. Creeps: ANGLE_GAIN 5. |

If turns end short, `R_RATIO` is too big (and the other way). Example:
4 x 90 deg ends 10 deg short of 360 -> new r = 0.14 x 350 / 360.

## Testing (motor_module.ino)
Flags at the top. Only one `AUTO_` true at a time; **all `AUTO_` false**
for the normal Serial test. Serial Monitor 115200, New Line:
- a number, e.g. `90` / `-45`: steady turn, prints READY and how far off
  it is 2 s later
- `s` stop, `g` gyro live print, `z` angle = 0, `c` recalibrate,
  `p` print saved test results

Battery-only tests (15 s countdown, LED on 3 s = calibrating, keep still,
LED stays on = finished, fast blinking = no gyro). Plug USB back in, set the
flag false, upload, type `p`:
- `AUTO_HOLD`: one 90 deg turn, 10 s hold, error every 0.5 s
- `AUTO_TEST`: 14 turns (90 down to 1 deg, both ways), 2 s hold each
- `AUTO_90` / `AUTO_45` / `AUTO_SWEEP`: older encoder-only tests

## Status (2026-11-01)
- Done on the satellite: driver, `turnBy(90)` direction, gyro wiring and
  sign, gyro drift while still (no creep).
- Being tested: the steady hold over 10 s (`AUTO_HOLD`), small turns
  (1-10 deg) and the full `AUTO_TEST`. Numbers may be tuned; function names
  won't change.
- A stiff suspension wire / seized swivel makes the body spin back after a
  turn. The continuous hold fights it, but a free swivel is the real fix.
