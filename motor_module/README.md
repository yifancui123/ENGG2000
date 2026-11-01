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

What `steadyTurnBy` does (`HOLD_MODE = HOLD_PULSE`, the final setting):
0. **TURNING** - `turnBy(deg)` on the encoder; target = gyro angle now + deg
1. **SETTLE** 0.1 s - turn finished, wheel braked
2. **RETURN** - if more than 0.5 deg off the target, a small `turnBy` back
   + brake (a pulse); READY once inside 0.5 deg (max 1.5 s)
3. **READY** - holding on the target. Keeps checking the gyro: more than
   0.5 deg off -> pulse back, wait 0.1 s, check again. Wheel braked in between.

Other modes in `steady.cpp` (tested, not used): `HOLD_CONTINUOUS` (measure
the drift, then a gyro angle/rate cascade adjusting the wheel speed every
10 ms; zigzagged on our rig without more tuning) and `HOLD_ONCE` (measure
the drift once, one cancelling speed). Limit of pulses: they correct the
angle but can't cancel a push that keeps building (a twisted suspension
wire / seized swivel), so the swivel must turn freely.

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
| `HOLD_MODE` | steady.cpp | `HOLD_PULSE` | how the target is held (final) |
| `PULSE_DEAD` | steady.cpp | 0.5 deg | no pulse inside this (bullseye is 2.3 deg) |
| `PULSE_GAP_MS` | steady.cpp | 100 ms | wait after a pulse before checking again |

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

## Status (2026-11-02, final)
- Done on the satellite: driver, `turnBy(90)` direction, gyro wiring and
  sign, gyro drift while still (no creep), 90 deg turn + 10 s pulse hold
  (`AUTO_HOLD`, looked good by eye).
- Hold methods compared on the hanging satellite: measure-once (lost it
  after ~2 s to the wire), continuous (zigzag), pulses (best) -> pulses final.
- A stiff suspension wire / seized swivel makes the body spin back after a
  turn. Pulses keep correcting, but a free swivel is the real fix.
