# MQ Sentinel – Sprint 2

The turret steps around, stops, reads the IR sensor(s), and fires the laser if the
front sensor clearly sees the beacon. After a shot it creeps slowly ("stalls")
while the laser is on.

## Files

| File | What it does |
|---|---|
| `main_sprint2.ino` | Main loop: motor step → read sensors → fire if the rule says so |
| `motor_control.cpp/.h` | Motor driver + closed-loop position control using the encoder |
| `analog_sensor.cpp/.h` | Power-cycled TSSP58P38 strength reader |

## Closed-loop motor control

Before, the motor was **open loop**. Each burst ran at PWM 150 for 80 ms and then
stopped, so how far the turret turned depended on battery level, friction and load.
The encoder was counting, but nothing used the count.

Now each step asks the encoder to move a set angle, and a controller drives the
motor until the encoder confirms it's there.

### How it works

- **`moveTo(target, maxSpeed, timeoutMs)`**: a PD controller that checks position
  every 2 ms and sets the PWM from:
  - **P (`KP`)**: pushes harder the further away it is.
  - **D (`KD`)**: brakes based on speed so it doesn't overshoot.
  - **`MIN_PWM`**: a floor so the motor still moves when close to the target.
  - It counts as done once it stays within ±`POS_TOLERANCE` counts (about 1°) for
    `SETTLE_SAMPLES` checks in a row. If it never gets there, it gives up after the timeout.
- **`stepDegrees(angle, maxSpeed)`**: moves relative to the previous *target*, not
  where the turret actually stopped. Small errors get corrected on the next step
  instead of adding up.
- **`pos` is a `long`**: as an `int` it would overflow after about 46 turns of
  continuous scanning. `readPos()` reads it with interrupts briefly turned off so you
  never get a half-updated value.
- `turnAngle`, `rotateTo` and `isAtTarget()` now use the controller. `isAtTarget()`
  reports a real result instead of always returning true.

In `main_sprint2.ino`, scanning calls `stepDegrees(SCAN_STEP_DEG, SCAN_SPEED)` and
stalling calls `stepDegrees(STALL_STEP_DEG, STALL_SPEED)`.

## Setup checklist (do this on the hardware first)

1. **Encoder direction**: run `motorForward(150)` briefly and print `readPos()`. If
   the count goes *down*, set `ENCODER_SIGN = -1` in `motor_control.cpp`. If it's
   wrong, the controller pushes the wrong way and the motor runs until the 2 s timeout.
2. **Step sizes**: `SCAN_STEP_DEG = 10` and `STALL_STEP_DEG = 1` are placeholders.
   Set them to whatever angle you actually want per step.
3. **Tuning**:
   1. Set `KD = 0` and raise `KP` until the turret reaches its target quickly.
   2. Raise `KD` until the overshoot or wobble stops.
   3. If it stalls just short of the target, raise `MIN_PWM`.
4. **`countsPerDegree = 700/360`**: the encoder only counts rising edges on
   channel A, so check that 700 really is the count for one turn of the turret.

## Tunable constants

| Constant | File | Default | Meaning |
|---|---|---|---|
| `KP` | `motor_control.cpp` | 3.0 | PWM per count of error |
| `KD` | `motor_control.cpp` | 20.0 | PWM per (count/ms) of speed |
| `MIN_PWM` | `motor_control.cpp` | 60 | Lowest PWM that still turns the motor |
| `POS_TOLERANCE` | `motor_control.cpp` | 2 | Counts counted as "at target" |
| `SETTLE_SAMPLES` | `motor_control.cpp` | 5 | Checks in tolerance before stopping |
| `ENCODER_SIGN` | `motor_control.cpp` | 1 | Flip to -1 if the encoder counts backwards |
| `SCAN_STEP_DEG` | `main_sprint2.ino` | 10.0 | Degrees per scan step |
| `STALL_STEP_DEG` | `main_sprint2.ino` | 1.0 | Degrees per step while stalling |
| `SCAN_SPEED` / `STALL_SPEED` | `main_sprint2.ino` | 150 | Max PWM for each mode |

## Notes

- **Scan speed is limited by the sensor, not the motor.** A sensor read takes
  about 0.5 s (380 ms off + up to 120 ms on), not the ~50 ms the code
  comments say.
- The closed-loop code hasn't been compiled or tested on hardware yet.
