# T1 — `brakeHold()` Reaction-Wheel Brake Test

**Branch:** `motor-brake-test`
**Goal of this test:** prove the turret can be commanded to **stop and stay still**, using only the encoder (no gyro).

---

## 1. Why this test exists

We drive the turret with a **reaction wheel** (flywheel on the FIT0186 gearmotor), not wheels on the ground.

- On a normal robot, cutting motor power = the ground brakes you → you stop.
- On a reaction wheel there is almost no friction, so cutting power (`motorStop()`) only makes the body **coast** at its current speed. It never really stops.

`brakeHold()` fixes this: it measures the wheel's speed from the encoder and drives the motor **in the opposite direction, proportional to speed**, until the body stops. This is the foundation for later steps (aim at the target, hold the laser on it for 2 s).

**Pass = after we call `brakeHold()`, the body stops within a few seconds and stays still.**
(It does **not** return to a set angle yet — that is a later task.)

---

## 2. Hardware / wiring

Arduino Nano + Pololu DRV8874 driver + FIT0186 gearmotor with quadrature encoder.

| Signal | Nano pin |
|---|---|
| Motor PWM | D5 |
| Motor direction | D6 |
| Driver sleep/enable | D7 |
| Encoder A | D2 |
| Encoder B | D3 |

- Power the motor driver from the battery (not USB).
- Serial monitor baud: **115200**.
- The IR sensors / laser are **not needed** for this test.

---

## 3. What the test does (code)

In `main_sprint2.ino` the normal `loop()` is temporarily commented out and replaced with:

```cpp
void loop() {
  Serial.println("spin...");
  motorForward(SCAN_SPEED);   // spin up to simulate a scan
  delay(400);

  Serial.println("brake..");
  brakeHold(3000);            // actively brake, up to 3 s

  Serial.print("done, rest 2s");
  delay(2000);                // watch: does it stay still?
}
```

Each cycle: spin 0.4 s → brake → rest 2 s → repeat.

---

## 4. The tuning knobs (and where they are)

**All of them are at the top of `main_sprint2/motor_control.cpp`.**

| Constant | What it does | Starting value |
|---|---|---|
| `BRAKE_KD` | Brake strength (PWM per unit of speed). Too low = won't stop; too high = overshoots / oscillates. | `30.0f` |
| `BRAKE_MINPWM` | Smallest PWM that actually moves the motor (overcomes stiction). | `40` |
| `BRAKE_MAXPWM` | Upper clamp, so a fast spin doesn't slam full power. | `200` |
| `STOP_SPEED` | Speed (counts/ms) below which we call it "stopped". | `0.05f` |
| `SAMPLE_MS` | How often speed is measured. Leave as is. | `10` |
| `SETTLE_NEEDED` | How many consecutive "stopped" samples before we trust it. Leave as is. | `5` |
| `BRAKE_DEBUG` | `true` = print `vel=` to serial while tuning. Set `false` later. | `true` |

---

## 5. Test procedure (do this in order)

> ⚠️ **First run can run away if the brake direction is wired backwards.** Do step 1 carefully.

### Step 1 — Safety first-run (check direction)
1. Set `BRAKE_KD = 10.0f` (small, so a runaway is slow and you can react).
2. Upload. Open Serial Monitor at **115200**.
3. **Keep a hand on the turret or be ready to cut power.**
4. Watch the `vel=` numbers during the "brake.." phase:
   - **`vel` shrinks toward 0** → direction is correct. Go to Step 2.
   - **`|vel|` keeps growing** → direction is backwards (positive feedback). **Cut power.** In `motor_control.cpp`, inside `brakeHold()`, **swap the last two lines**:
     ```cpp
     if (vel > 0) motorForward(drive);   // was motorReverse
     else         motorReverse(drive);   // was motorForward
     ```
     Re-upload and repeat Step 1.

### Step 2 — Set the "stopped" threshold
1. While the turret is physically still, look at the `vel=` values (noise).
2. Set `STOP_SPEED` just **above** that noise (e.g. if it jitters around ±0.02, set `0.05`).

### Step 3 — Tune the brake strength
1. Raise `BRAKE_KD` gradually and re-test:
   - Stops **too slowly / never** → increase `BRAKE_KD`.
   - **Overshoots / swings back and forth** → decrease `BRAKE_KD`.
2. If the motor buzzes but doesn't move near the end → increase `BRAKE_MINPWM`.

### Step 4 — Confirm pass
- During each "rest 2s" phase the body should **stay still** (not drift away).
- If it stops cleanly and holds through the 2 s rest → **T1 passes.**

---

## 6. Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `|vel|` grows, motor speeds up | Brake direction backwards | Swap the two `motorForward`/`motorReverse` lines in `brakeHold()` |
| Never stops, `brakeHold` always times out | `BRAKE_KD` too low, or `STOP_SPEED` too low | Raise `BRAKE_KD`; raise `STOP_SPEED` a bit |
| Stops then swings back and forth | `BRAKE_KD` too high | Lower `BRAKE_KD` |
| Buzzes but won't move when slow | PWM below stiction | Raise `BRAKE_MINPWM` |
| No `vel=` output | Wrong baud / `BRAKE_DEBUG` false | Serial Monitor at 115200; set `BRAKE_DEBUG = true` |
| Motor doesn't spin at all | Driver asleep / no motor power | Check D7 (sleep) HIGH; battery connected |

---

## 7. Notes / limits

- This only **stops** the body. It does **not** return to a target angle — that needs a position term (P) added later.
- The encoder measures the **wheel**, and body angle only tracks wheel angle while momentum is conserved, so this estimate **drifts** over time. The IR/TSSP sensor is what corrects that drift later.
- Record the final tuned values of `BRAKE_KD`, `BRAKE_MINPWM`, `STOP_SPEED` — they are evidence for the Engineering Rigour report.
