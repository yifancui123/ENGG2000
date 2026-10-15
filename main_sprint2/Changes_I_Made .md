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

## 7. Notes / limits

- This only **stops** the body. It does **not** return to a target angle — that needs a position term (P) added later.
- The encoder measures the **wheel**, and body angle only tracks wheel angle while momentum is conserved, so this estimate **drifts** over time. The IR/TSSP sensor is what corrects that drift later.
- Record the final tuned values of `BRAKE_KD`, `BRAKE_MINPWM`, `STOP_SPEED` — they are evidence for the Engineering Rigour report.
