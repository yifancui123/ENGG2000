# ENGG2000 — T1 Brake Test Data Record

| Constant        | What it does                                                                                      | Starting value |
| --------------- | ------------------------------------------------------------------------------------------------- | -------------- |
| `BRAKE_KD`      | Brake strength (PWM per unit of speed). Too low = won't stop; too high = overshoots / oscillates. | `30.0f`        |
| `BRAKE_MINPWM`  | Smallest PWM that actually moves the motor (overcomes stiction).                                  | `40`           |
| `BRAKE_MAXPWM`  | Upper clamp, so a fast spin doesn't slam full power.                                              | `200`          |
| `STOP_SPEED`    | Speed (counts/ms) below which we call it "stopped".                                               | `0.05f`        |
| `SAMPLE_MS`     | How often speed is measured. Leave as is.                                                         | `10`           |
| `SETTLE_NEEDED` | How many consecutive "stopped" samples before we trust it. Leave as is.                           | `5`            |
| `BRAKE_DEBUG`   | `true` = print `vel=` to serial while tuning. Set `false` later.                                  | `true`         |

### 1. Direction Test
`motor_control.cpp`: lines 69–70 (the last two lines inside `brakeHold()`)

Current:
```cpp
if (vel > 0) motorReverse(drive);
else         motorForward(drive);
```
If speed increases (runaway → wrong direction), swap them:
```cpp
if (vel > 0) motorForward(drive);
else         motorReverse(drive);
```

### 2. Speed Test

- **STOP_SPEED**
	1. **Observe:** During testing, check the serial monitor when the motor has stopped. Note the small changes in velocity (`vel`), such as ±0.1.
	2. **Adjust:** Set `STOP_SPEED` slightly above the maximum fluctuation.
	3. **Test:** Upload the updated code and run the test again. Check that the motor stops correctly without stopping too early.

- **BRAKE_KD**
	1. **Adjust `BRAKE_KD`:** Increase it gradually if the motor stops too slowly. Decrease it if the motor overshoots or swings back and forth.
	2. **Adjust `BRAKE_MINPWM`:** Increase it if the motor buzzes but cannot move near the end.
	3. **Re-test:** Upload the changes and check the motor's behaviour.

|              | 1   | 2   | 3   |
| ------------ | --- | --- | --- |
| BRAKE_KD     |     |     |     |
| STOP_SPEED   |     |     |     |
| BRAKE_MINPWM |     |     |     |

Serial Monitor:

```
(paste serial output here)
```

### 3. Confirm pass
1. **Observe:** Check that the body stays still during each 2-second rest period without drifting.
2. **Confirm:** If the body stops smoothly and stays still for the full 2 seconds, T1 passes.

### 4. Troubleshooting
| Symptom                                   | Likely cause                                | Fix                                                               |
| ----------------------------------------- | ------------------------------------------- | ----------------------------------------------------------------- |
| `vel` grows, motor speeds up              | Brake direction backwards                   | Swap the two `motorForward`/`motorReverse` lines in `brakeHold()` |
| Never stops, `brakeHold` always times out | `BRAKE_KD` too low, or `STOP_SPEED` too low | Raise `BRAKE_KD`; raise `STOP_SPEED` a bit                        |
| Stops then swings back and forth          | `BRAKE_KD` too high                         | Lower `BRAKE_KD`                                                  |
| Buzzes but won't move when slow           | PWM below stiction                          | Raise `BRAKE_MINPWM`                                              |
| No `vel=` output                          | Wrong baud / `BRAKE_DEBUG` false            | Serial Monitor at 115200; set `BRAKE_DEBUG = true`                |
| Motor doesn't spin at all                 | Driver asleep / no motor power              | Check D7 (sleep) HIGH; battery connected                          |
