# test_motor_3: motor and satellite tests

Standalone test sketches for measuring the reaction wheel motor. They don't touch `main_sprint2`.
Pins match the current wiring: encoder A/B on D2/D3, DRV8874 EN on D5, PH on D6, nSLEEP on D7.

**Safety:** the flywheel is 0.6 kg of steel. Clamp the motor (or the satellite) before running, and keep hands clear.

## Before you go

- [ ] Batteries charged. Run all tests on **battery power**, not USB alone.
- [ ] Arduino IDE and MATLAB on the laptop. Note the Nano's COM port (Tools → Port) and set it in `matlab/run_motor_tests.m`.
- [ ] **phyphox** app installed on your phone. Look up your phone's weight.
- [ ] Tape, protractor or printed angle disc, ruler.

## Session plan (about 4 h)

| Time | Step | Sketch | Output |
|---|---|---|---|
| 0:00 | Meeting with teammate | — | Gyro ordered, pins and interface agreed |
| 0:30 | **1. Encoder check** (5 min) | `test_encoder_check` | Counts per rev and direction |
| 0:40 | **2. Deadband** (4 min) | `test_deadband` | Lowest PWM that moves the wheel |
| 0:50 | **3. Step and stop tests** (2 min) | `test_step` | Motor gain K, time constant τ, brake vs coast |
| 1:00 | **4. Speed curve** (2 min) | `test_speed_curve` | Speed vs PWM, max speed |
| 1:10 | **5. Analyse, then repeat anything odd** | MATLAB | `summary.txt` and plots |
| 2:15 | **6. Hanging satellite and phone** | `test_satellite` | Inertia ratio, drift after braking |
| 3:15 | **7. Analyse satellite data** | MATLAB + phyphox CSV | Body vs wheel angle |

## Step by step

### 1. Encoder check (Serial Monitor, 115200)
1. Upload `test_encoder_check`. Put a tape mark on the flywheel. Send `z`.
2. Turn the flywheel **5 full turns** by hand, slowly. Counts ÷ 5 = **counts per rev**. Expect about **2800**, because the sketches count every edge on both channels (4× the 700 the old code used).
3. Send `f`. The count should go **up**. Write down if it goes down.
4. If counts per rev isn't about 2800, change `CPR` in `matlab/analyse_motor_tests.m` and `COUNTS_PER_REV` in the sketches.

### 2–4. Bench tests (MATLAB records them for you)
**Close the Serial Monitor first.** Only one program can use the port at a time.
1. Open `matlab/run_motor_tests.m` and set `port`.
2. Upload `test_deadband`, then run **section 1** (Ctrl+Enter).
3. Upload `test_step`, then run **section 2**. It runs 7 steps automatically, including max speed (255) and two coast stops.
4. Upload `test_speed_curve`, then run **section 3**.
5. Run **section 4**. It prints a summary and saves `data/summary.txt`, `step_responses.png` and `speed_curve.png`.

If MATLAB recording gives trouble, every sketch also works in the Serial Monitor. Copy the output into a `.csv` in `matlab/data/` with the same file name.

### 6. Hanging satellite (battery, **no USB**)
1. Upload `test_satellite`, unplug USB, and fit everything in the satellite.
2. Tape the phone flat on top, centred. In phyphox, open **Gyroscope** and press ▶.
3. Switch the satellite on. It does a **15 s countdown** (LED blinks), then **6 moves** (+ − + − + −), each 1.5 s of wheel spin, a brake, and an 8 s wait. The LED stays on when it's done (about 1.2 min).
4. Stop phyphox and export the data as CSV (⋮ → Export Data → CSV).
5. Plug USB in, open the Serial Monitor, and **send `p` within 15 s** to print the wheel results (otherwise the test runs again).
6. Do it **twice** for repeatability.

**What it gives you:**
- **Inertia ratio** = body angle turned (from phyphox: z-rotation added up over each move) ÷ wheel angle (`wheel_deg`).
- **Drift after braking:** during each 8 s wait, does the body stop dead or keep turning? That shows how much the swivel friction and air drag disturb it.

## Files

| Folder or file | What it does |
|---|---|
| `test_encoder_check/` | Counts per rev and direction check |
| `test_deadband/` | Breakaway and stopping PWM, 3 trials each way |
| `test_step/` | Step response, then brake or coast, logged every 5 ms |
| `test_speed_curve/` | Steady speed at 11 PWM levels each way |
| `test_satellite/` | Hanging test on battery; results saved to EEPROM |
| `matlab/run_motor_tests.m` | Records the bench tests to CSV |
| `matlab/capture_serial.m` | Helper: Nano → CSV |
| `matlab/analyse_motor_tests.m` | Fits K and τ, deadband, speed curve, stop times |

## Notes

- **Brake vs coast:** in DRV8874 PH/EN mode, PWM 0 shorts the motor (**brake**). Coast means putting the driver to sleep (outputs off). The step test compares the two. This decides how sharply the satellite can stop.
- **Stop PWM in the deadband test** reads slightly low, because the heavy flywheel keeps turning while the PWM ramps down. Use the speed-curve zero-speed PWM as a cross-check.
- **Not yet compiled or run on hardware.** The MATLAB analysis was checked on synthetic data only.
