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
| 0:25 | **0. Driver mode** (2 min) | `test_driver_mode` | Which pin logic the driver needs (saved on the Nano) |
| 0:30 | **1. Encoder check** (5 min) | `test_encoder_check` | Counts per rev and direction |
| 0:40 | **2. Deadband** (4 min) | `test_deadband` | Lowest PWM that moves the wheel |
| 0:50 | **3. Step and stop tests** (2 min) | `test_step` | Motor gain K, time constant τ, brake vs coast |
| 1:00 | **4. Speed curve** (2 min) | `test_speed_curve` | Speed vs PWM, max speed |
| 1:10 | **5. Analyse, then repeat anything odd** | MATLAB | `summary.txt` and plots |
| 2:15 | **6. Hanging satellite and phone** | `test_satellite` | Inertia ratio, drift after braking |
| 3:15 | **7. Analyse satellite data** | MATLAB + phyphox CSV | Body vs wheel angle |

## Step by step

### 0. Driver mode (do this first, once per board)
The DRV8874 can be in **PH/EN** mode (what the original code assumed) or **IN1/IN2** mode (set by its PMODE pin), or have D5/D6 swapped. With the wrong logic, "brake" can mean "full speed", which is what happened on the first run. `test_driver_mode` works out which one your board is and saves it in the Nano's EEPROM. Every other sketch reads it from there and keeps the motor off until it's set.
1. Motor clamped, battery on, hands clear. Upload `test_driver_mode`, open the Serial Monitor (115200).
2. Send `d`. The wheel gets three short kicks (0.15 s each), then a check pulse.
3. You want to see `# RESULT: mode ...` and then `# OK: brake works.` Anything else: battery off and tell Claude.
4. If it says `NOTE: forward pulse counted DOWN`, swap the D2 and D3 encoder wires.

The saved mode survives uploading other sketches. Re-run this only if the wiring or the Nano changes.

### 1. Encoder check (Serial Monitor, 115200)
You never touch the flywheel: the motor does all the turning. Battery on, motor clamped, hands clear.
1. Upload `test_encoder_check`.
2. Send `f` and `r` 2–3 times each. `f` should move a clearly **positive** count (hundreds or more), `r` a similar negative one.
   - `f` goes negative: swap the D2 and D3 wires and check again.
   - Wheel spins but the count stays near 0: one encoder wire is loose.
3. **Battery off**, put a tape mark on the flywheel rim and a matching one on the frame. **Battery on**, hands clear, send `1`. The motor turns the wheel one revolution (2800 counts) and prints how far past the frame mark the tape should stop (it overshoots a little). If the tape is where it says, **2800 counts per rev** is confirmed (the datasheet's 700 × 4, because the sketches count every edge on both channels).
4. If it's clearly not 2800, tell Claude before going on: `CPR` in `matlab/analyse_motor_tests.m` and `COUNTS_PER_REV` in the sketches would need changing.

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
| `test_driver_mode/` | Detects the driver mode and saves it to EEPROM. Run first |
| `*/driver.h` | Motor drive/brake/coast for all three driver modes. Identical copy in each sketch folder |
| `test_encoder_check/` | Counts per rev and direction check |
| `test_deadband/` | Breakaway and stopping PWM, 3 trials each way |
| `test_step/` | Step response, then brake or coast, logged every 5 ms |
| `test_speed_curve/` | Steady speed at 11 PWM levels each way |
| `test_satellite/` | Hanging test on battery; results saved to EEPROM |
| `matlab/run_motor_tests.m` | Records the bench tests to CSV |
| `matlab/capture_serial.m` | Helper: Nano → CSV |
| `simple_run/` | Just runs the motor: forward 2 s, brake, reverse 2 s, brake, repeat. PH/EN only, no setup |
| `matlab/analyse_motor_tests.m` | Fits K and τ, deadband, speed curve, stop times |
| `matlab/data/` | **Results from 2026-10-16**: CSVs, plots, `summary.txt`, and `RESULTS.md` (read this first) |

## Notes

- **Brake vs coast:** in DRV8874 PH/EN mode, PWM 0 shorts the motor (**brake**). Coast means putting the driver to sleep (outputs off). The step test compares the two. This decides how sharply the satellite can stop.
- **Stop PWM in the deadband test** reads slightly low, because the heavy flywheel keeps turning while the PWM ramps down. Use the speed-curve zero-speed PWM as a cross-check.
- All sketches **compile** for the Arduino Nano with no warnings. On 2026-10-16, `test_driver_mode`, `test_encoder_check`, `test_deadband`, `test_step`, `test_speed_curve` and `test_satellite` all ran on the real hardware, and the MATLAB analysis ran on the real data. `simple_run` compiles but hasn't been confirmed on the motor. Results are in `matlab/data/RESULTS.md`.
