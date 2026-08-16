# MQ Sentinel 🛰️
**Autonomous IR Beacon-Tracking Reaction-Wheel Satellite**

Macquarie University · ENGG2000 SPINE Engineering Project (2026 S2)

MQ Sentinel is a proof-of-concept autonomous "satellite" that locates and
"removes" simulated space debris. Debris is represented by a **38 kHz modulated
IR beacon**; the satellite autonomously detects the active beacon, rotates to
face it using an internal **reaction wheel**, and holds an **eye-safe laser** on
the target.

> Academic project. This repo contains our own design, code, and documentation.
> It does **not** include Macquarie's copyrighted brief/rubric materials.

---

## What it does (the core loop)
1. **Detect** — find the active 38 kHz IR beacon (~1 m away).
2. **Locate** — determine the beacon's bearing accurately enough to aim.
3. **Rotate** — spin the reaction wheel so the body turns to face the beacon.
4. **Fire & hold** — put the eye-safe laser on the target and hold for 2 s.
5. Repeat for each of three targets over a 5-minute run.

## My role — IR Sensing & Target Localisation
I own the **IR sensing subsystem** and act as the team's **code integrator**.
- **Input:** 38 kHz modulated IR from the beacon.
- **Output:** the beacon's *bearing* (direction), handed to the control/motor
  subsystem to drive rotation.

### Two-stage localisation
Because a plain digital sensor ring only resolves ~±22.5° (far too coarse to
score at 1 m, where the bullseye needs ~±2.3°), the design uses **coarse + fine**:

| Stage | Sensor | Method | Resolution |
|-------|--------|--------|------------|
| **Coarse** | Digital IR receiver (TSOP-style) + slow spin | "Which direction is the beacon?" | ~±22.5° |
| **Fine** | 4× Vishay **TSSP58P38** (overlapping FOV) | Compare per-sensor signal **strength** (pulse-width) → weighted centroid → angle | sub-degree (target) |

Key detail: the TSSP58P38's AGC makes its **output pulse width proportional to
signal strength**, so strength is read via `micros()`/`pulseIn()` on a digital
pin — *not* `analogRead()`.

## Hardware
| Part | Item |
|------|------|
| Controller | Arduino Nano V3.0 |
| Actuator | DC gearmotor (DFRobot FIT0186, w/ encoder) + steel flywheel (reaction wheel) |
| Motor driver | Pololu DRV8874 |
| Coarse sensor | 38 kHz IR receiver (TSOP-series) |
| Fine sensors | 4× Vishay TSSP58P38 IR detector modules |
| Emitter (target) | Laser: Jaycar XC4490 (eye-safe) |
| Power | 6× NiMH AA |
| Enclosure | 150 mm-dia × 170 mm PVC pipe |

## Repository structure
```
MQ_Sentinel/
├── MQ_Sentinel.ino     # main sketch: state machine + subsystem integration
├── ir_sensor.h         # IR module interface (declarations, constants)
├── ir_sensor.cpp       # IR detection: sawBeacon(), scan/localisation
└── motor_control.*     # reaction-wheel control (integrated here)
```

## Software overview
- `sawBeacon(pin)` — validates a real beacon by measuring the demodulated
  LOW-pulse length against the beacon's fingerprint (rejects ambient IR).
- Coarse scan — sweeps the digital sensor(s) to find the beacon's rough bearing.
- Fine localisation — reads the 4× TSSP strength cluster and interpolates angle.
- A `SIMULATION` build flag swaps real pulse-detection for button inputs so the
  logic can be tested in [Wokwi](https://wokwi.com) without hardware.

## Simulation
The detection/decision logic is validated in Wokwi (pushbuttons = sensors,
onboard LED = laser) before hardware testing.

## Roadmap (sprints)
- **V1 (MVP)** — integrated core loop: detect → rotate → fire (accuracy not required).
- **V2 (CDR)** — precision: TSSP strength interpolation, encoder + PID control, tuned reaction wheel.
- **V3 (Final)** — enclosed, robust, performance-optimised.

## References
- Vishay TSOP382 datasheet — 38 kHz IR receiver.
- Vishay TSSP58P38 datasheet — IR proximity detector (mid-range).
- joeiddon/ADCS_demo — single-axis reaction-wheel PID control (concept reference).
- matthieuvigne/eurobot_beacon — 360° IR beacon detection (concept reference).
- RoboCup Junior soccer bearing algorithms — strength-based IR localisation.

## License
Educational project. Code released under the MIT License. Course-supplied
materials and hardware remain the property of their respective owners.
