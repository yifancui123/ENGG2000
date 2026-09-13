# TSSP58P38 Strength Test (tssp_strength)

## 1. Why the reading uses micros() instead of analogRead()

- The TSSP58P38 tells you signal strength through TIME, not voltage.
- Its OUT pin is digital: it only sits at 0 V or 5 V, nothing in between.
- The strength is encoded in how long OUT stays LOW = the pulse WIDTH.
  - Stronger / more on-axis signal = wider LOW pulse.
  - Weaker / off-axis = narrower pulse. Nothing = no pulse.
- So we measure the LOW time with micros() / pulseIn().
- analogRead() would be wrong here: on a digital pin it only ever reads ~0 or ~1023, so it cannot capture a graded strength.
- this sensor is not a true analog sensor

## 2. Why the serial speed is 115200 instead of 9600

- The sketch prints all sensor values on every loop, very often.
- At 9600 baud each character takes about 1 ms, so one line takes tens of
  ms to send. That can be longer than the loop, so the serial buffer
  backs up and the readings lag / lose real-time timing.
- At 115200 the same line sends about 10x faster, so printing no longer
  slows the loop and the Serial Plotter stays live.
- It also matches tssp_continuous_test (also 115200), so every TSSP
  sketch uses the same Serial Monitor speed.

## 3. What this file does

- Reads the TSSP58P38 sensor(s) and prints each one's strength.
- Strength = LOW pulse width in microseconds (bigger = stronger).
- Prints tab-separated values for the Arduino Serial Plotter at 115200.
- Current pins: A2 = front, A3 = right, A4 = back, A5 = left
  (used as digital inputs; this 4-sensor layout is a test scaffold,
  see section 5 for the intended layout).
- Wiring per sensor: OUT -> sensor pin, GND -> GND, VS -> 5 V.
  (Push-pull output, so no pull-up resistor needed.)
- Purpose: this is the fine-aim strength sensor for V2, used after the
  VS1838B has found the beacon roughly.

## 4. How the auto-gain (AGC) works and why it matters for accuracy

- AGC = Automatic Gain Control: the sensor constantly adjusts its own
  internal amplification to the incoming signal.
- It is built for BURSTS of 38 kHz. It measures the burst and reports
  the result as the OUT pulse width, which grows with signal strength
  (datasheet Fig 2 and Fig 6).
- Good side: this gives a graded strength number, which is exactly what
  lets us interpolate an angle finer than one sensor's field of view.
- Three ways it hurts accuracy, all of which we must design around:
  - Saturation: a very strong signal pushes the pulse width to its
    maximum. At 1 m the beacon is strong (range spec is 21 m), so
    sensors can all read "max" and give no angle information.
  - Slow updates: AGC needs recovery time between bursts (Fig 9), so
    the sensor only updates a few times per second. We must slow or
    pause the spin while taking a fine reading.
  - Continuous signal risk: with a constant (non-bursted) beacon the
    AGC may fire once then go quiet. This is the S2-01 gate test.

## 5. Using multiple TSSP sensors to find the target bearing

### The idea: use multiple sensors

- Mount ~3 sensors facing roughly the SAME forward direction, slightly fanned so their wide cones overlap.
- The body spins to bring the beacon into that forward cluster.
- Compare the sensors' pulse widths and take a strength-weighted centre:
      `bearing = sum(strength_i * angle_i) / sum(strength_i)`
  This gives a finer angle than any single sensor, plus some averaging.

### The constraint: the field of view is very wide

- Datasheet: directivity phi-half = +/-45 deg (Vishay 82476). Response is
  still ~50% of peak at 45 deg off-axis.
- 3 forward sensors with +/-45 deg cones overlap almost completely, so they read nearly the same and cannot tell angles apart on their own.

### Preventing too much overlap (and the 1 m problem)

- At 1 m the beacon is strong (range spec 21 m) AND the cones are wide,
  so the likely failure is all sensors saturating at max pulse width =
  no gradient = no usable bearing.
- Fixes, to confirm on the bench:
  - Collimate each sensor (shroud / tube) to narrow its field of view
    well below +/-45 deg. This is the main lever that turns overlap into
    a usable gradient.
  - Slightly splay the aim directions so the narrowed peaks sit at
    distinct angles.
  - Keep the target on the STEEP part of each sensor's angle curve, not
    the flat peak.
- The exact field of view, splay and sensor count are a later decision.
  Measure it: sweep one sensor -40 to +40 deg at 1 m, log pulse width vs
  angle, then set geometry from that curve (task S2-04).

## 6. Plan and my current thinking

### Chosen approach: multiple sensors + spin

- Coarse then fine: the VS1838B (digital) finds the beacon roughly and
  co-aims the laser; the TSSP cluster then refines the exact bearing.
- Use ~2-3 TSSP sensors facing the same forward direction (fanned, cones
  overlapping) as the fine stage.
- The body spins (reaction wheel) to sweep the sensors across the beacon.

### Two jobs every spin plan must do

1. ACQUIRE - find the beacon somewhere around you (wide, fast, coarse).
2. AIM + HOLD - centre on it to +/-2.3 deg and hold for 2 s (fine).

Two facts drive the choice:
- Reaction-wheel reality: the motor gives TORQUE, not speed, so every
  move is accelerate -> brake and it naturally OVERSHOOTS and oscillates.
- What PID needs: PID servos on an ERROR signal (how far off, which side).
  - A digital sensor only says present / not present over a wide cone =
    on/off, good for ACQUIRE, weak for AIM.
  - A TSSP cluster gives a graded, SIGNED error (one side stronger) =
    the real PID input for AIM.
  - The encoder gives relative angle (drifts) = good fast inner damping
    loop, no absolute beacon info.

### Spin control - one direction

**Plan 1: one digital sensor + gradual slow-down**

| Pros | Cons |
|---|---|
| Very simple, no tuning | Digital cone is wide (~+/-35 deg), so "detected" can be 35 deg off centre - it starts slowing at the wrong place |
| One cheap sensor | Open-loop stop: no idea HOW FAR off it is -> cannot hit +/-2.3 deg |
| | Reaction wheel keeps coasting (torque) -> unpredictable overshoot |
| | Slow too early = stop short; too late = sail past. No mid-move fix |

Verdict: fine for V1 "roughly points at it", cannot score the bullseye.
The missing piece is feedback - it never measures its error.

**Plan 1b: PID, no digital sensor (PID on TSSP only)**

| Pros | Cons |
|---|---|
| TSSP centroid IS the error signal PID wants -> accurate AIM | Bad at ACQUIRE: TSSP faces forward with limited angle + slow AGC -> blindly spinning to first catch it is slow/unreliable |
| Fewer sensor types | If the beacon is outside the TSSP view, PID has no signal -> cannot converge |

Verdict: great fine loop, poor finding. Missing the acquire stage.

**Plan 1c: digital sensor + PID**

| Pros | Cons |
|---|---|
| Digital = reliable ACQUIRE across the spin | Most complex (two sensor types + tuning + handoff logic) |
| PID on TSSP = precise AIM + damps overshoot + holds | One-direction trap: if PID overshoots past the beacon, it can only spin forward -> nearly a full 360 deg to re-acquire |

Verdict: best of the one-direction options for accuracy - but the one-way
constraint punishes every overshoot with a full lap.

Best one-direction = 1c. Plan 1 cannot reach scoring accuracy; 1b cannot
reliably find the beacon.

### Spin control - two directions + PID

This fixes the deepest flaw. A reaction wheel overshoots - and with two
directions you just REVERSE to correct, which is exactly what PID does
naturally. Overshoot stops being a 360-deg-lap disaster and becomes a
small back-and-forth trim. This is the correct control architecture for a
reaction wheel.

| Pros | Cons |
|---|---|
| Overshoot -> immediate reverse (no full lap) | Must drive the wheel both ways + manage wheel saturation over 5 min |
| PID can dither around centre and HOLD during the 2 s dwell | More tuning; slightly more code |
| Fastest, most accurate settle | |

How many TSSP sensors does two-direction + PID need?
- Minimum = 1 TSSP (plus the digital for acquire). Two directions let you
  probe: nudge, see if strength rose or fell, hill-climb to the peak.
  Works, but slow (each probe waits on the slow AGC), and one sensor gives
  magnitude, not which-side.
- Sweet spot = 2 TSSP (left/right, overlapping). Their difference
  (left - right) is a SIGNED error -> drop straight into PID. Tells you
  which way AND how far in one read.
- Best = 3 TSSP - adds a centre reference, smoother centroid, redundancy.

Answer: 2 is the practical minimum for a clean PID servo; 3 for the
smoothest, most robust centroid. (1 works only as slow hill-climbing.)

### Recommendation

- Architecture: two-direction spin + PID, digital sensor for ACQUIRE +
  TSSP for AIM, 2-3 TSSP sensors.
- Why: only this combo does both jobs well - the digital sensor finds the
  beacon fast, two-direction PID beats the reaction wheel's overshoot
  instead of fighting it, and the 2-3 TSSP difference gives PID the signed
  error it needs for +/-2.3 deg.
- Build order: get Plan 1c working first (one-direction, digital + PID) to
  prove acquire + aim, then upgrade to two-direction to kill the overshoot
  penalty. Mostly the same code - you are just allowing reverse.

### Plan to complete the sensor subsystem

1. Get the Nano talking to the PC again (current blocker: USB port).
2. S2-01 gate: confirm ONE TSSP keeps pulsing under the CONTINUOUS beacon
   (tssp_single_test / tssp_continuous_test).
   - If it goes quiet -> fallback: ask tutor to burst the beacon, or use a
     real analog sensor, or tighten the VS1838B collimator.
3. Calibrate one sensor: sweep -40 to +40 deg at 1 m, log pulse width vs
   angle, plot the curve (task S2-04).
4. Add the 2-3 sensor cluster (same direction); check the centroid gives a
   finer, stable bearing (target +/-2.3 deg for the 8 cm bullseye).
5. Add collimation if the cones overlap too much / saturate at 1 m.
6. Build the coarse -> fine handoff (VS1838B scan -> slow -> TSSP read).
7. Integrate with motor + laser: full chain spin -> detect -> fine aim ->
   hold 2 s -> fire.
8. Decide and tune the spin control (Plan 1c first, then two-direction).
9. Measure V1 vs V2 bearing accuracy at 1 m and plot the improvement.

## Status

- Compiles cleanly for Arduino Nano (verified with arduino-cli).
- Not yet tested on hardware (blocked on the Nano USB connection).
- Gated on S2-01: confirm TSSP keeps pulsing under the continuous beacon
  before relying on any of the above.

## Reference

- Vishay TSSP58P38 datasheet, Document 82476, Rev 2.1 (27-May-2025).
