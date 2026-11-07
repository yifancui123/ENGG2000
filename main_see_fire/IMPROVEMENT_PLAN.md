# main_see_fire: improvement plan

Starting point: branch `sprint2-see-fire`, commit `71f3c31` (8 Nov 2026).

The current code finds the beacon in 10° steps and fires as soon as A0 crosses 96,000. It works end to end. It is **slow to find** the beacon (up to 18 steps) and it **fires at the edge** of A0's view, not the centre.

How we work: one step at a time. Each code change is shown first, made only once approved, then committed and tested on the hanging robot. Results go in the table at the bottom.

---

## Tomorrow (8 Nov): checklist

1. **New gyro:** fit a new MPU6050 with the headers soldered. VCC → 5V, GND → GND, SDA → A4, SCL → A5. Run `mpu_raw_test`: when still, one `acc` axis should read about ±16000, and `gyro` should change when the body is turned. The old module reads all 0 and is dead.
2. **Motor check:** with the 12 V battery on, run `motor_encoder_test` and type `10`, `-10`, `90`. Each should print `DONE`. Hanging and seen from above, `+` must turn the body anticlockwise. After a reset the direction must not flip. If it does, the PMODE→GND wire is loose again.
3. **Step 0 baseline:** upload `main_see_fire` with the robot hanging and still for 7 s. Record the numbers listed under Step 0.
4. **A0 scan (`a0_scan_test`):** beacon 1 m away at A0's height, robot hanging, battery on.
   1. Power on and keep still for 2 s.
   2. `l` turns the laser on. Type small angles, e.g. `2` or `-1`, until the laser is on the bullseye.
   3. `z` sets that angle as 0. Then `l` turns the laser off.
   4. `g` starts the scan: -15° → +15° → -15° in 1° steps, 3 frames each, about 3 min.
   5. Paste all `SCAN`, `STEP` and summary lines back.
5. **Decide the fire logic** from the scan (see Step 1, option D).

The test sketches are in the repo root: `mpu_raw_test/`, `motor_encoder_test/` and `a0_scan_test/`. Their library files are copies. `motor_encoder_test` uses the PD folder's `motor.cpp`, which doesn't pull PH low when braking, so keep the battery within reach.

---

## Step 0: baseline test with a new gyro (no code change)

**Why:** every later step needs real numbers.

**Before starting:** fit a new MPU6050, run `mpu_raw_test` (one acc axis about ±16000), connect the 12 V battery and hang the robot.

**Measure:**
- **Time per 10° step:** from `-> TURN` to the next printed line.
- **Turn accuracy:** after a turn, how close is `gyro` to `hold`? Does the body really turn left for A3 and right for A1?
- **Hold:** does the angle drift during the 2.5 s laser shot?
- **Aim:** where does the laser land on the target the first time it fires? Measure the distance from the bullseye in cm.
- **Total:** time from the beacon switching on to the first shot, with the beacon at 90° and at 180°.

**Done when:** the table has these numbers for at least 3 runs.

---

## Step 1: aim at the centre of A0, not the edge (biggest score gain)

**Problem:** the first frame where A0 is at or above 96,000 is at the edge of A0's view. With the tube, A0 stays saturated about 10° off-axis. The bullseye is ±2.3° at 1 m, and the outer ring is ±6.9°.

**Test first (no code change):** with `SENSOR_TEST = true` and the beacon at 1 m, turn the robot by hand in 2° steps from -20° to +20°. Record A0 at each angle. This shows where A0 crosses 96,000 on each side and whether the two sides are equal. Earlier data hinted they are not: about 10-20° on one side, and under 10° on the other.

**Hand sweep, 7-8 Nov night.** Beacon at 1 m, robot turned by hand, so angles are about ±2°:

| Angle | A0 left | A0 right |
|---|---|---|
| 0° | 119,524-119,684 (saturated) | same |
| ~2° | not measured | 111,260-113,180 |
| ~4-5° | not measured | 107,952-109,544 |
| 10° | 116,620-118,744 | 111,384-115,008 (separate run) |
| 20° | 96,324-97,296 | 96,260-99,352 |
| 30° | 93,304-95,428 | 94,000-95,436 |
| 45°+ | | 102,000-104,000 |

What this shows:
- A0 saturates only very close to straight on. About 2° to the right it already reads about 111k.
- 96,000 is far too low. It fires even 45° off, because leaks and reflections leave a floor of 93-104k at every angle past 20°.
- A threshold of about **119,200** splits straight on (lowest seen 119,524) from left 10° (highest seen 118,744).
- Left looks wider than right, so A0's centre may sit a little left of the 0° mark.
- Past 20°, A1-A3 pick the right way. At 10° they can pick the wrong way (left 10°: A3 104k > A1 98k).
- Risk: with a ~2° saturated zone, 10° steps can jump over it. Near the beacon the robot needs small steps.

**Next:** run `a0_scan_test` with a working gyro. The robot turns itself in 1° steps from -15° to +15° and logs A0 against the gyro angle.

**Options, chosen by that data:**
- **A. Calibrated overshoot (fast):** A0 crosses the threshold while turning, so keep turning the same way by half of A0's measured view width, then fire. This costs one extra turn and only works if the edges are consistent.
- **B. Edge-to-edge scan (accurate, slower):** after the first crossing, keep turning in 2-3° steps until A0 drops below the threshold. That is the second edge. Turn back to the middle of the two edges and fire. This costs about 5-8 extra steps.
- **C. Make A0 narrower (hardware):** use a longer tube or a smaller hole, so A0 is only saturated within ±2-3°. That makes the current code accurate with no code change. Fixing the A0 light leak belongs here too.
- **D. Three zones (most likely, after the hand sweep):**
  - A0 below about 100,000: use the side sensors and big turns.
  - A0 between about 100,000 and 119,200: the beacon is near. Turn 1-2° at a time. If A0 goes up, keep going the same way. If it goes down, turn back.
  - A0 at or above 119,200: hold still and fire. If the scan shows A0's centre is off from the laser, add that offset first.
  The exact numbers come from the `a0_scan_test` result.

**Done when:** in 5 runs, the laser lands within the 16 cm ring each time, ideally in the 8 cm bullseye.

---

## Step 2: big turns from the side sensors (faster search)

**Problem:** 10° per step means 9 steps for a beacon at the side and 18 steps for one behind.

**Idea:** the side sensors are 90° apart, so the strongest one already says roughly where the beacon is (within ±45°):
- A3 (left) strongest → turn **+90°**.
- A1 (right) strongest → turn **-90°**.
- A2 (back) strongest → turn **180°**, taking whichever way keeps the wire inside ±360°.
- Then A0 and small steps finish the job.

**Risk:** a 90° jump can overshoot by up to 45°. If tests show overshoot, try 60° jumps.

**Done when:** a beacon at 90° or 180° is reached in 3 turns or fewer, and Step 0's total time is at least halved.

---

## Step 3: don't trust a single reading

**Problem:** with the beacon still, the side sensors jump between frames (A3: 132 → 50,000 → 132). Close up, A1-A3 all saturate at about 119,500, so "the strongest one" is random.

**Ideas:**
- Turn only when two frames in a row pick the same side. This costs one extra frame (0.5 s) per decision.
- When two or more sides are saturated, don't pick one. Look again, or use A0.
- Find out why A3 flickers: check the beacon's on/off pattern, the sensor power timing, and reflections.

**Done when:** in 5 runs the robot never turns the wrong way first.

---

## Step 4: smaller fixes

- **A0 half-lit:** when A0 reads 20,000-96,000, the beacon is close in front. Take small steps towards the brighter of A1 and A3 instead of the blind search.
- **Wire limit:** every turn, not only the search, must keep the body within ±360° so the hanging wire can't wind up.
- **Fire threshold:** re-check 96,000 after Step 1 and the A0 leak fix.

---

## Results

| Step | Date | What we tried | Result | Keep? |
|---|---|---|---|---|
| 0 | | baseline, new gyro | | |
| 1 | 7-8 Nov | hand sweep of A0, ±30° | A0 saturated only near 0°; 96,000 fires up to 45° off; about 119,200 looks right | need a gyro scan |
| 2 | | | | |
| 3 | | | | |
| 4 | | | | |
