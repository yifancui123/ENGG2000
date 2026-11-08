# main_see_fire: improvement plan

Branch `sprint2-see-fire`. Rewritten 8 Nov 2026 after the new gyro was fitted.

How we work: one step at a time. Each test gives numbers first. Each code change is shown first, made only once approved, committed on its own, then tested. Results go in the table at the bottom.

---

## The new scheme

1. **Coarse (A1-A3 + gyro):** A1-A3 are read as 1/0 (1 = LOW pulse of 2000 us or more). The pattern gives a rough direction to the beacon. Its world angle is β = gyro angle + that direction. The robot turns A0 to β.
2. **Fine (A0, gyro executes):** small steps around β. If A0 goes up, keep going the same way; if it goes down, turn back. The gyro makes each step and holds still before the next read.
3. **Fire:** A0 at or above the fire level (about 119,200, set by T3) and the body still: fire. The gyro holds the angle for the 2.5 s shot.
4. **Remember:** the gyro angle at each hit is saved as target 1/2/3. Next time a side sensor sees a beacon near a saved angle, turn straight there, let A0 confirm, fire.

Roles: A1-A3 know roughly which side. The gyro knows where the body points and how far it turned, but can't see the beacon. Only A0 knows when it is exactly on target.

---

## Where things stand (8 Nov)

- New MPU6050 works. Sign: + is anticlockwise from above. On the floor it drifts about 0.5° per minute.
- Motor sign checked: `10` turns the body anticlockwise, `-10` clockwise. Motor and encoder agree.
- Hanging, the string and USB cable twist the body about 0.5°/s, and calibrating while it twists gives a false gyro bias.
- A sensor with no power reads 119,980 (LOW from the first instant), which looks like saturation. The D4 joint came loose today. A real saturated reading is about 119,5xx-119,7xx and differs between sensors.
- If the battery is off while the code runs, the wheel target keeps building up. Plugging the battery in then made the robot spin about 4.5 turns. **Always connect the battery first, then reset.**

---

## Part 1: tests before any code

### T0: test setup
| What | How | Decides | Pass |
|---|---|---|---|
| String and USB twist | Battery off, hang still 60 s, watch `gyro` | Whether bench tests mean anything | Under 0.1°/s (swivel, USB hung loose from the same point) |
| String pull-back | Twist 45° by hand, let go. Time one swing, note where it stops | Largest safe turn | No clear spring-back |
| Start order | Battery first, then reset, hand-hold still for 7 s | No spin at start | Never spins at start |

### T1: how far one turn can go
`a0_scan_test`: type `10`, `20`, `45`, `90`, `135`, `180` in turn. For each, record the time to `at ...`, the error at the end, and the drift over the next 10 s.
Decides the largest coarse jump (90° or 45°) and whether 180° is split in two.
Pass: within 1°, under 1° of drift in 10 s.

### T2: side sensor field of view
Beacon 1 m away. `main_see_fire`, battery off, turn by hand to 0°, 22.5°, ... 337.5° (16 positions). At each, record `gyro` and A0-A3 from 2 lines. Repeat at 0.5 m and 1.5 m.
Decides the direction table for each 1/0 pattern, whether 2000 us is the right "seen" level (noise reaches about 800), any blind gaps, whether A3 works, and whether the sides all saturate together when close.
Pass: every pattern points within ±30° of the real direction, no blind gaps.

### T3: A0 angle curve
`a0_scan_test` `g`: -15° to +15° in 1° steps and back. Then the same to ±30°.
Decides the fire level, A0's centre offset from the laser, the fine step size (1° or 2°), and how wide the zone is where A0 rises towards the centre.
Pass: the edges match within 1° between the up and down sweeps.

Hand sweep from 7-8 Nov (beacon at 1 m, angles about ±2°), for reference:

| Angle | A0 left | A0 right |
|---|---|---|
| 0° | 119,524-119,684 (saturated) | same |
| ~2° | not measured | 111,260-113,180 |
| ~4-5° | not measured | 107,952-109,544 |
| 10° | 116,620-118,744 | 111,384-115,008 |
| 20° | 96,324-97,296 | 96,260-99,352 |
| 30° | 93,304-95,428 | 94,000-95,436 |
| 45°+ | | 102,000-104,000 |

So 96,000 fires even 45° off. About 119,200 splits straight-on from 10° off. Past 20° a leak floor of 93-104k means A0 can't tell direction.

### T4: A0 frame noise
Hold at 0°, 2° and 5°, 20 frames each.
Decides whether one frame is enough to say "A0 went up" or 2-3 frames must be averaged, and how big a change counts.
Pass: the spread at one angle is smaller than the step between neighbouring degrees.

### T5: hold during a shot
On the bullseye, `l` laser on, watch the dot for 3 s.
Decides whether the 2.5 s shot stays in the zone or needs a longer settle first.
Pass: the dot stays in the 8 cm bullseye (±4 cm).

### T6: how well the gyro remembers an angle
Laser on the bullseye, `z`. Then `90`, `-90`, `0`, five times. Measure how far the dot lands from the mark each time. Then hold 5 min and measure again.
Decides whether a saved angle can be fired at directly or A0 must re-check first.
Pass: within 2° (3.5 cm at 1 m) after 5 round trips.

---

## Part 2: code changes (each shown first, one commit each)

| # | Change | Based on |
|---|---|---|
| C1 | Safety: a reading of 119,900 or more is a sensor fault, never a fire | Today's no-power reading |
| C2 | Safety: don't let the wheel target build up while the motor can't move | Today's spin |
| C3 | Fire on the same frame, if the gyro rate is under about 2°/s | T5 |
| C4 | A1-A3 as 1/0 → direction table → β → turn to β | T1, T2 |
| C5 | A0 fine search around β; fire at the fire level | T3, T4 |
| C6 | Save hit angles as targets 1-3; go straight there next time | T6 |

C1 and C2 don't need any test data.

## Part 3: tests after the code

| Change | Test | Pass |
|---|---|---|
| C1 | Pull the D4 wire | Prints FAULT, no laser |
| C2 | Run 10 s with the battery off, then connect it | No spin |
| C3 | Beacon in the tube; time from FIRE print to laser on | Under 0.2 s. Prints HOLD while swinging |
| C4 | Beacon at 90°, 135°, 180°, -90°, 3 runs each | Within ±30° of β in 1-2 turns, never past ±360° |
| C5 | Start 10° and 20° off on each side | Always centres; in 5 runs the dot is in the 16 cm ring, ideally the 8 cm bullseye; never fires off target |
| C3 + C5 | Watch the dot during the 2.5 s shot | Stays in the same ring |
| C6 | Hit target 1, then 2, then target 1 again | Second visit fires within 4 s, no search |
| All | 3 beacons at random, full 5 min, 3 runs | Every run scores 5 or more; aim for 150+ |

Order: T0 → T1 → T2 → T3 with T4 → T5 → T6. C1 and C2 can be written any time.

The test sketches are in the repo root: `mpu_raw_test/`, `motor_encoder_test/` and `a0_scan_test/`. Their library files are copies. `motor_encoder_test` uses the PD folder's `motor.cpp`, which doesn't pull PH low when braking, so keep the battery within reach.

---

## Results

| Test | Date | Result | Decision |
|---|---|---|---|
| Gyro sign | 8 Nov | + is anticlockwise; floor drift about 0.5°/min | keep GYRO_SIGN |
| Motor sign | 8 Nov | `10` anticlockwise, `-10` clockwise; ±10 and ±30 DONE | keep TURN_SIGN |
| T0 | | | |
| T1 | | | |
| T2 | | | |
| T3 | 7-8 Nov (hand) | see the table under T3 | need the gyro scan |
| T4 | | | |
| T5 | | | |
| T6 | | | |
| C1-C6 | | | |
