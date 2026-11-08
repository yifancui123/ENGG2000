# MQ Sentinel test data (2026-11-08)

All serial data from 8 Nov 2026, 12:23-14:57, on branch `sprint2-see-fire`. The raw logs, CSVs and the chart are in [`test_data_2026-11-08/`](test_data_2026-11-08/).

![A0, A1 and A3 against body angle, sweep S2](test_data_2026-11-08/a0_sweep_s2.svg)

## 1. Key findings

| Question | Result | Evidence |
|---|---|---|
| Does the new MPU6050 work? | Yes. + is anticlockwise from above | R1 |
| Gyro drift when still | About 0.5° per minute on the floor | R4 |
| Motor direction | `10` turns the body anticlockwise, `-10` clockwise | R2 |
| Why did the body swing about 120° in the first run? | Not a sign error. Likely the string and USB twist plus a false gyro bias from calibrating while twisting | R0, R3, R4 |
| Hanging twist with the motor off | About 0.5°/s (string + USB cable) | R3 |
| What does an unpowered sensor read? | 119,820-119,980 on every unpowered pin, the same on all of them. Looks like saturation and fires the laser | R1, R5, R7, R8 |
| What does a real saturated A0 read? | 119,268-119,760, different on each sensor | R10, S1, S2 |
| Background with the beacon off | 0-760 us | R6, R9, R11 |
| What caused the 4.5-turn spin? | The battery was connected while the code ran. The wheel target had built up with the motor dead, and it caught up all at once | R7 |
| **A0 angle curve** | **Saturated (about 119,500) from +2.0° to +8.4°, a 6.4° flat top centred at about +5.2°. Falls about 6,000 us per degree on both sides** | S2 |
| A0 noise per frame | About ±1,000 on the slopes, about ±150 on the top | S1, S2 |
| Where does 96,000 fire? | From about -1.5° to +9.3° (11° wide). Up to about 5° from the centre | S2 |
| Do A1-A3 work as 1/0 (2,000 us)? | No. With the beacon in front at 1 m, A1, A3 and even A2 read 40,000-107,000 | S1, S2 |
| Does A3 - A1 give the direction? | Yes, beyond about ±3° of 0. Its zero is about 4-5° away from A0's centre | S2 |

## 2. Test setup

- **Code:** `main_see_fire` at `b444da0` (no code changes today). Plan rewritten at `a936688`.
- **Sensors:** four TSSP58P38. OUT goes to A0-A3, VS shares D4, 380 ms off / 120 ms on. A0 has a tube. A1-A3 have 6.5 mm holes.
- **Gyro:** new MPU6050, VCC 5 V, SDA A4, SCL A5.
- **Positions:** looking along A0 and the laser, A1 is the body right, A2 back, A3 the body left. The gyro's + is anticlockwise from above.
- **Fire rule in the code:** A0 ≥ 96,000 (`FIRE_MIN_US`).

## 3. All runs

| Run | Time | Condition | Result |
|---|---|---|---|
| R0 | 12:23 | Hanging, battery on, beacon behind and close | Each 10° command became a 90-140° swing the other way. Swing grew from about ±35° to about ±70° |
| R1 | 12:38-12:42 | Plan A: gyro sign by hand, battery off | Each "turn anticlockwise" read about +80°, each "back" about -98°. Sign correct. Creep of about +1.1°/s. A0-A2 = 119,9xx (sensor power lost) |
| R2 | 12:46-12:49 | Plan B: `motor_encoder_test`, hanging, battery on | ±10 and ±30 all DONE within 10 counts in 0.5-1.25 s. Wheel at about 1,550 counts/s at PWM 60 |
| R3 | 12:50-12:52 | Hanging, untouched, USB attached, battery off | Gyro 0.9 → 52.3 in 100 s. The body really turned (string + USB) |
| R4 | 12:54-12:55 | On the floor, untouched | Gyro 0.0 → -0.5 in 63 s. The gyro is good |
| R5 | 13:09 | All D4 wires pulled | All four read 119,976-119,980. The no-power signature |
| R6 | 13:11 | D4 back, beacon off | 0-376. Normal |
| R7 | 13:13 | Beacon on, battery plugged in while running | Spun 1283° → 1644° in 11 s (about 32°/s). All four read 119,976-119,980 again |
| R8 | 13:14 | One sensor covered by hand | No change, all four still 119,976-119,980 (no power, not light) |
| R9 | 13:18 | On the table, battery off, beacon off | 0-760. Normal |
| R10 | 13:21 | Beacon pushed into A0's tube | A0 119,592-119,760, A1-A3 0-672. Real saturation; the tube blocks the others |
| R11 | 13:39-13:40 | Beacon off, battery off | A search step every 2 s. Each turn waits for its timeout (1.6 s) because the motor can't move |
| S1 | 14:41-14:53 | A0 sweep #1, by hand on the table | Good shape, but the curve shifted about +2.5° in the last 3 minutes (gyro drift and/or the robot sliding). Use only for noise |
| S2 | 14:54-14:57 | **A0 sweep #2, one quick pass, by hand on the table** | **The clean curve. The chart and Section 4 use this run** |

## 4. A0, A1 and A3 against angle (S2)

0 is where the robot was at reset, roughly facing the beacon. Frames are grouped to the nearest 0.5°. Full numbers are in [`s2_summary.csv`](test_data_2026-11-08/s2_summary.csv).

| Angle | Frames | A0 mean | A1 mean | A3 mean | A3 - A1 |
|---:|---:|---:|---:|---:|---:|
| -6.0 | 7 | 63,111 | 74,821 | 103,863 | +29,042 |
| -5.0 | 1 | 68,900 | 76,788 | 102,352 | +25,564 |
| -4.5 | 1 | 71,808 | 76,124 | 101,172 | +25,048 |
| -4.0 | 4 | 74,874 | 75,434 | 99,282 | +23,848 |
| -3.5 | 1 | 85,960 | 83,248 | 99,316 | +16,068 |
| -3.0 | 1 | 90,016 | 88,700 | 98,672 | +9,972 |
| -2.5 | 5 | 89,710 | 88,302 | 98,446 | +10,144 |
| -1.5 | 1 | 97,476 | 96,628 | 99,292 | +2,664 |
| -1.0 | 5 | 99,670 | 97,798 | 99,267 | +1,469 |
| 0.0 | 4 | 105,896 | 99,367 | 99,444 | +77 |
| +0.5 | 4 | 107,459 | 99,213 | 97,717 | -1,496 |
| +1.5 | 3 | 114,447 | 99,812 | 96,379 | -3,433 |
| +2.0 | 2 | 116,752 | 100,444 | 96,162 | -4,282 |
| +2.5 | 2 | 119,504 | 101,542 | 95,958 | -5,584 |
| +3.5 | 4 | 119,518 | 101,210 | 95,175 | -6,035 |
| +5.5 | 4 | 119,540 | 103,294 | 92,323 | -10,971 |
| +6.0 | 3 | 119,540 | 103,559 | 91,264 | -12,295 |
| +7.5 | 4 | 119,551 | 103,498 | 87,991 | -15,507 |
| +9.0 | 3 | 98,263 | 105,119 | 85,861 | -19,258 |
| +9.5 | 1 | 93,260 | 105,056 | 85,512 | -19,544 |
| +10.0 | 3 | 87,368 | 105,295 | 85,333 | -19,962 |
| +10.5 | 1 | 85,048 | 105,268 | 86,776 | -18,492 |
| +11.5 | 3 | 77,071 | 105,679 | 87,485 | -18,194 |
| +12.5 | 4 | 71,460 | 105,839 | 88,110 | -17,729 |
| +13.5 | 5 | 66,544 | 106,234 | 89,415 | -16,819 |
| +15.0 | 5 | 60,554 | 106,285 | 90,774 | -15,511 |
| +16.0 | 5 | 56,506 | 106,963 | 91,066 | -15,897 |

**What the curve means:**
- **A flat top 6.4° wide.** "A0 saturated" alone could leave the robot anywhere within ±3.2° of A0's centre, ±5.6 cm at 1 m. That always reaches the 16 cm ring, but the 8 cm bullseye only sometimes.
- **Even, straight slopes of about 6,000 per degree.** Noise is about ±1,000, so one frame on a slope places the robot to about ±0.2°. The midpoints between matching levels on the two sides (90k: -2.7° and +9.8°; 110k: +1.1° and +8.3°) put A0's axis at about +3.5° to +5°.
- **The fire level.** At 96,000 it fires anywhere in an 11° window. 110,000 narrows that to about 7°, and leaves 9,000 of margin below the lowest real saturated reading (119,268), so noise can't skip a shot. Aiming at the middle of the top (edge to edge, or a calibrated overshoot) is still needed for the bullseye.
- **Side sensors.** They see the beacon from almost any angle at 1 m (light through the plastic, inside the pipe, reflections). Use **A3 - A1** for left or right, not 1/0. It is reliable beyond about ±3°.

## 5. Not measured yet

- **Where the laser points** compared with A0's centre (the laser isn't mounted properly yet). Until then the code uses `LASER_OFFSET = 0`.
- Side sensors all the way round (T2), how far one turn can go (T1), hold during a shot (T5), angle memory (T6).
- Everything with the exam string (steel wire or fishing line).

## 6. Limits of this data

- Angles were set by hand on the table, and the robot may have slid a little. The gyro gives the angle, but a slide of 4 cm changes the beacon's direction by about 2.3° without the gyro seeing it.
- S2 took 3.5 minutes. At about 0.5°/min of gyro drift, the far end may be off by up to about 1.5°.
- The 0 in S2 is the start position, not an exact "facing the beacon" mark.
- The beacon is the test emitter on a second Nano, about 1 m away.

## 7. Files

| File | What it holds |
|---|---|
| [`raw_logs.txt`](test_data_2026-11-08/raw_logs.txt) | Every serial line from today (R0-R11, S1, S2), with a note for each run |
| [`s1_sweep.csv`](test_data_2026-11-08/s1_sweep.csv), [`s2_sweep.csv`](test_data_2026-11-08/s2_sweep.csv) | Every frame of each sweep |
| [`s2_summary.csv`](test_data_2026-11-08/s2_summary.csv) | S2 grouped per 0.5°: mean, min, max for A0-A3, and A3 - A1 |
| [`a0_sweep_s2.svg`](test_data_2026-11-08/a0_sweep_s2.svg) | The chart above. Hover a point for its numbers |
| [`make_chart.py`](test_data_2026-11-08/make_chart.py) | Rebuilds the CSVs and the chart from `raw_logs.txt` (`python3 make_chart.py`, standard library only) |
