# What changed tonight (7 Nov 2026)

## In very simple words

- **PD folder:** a robot that only turns. You type a number, like 90, and it spins to that angle. It has no eyes and no laser.
- **main_see_fire now:** we gave that turning robot eyes (4 IR sensors) and a laser. Its eyes tell it which way to turn. When the front eye sees the beacon, it fires the laser.

## What we added on top of the PD code

1. **Only fire when holding still on target.** The robot checks it is within 1° of where it wants to point before it fires.
2. **Trust the front eye.** If the front sensor (A0) already sees the beacon, the robot stops where it is and fires. It no longer keeps turning to an old target. Before this fix it got stuck and never fired.
3. **Safer stop.** When the motor brakes, both control wires go low, so the wheel can't suddenly spin at full speed.
4. **It tells us what it is thinking.** Every line on the Serial Monitor shows the target angle, the gyro angle and its choice (FIRE, TURN or NONE).

## Hardware we found tonight

- The motor needs the 12 V battery. USB alone can't drive it.
- A loose ground wire on the motor driver (PMODE) made the motor turn the wrong way or run away. It is fixed now.
- **The gyro (MPU6050) is broken.** It always reads 0, so real turning can't be tested until we get a new one.

## Status

- The laser fires when the beacon is in front of A0. This was tested without the motor battery.
- Next: fit a new gyro, then test real turning with the robot hanging.
