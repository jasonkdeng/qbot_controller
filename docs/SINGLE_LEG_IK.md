# Front-right 3D IK bench test

This firmware uses only `RobotConfig::FRONT_RIGHT` and the existing shared IK.
Keep the body supported and the foot clear of the bench. Other legs are never
attached. Use external servo power with a common ESP32 ground.

```powershell
pio run -e single_leg_ik -t upload --upload-port COM3
pio device monitor -e single_leg_ik --port COM3
```

PWM starts detached. Type `help` if you missed the startup message.
Send commands individually, waiting for `Move complete` between moves.

1. `check 0 0 20` prints hip/thigh/knee mathematical angles and raw servo commands
   without attaching or moving anything. With the saved calibration (88/+1,
   86/-1, 95/-1), expect joint angles near 0, -50.29, 79.97 degrees and raw
   commands near 88, 136.29, 15.03 degrees. Verify these agree with the physical
   calibration before arming.
2. `arm` attaches all three front-right servos and immediately commands the
   configured stance, currently `(0, 0, 20)` cm. This initial movement cannot be
   interpolated from the actual pose because the robot has no position feedback.
3. Test each Cartesian axis separately:

   | Command | Expected foot motion |
   | --- | --- |
   | `move 1 0 20` | 1 cm forward |
   | `move 0 0 20` | Back to starting pose |
   | `move 0 1 20` | 1 cm outward; exercises hip roll |
   | `move 0 0 20` | Back to starting pose |
   | `move 0 0 19` | 1 cm upward, because +Z is down |
   | `move 0 0 20` | Back to starting pose |
   | `move 1 1 20` | Combined forward and outward |
   | `move 0 0 20` | Back to starting pose |

4. `off` detaches PWM and releases holding torque. Support the leg first.

Positions are in cm from the hip roll axis, with +X forward, +Y outward, and +Z
down. Each move takes two seconds, uses smoothstep Cartesian interpolation, and
updates every 20 ms. Bench-test moves are limited to 3 cm from the last commanded
position. The path is sampled before movement and every actual output is checked
against joint and servo limits. Invalid targets are rejected without clamping.

`stop` cancels interpolation and holds the last command. `status` reports the
last commanded position, not measured feedback. `check 0 0 5` should reject an
unreachable target without changing output. If motion is reversed or the foot
does not follow the expected axis, stop and inspect calibration/mechanical
alignment before increasing travel.

The normal static-stance firmware remains the default build. Restore it with:

```powershell
pio run -e esp32_wroom -t upload --upload-port COM3
```
