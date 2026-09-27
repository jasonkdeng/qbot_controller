# Front-right servo and IK diagnostics

The `single_leg_ik` firmware uses the current front-right calibration from
`RobotConfig.h`. It starts with PWM off. Keep the leg supported and use external
servo power with a shared ESP32 ground.

```powershell
pio run -e single_leg_ik -t upload --upload-port COM3
pio device monitor -e single_leg_ik --port COM3
```

## Test individual joints first

The mapping now uses **1000..2000 us** per joint. Numeric angle neutrals are
preserved, but their physical positions change with this narrower mapping.
Recheck alignment before loading the leg or testing IK.

Send these individually, waiting for physical settling after each command:

```text
arm
joint thigh 0
joint thigh -15
joint thigh -30
joint thigh 0
joint thigh 15
joint thigh 30
joint thigh 0
off
```

`arm` only enables commands. A joint's first attachment writes its validated
configured neutral immediately, then its requested target. `joint` controls only
that joint; other outputs remain as they were. Use `off` before switching joints
if you want only one servo attached. Direct joint commands invalidate the last
Cartesian-position display because it no longer describes the command.

Expected thigh diagnostic output with neutral 86, direction -1:

| Joint angle | Raw angle | Pulse (us) |
| --- | --- | --- |
| -30 | 116 | 1644 |
| -15 | 101 | 1561 |
| 0 | 86 | 1477 |
| 15 | 71 | 1394 |
| 30 | 56 | 1311 |

The integer mapping truncates fractional microseconds. Non-finite or out-of-limit
commands are rejected, not clamped. Negative mathematical angles
are valid only within that joint's range (for example, thigh -30 is valid).

## Check IK without moving

```text
check 0 0 20
```

This prints requested XYZ, all joint/raw angles, all calculated pulses, and each
joint's validation result. The 20 cm target is now **rejected**: thigh -50.29
exceeds the temporary -45 degree minimum. The IK geometry was not changed.
The normal controller also leaves PWM off when this startup pose is invalid.

A target that passes the current mathematical limits is:

```text
check 0 0 22
```

Passing validation does not establish physical calibration or clearance. After
individual tests and neutral checks, use the supported leg:

```text
arm
move 0 0 22
```

Each `move` solves IK once, reports all outputs, validates every joint before any
attachment/write, and sends one target through the Leg command path. There is no
interpolation, smoothing, or gait. Check small changes before moving, such as
`check 1 0 22` and `check 0 1 22`. Coordinates are cm from the hip roll axis:
+X forward, +Y outward, +Z downward. The physical travel path is not checked.

`status` reports commanded state, not measured position. `stop` leaves the current
target active; `off` detaches PWM and releases holding torque. Wait for physical
settling between commands; the program cannot detect completion.

See [SERVO_OUTPUT_SAFETY.md](SERVO_OUTPUT_SAFETY.md) for exact calibration, tests,
and remaining attachment/power/mechanical limitations.
