# Servo output safety refactor

## Configuration in use

| Joint | GPIO | Neutral | Direction | Joint range | Pulse endpoints | Reachable command pulses |
| --- | --- | --- | --- | --- | --- | --- |
| Front-right hip | 18 | 97 degrees | -1 | -20..20 degrees | 1000..2000 us | 1427..1650 us |
| Front-right thigh | 19 | 86 degrees | -1 | -45..45 degrees | 1000..2000 us | 1227..1727 us |
| Front-right knee | 21 | 95 degrees | +1 | 0..80 degrees | 1000..2000 us | 1527..1972 us |

Neutral attachment pulses are hip 1538, thigh 1477, knee 1527 us. Values use
integer truncation, as before. The other three legs remain disabled with -1 pins
and conservative 1000..2000 us endpoints.

Hip 97/-1 differs from the older 88/+1 calibration. Its current value is preserved,
as are thigh 86/-1 and knee 95/+1; physically re-verify them if the changes were not
intentional. Even intentionally measured angle neutrals need rechecking because
changing the pulse endpoints changes physical positions. These are diagnostic
bounds, not a claim of measured mechanical safety.

## Behavior

- Joint calculates raw angle, pulse and a rejection reason without output.
- Non-finite angles, malformed calibration, invalid pulse endpoints, joint/raw
  angle violations, and out-of-range pulses are rejected without clamping.
- Pulse endpoints must fit ESP32Servo's supported 500..2500 us envelope. That
  envelope is not used to map commands; each joint's configured endpoints are.
- Attach first validates mathematical zero and immediately writes the configured
  neutral pulse after library attachment. Reattaching an attached Joint is a no-op.
- Leg validates every neutral before attachment and detaches all joints if a
  hardware attachment fails. Every target validates all three outputs before any
  joint command. This guarantees rejection before writes for invalid targets;
  PWM writes themselves are sequential, not electrically simultaneous.
- The IK test prints requested XYZ, joint/raw angles, pulses, and per-joint
  rejection reasons. `joint thigh -30` bypasses IK and tests the actual Joint path.
- The calibration tool shares Joint's pulse mapping, checks its current joint
  bounds, and exports seven-field joint configs including pulse endpoints.
- The old `(0,0,20)` target is rejected by the new thigh limit (-50.29 < -45).
  Normal startup leaves that leg's PWM off instead of moving to neutral first.
  `(0,0,22)` passes mathematical validation but still needs physical verification.

## Before this change

The thigh used raw angle `86 - jointAngle`, then the global 500..2500 us mapping.
At -80, -45, 0, +45, +80 degrees it sent 2344, 1955, 1455, 955, 566 us respectively.
Leg checked mathematical and raw angles, but lacked final pulse/config validation.
Attach connected PWM without immediately writing a configured neutral.

## Verification

Run host tests on Windows with:

```powershell
./test/run_native.ps1
# Or use your own native C++17 compiler:
./test/run_native.ps1 -Compiler 'C:/path/to/g++.exe'
```

The runner uses the compiler in ignored `.pio/host-tools/w64devkit` if present,
otherwise `g++` on PATH. It runs the existing kinematics/motion/joint tests plus
servo safety and serial tests. The latter substitute only Arduino/Servo hardware
boundaries, recording calls from the real Joint, Leg, IK and Serial command code.

```powershell
pio run -e esp32_wroom -e leg_calibration -e single_leg_ik
```

## Limits of the result

No physical settling, PWM waveform, current or position was measured. Software
cannot remove servo backlash, binding, inadequate supply current, voltage drops,
ground faults or internal servo hunting. Measure the supply under load and the
PWM waveform if instability persists during isolated joint tests.

The library enables PWM before the neutral write. The initialization interval is
shortened, but a transient-free electrical startup is not proven. Hardware attach
failure can occur after other joints have received neutral; rollback detaches
outputs but cannot undo physical movement. Existing four-leg stance commands are
still sequential across legs; this change guarantees validation per leg only.

A future physically calibrated mapping may use `neutralPulseUs`, `direction`,
`usPerDegree` and pulse bounds. This change retains the requested angle-neutral
mapping. IK equations, link lengths, branch and coordinates remain untouched.
