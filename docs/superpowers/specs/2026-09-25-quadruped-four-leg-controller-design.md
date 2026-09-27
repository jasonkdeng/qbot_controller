# Quadruped Four-Leg Controller Design

## Goal
Refactor the current single-leg ESP32/PlatformIO controller into a reusable four-leg locomotion framework supporting 12 servos, shared 3D inverse kinematics, per-leg calibration, non-blocking Cartesian motion, and clean extension points for gait control and future IMU/LiDAR/depth-camera integration.

## Confirmed Geometry
- Hip roll axis runs forward/backward.
- Hip roll axis to thigh pitch axis offset: 5.92 cm, pointing straight down at hip neutral.
- Thigh length: 7.08 cm.
- Shin length: 11.0 cm.
- Local leg coordinates: +X forward, +Y outward from the robot body, +Z downward.

## Architecture

### `include/RobotConfig.h`
Stores shared geometric constants, servo PWM bounds, update rates, leg IDs, and placeholder GPIO configuration.

### `include/Joint.h` / `src/Joint.cpp`
`Joint` owns one servo's:
- GPIO pin
- neutral angle
- direction (+1/-1)
- joint-space limits
- attach state
- servo write/validation logic

No inverse kinematics belongs in this class.

### `include/Leg.h` / `src/Leg.cpp`
`Leg` owns:
- hip, thigh, and knee `Joint`s
- leg name/ID
- current Cartesian foot position
- target/motion state
- initialization and joint-command helpers

Each leg uses the same local coordinate convention. Mirroring is handled only through per-joint direction/calibration.

### `include/Kinematics.h` / `src/Kinematics.cpp`
Shared stateless IK functions:
- 2D thigh/knee IK
- 3D hip/thigh/knee IK
- workspace/reachability checks

The 3D solver uses:
- `radialDistance = sqrt(y^2 + z^2)`
- `hipAngle = atan2(y, z)`
- `legPlaneZ = radialDistance - HIP_OFFSET`
- existing 2D IK for `(x, legPlaneZ)`

### `include/Motion.h` / `src/Motion.cpp`
Non-blocking Cartesian motion using `millis()`:
- start move from current XYZ to target XYZ
- smoothstep interpolation
- periodic update
- no `delay()` calls

This becomes the base for future stance/swing trajectories and gait generation.

### `src/main.cpp`
Responsible only for:
- creating/configuring four legs
- attaching enabled legs
- running update loops
- basic single-leg test / standing test
- future subsystem hooks (`updateGait`, `updateIMU`, `updateLidar`, `updateDepthCamera`, `updateSafety`)

## Leg Configuration
Front-right uses current known calibration/pins. Front-left, rear-right, and rear-left use placeholder pins until wiring is chosen. Each leg will eventually define:
- hip pin / neutral / direction / limits
- thigh pin / neutral / direction / limits
- knee pin / neutral / direction / limits

Placeholder pins are represented as `-1`; joints with unassigned pins are not attached or commanded.

## Safety Rules
1. Validate all three IK joint angles before moving any servo.
2. Validate converted servo angles before output.
3. Never partially execute a Cartesian target when one joint is invalid.
4. No blocking `delay()` in locomotion code.
5. Disabled/unconfigured legs are skipped safely.

## First Implementation Milestone
The first implementation will include:
- modular PlatformIO file structure
- four `Leg` instances
- known front-right calibration
- placeholder configuration for the other three legs
- shared 3D IK
- non-blocking per-leg Cartesian interpolation
- single-leg motion test
- static four-leg stance API (only active once legs are configured)
- sensor/update hooks, but no gait, IMU, LiDAR, or depth-camera behavior yet

## Deferred Work
Not included in this milestone:
- crawl gait
- trot gait
- body pose compensation
- IMU stabilization
- LiDAR/depth-camera processing
- SBC communication protocol
- turning/body velocity controller

These should be layered on only after all four legs are calibrated and can consistently reach the same logical local XYZ targets.

## Success Criteria
- Front-right leg behaves the same as the current working single-leg implementation.
- Additional legs can be enabled by filling in pin/calibration values only.
- All four legs share the same IK code.
- Mirrored hardware requires no changes to kinematic equations.
- Cartesian moves are smooth and non-blocking.
- Main loop remains available for future sensor and safety updates.
