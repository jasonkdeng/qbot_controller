You are working on an ESP32-based quadruped robot project. Before changing code, familiarize yourself with the architecture, kinematics, mechanical assumptions, calibration conventions, and development history below. Treat this as project context and preserve the established conventions unless there is a strong technical reason to change them.

# Project Goal

Build a small 4-legged quadruped robot with:

- 3 servos per leg
  - hip
  - thigh
  - knee
- 12 servos total
- ESP32 for low-level locomotion and actuator control
- future support for:
  - IMU
  - LiDAR
  - depth camera
  - onboard SBC such as Raspberry Pi / Jetson for higher-level perception and navigation
- future autonomous navigation and obstacle avoidance

The project is currently focused on locomotion, kinematics, and creating a clean software architecture before adding perception.

# Hardware

Servos:
- MG996R hobby servos
- controlled using 50 Hz servo PWM
- ESP32Servo library
- servos are externally powered
- ESP32 and servo power supply must share ground

Current known front-right leg pins:
- hip: GPIO 18
- thigh: GPIO 19
- knee: GPIO 21

The other three legs do not yet have assigned ESP32 GPIO pins. Use placeholder pins such as `-1` until wiring is finalized.

# Current Mechanical Geometry

The leg configuration has changed during development.

IMPORTANT: do not use the old hip-yaw geometry.

The CURRENT configuration is:

- hip axis runs FORWARD/BACKWARD
- hip therefore acts as a ROLL joint
- hip rotates the leg in the Y-Z plane
- thigh and knee form a planar mechanism that moves in the X-Z plane after hip rotation

Coordinate convention:

+X = robot forward
+Y = outward from the robot body
+Z = downward

For every leg, use the same LOCAL coordinate convention:
- +X always means robot forward
- +Y always means outward from that side of the robot
- +Z always means downward

Do not mirror the IK equations for left/right legs.
Handle mirrored physical servo installations using per-joint servo `direction` values.

# Link Lengths

Current measured geometry:

- hip roll axis -> thigh pitch axis:
  5.92 cm

- thigh pitch axis -> knee axis:
  7.08 cm

- knee axis -> toe:
  11.0 cm

So:

HIP_OFFSET   = 5.92 cm
THIGH_LENGTH = 7.08 cm
SHIN_LENGTH  = 11.0 cm

The hip-to-thigh offset points STRAIGHT DOWN when the hip is at its neutral angle.

The hip roll axis and thigh pitch axis are perpendicular and separated by 5.92 cm.

# Front-Right Calibration History

Previously calibrated single-leg values include:

HIP_NEUTRAL   = 88 degrees
THIGH_NEUTRAL = 86 degrees
KNEE_NEUTRAL  = 94 degrees

Known direction convention for the previously tested leg:

HIP_DIRECTION   = +1
THIGH_DIRECTION = -1
KNEE_DIRECTION  = +1

These direction values map mathematical joint angles into physical servo angles using:

servoAngle = neutral + direction * jointAngle

Do not bake servo mounting direction into IK math.

Joint-space limits used during testing:

HIP:
- approximately -20 to +20 degrees

THIGH:
- approximately -80 to +80 degrees

KNEE:
- approximately 0 to +95 degrees

Treat these as current working limits, not immutable mechanical truth. Other legs will need their own calibration values.

# Servo Calibration Philosophy

The project intentionally separates:

1. mathematical joint angle
2. physical servo command

Example:

jointAngle = 0 degrees

means the defined kinematic neutral, regardless of whether the actual servo command is:

86 degrees
94 degrees
88 degrees
etc.

Conversion:

servoAngle = neutral + direction * jointAngle

This allows the IK and gait code to remain hardware-independent.

Each of the 12 servos should eventually have:

- GPIO pin
- neutral servo angle
- direction (+1 or -1)
- minimum mathematical joint angle
- maximum mathematical joint angle

# Current 3D IK

The current hip configuration is a roll joint around the X axis.

Given desired toe position:

(x, y, z)

first solve the hip in the Y-Z plane.

Radial distance from hip roll axis:

radialDistance = sqrt(y^2 + z^2)

Hip roll:

hipAngle = atan2(y, z)

Because the fixed 5.92 cm hip-to-thigh offset points along the same radial direction when neutral:

legPlaneZ = radialDistance - HIP_OFFSET

Then reuse the 2D thigh/knee IK:

inverseKinematics2D(
    x,
    legPlaneZ,
    thighAngle,
    kneeAngle
)

The target is invalid if:

radialDistance <= HIP_OFFSET

# 2D IK

The thigh/knee subsystem uses:

+X = forward
+Z = downward from thigh pitch axis

Distance:

d = sqrt(x^2 + z^2)

Reachability:

abs(SHIN_LENGTH - THIGH_LENGTH)
<= d <=
THIGH_LENGTH + SHIN_LENGTH

Knee is solved with the law of cosines.

Convention:

kneeAngle = 0 degrees when the leg is straight

The project already tested 2D IK successfully.

The single leg was able to:
- move to Cartesian targets
- trace an oval
- follow basic stepping-style waypoints

This validated:
- link lengths
- thigh direction
- knee direction
- IK branch
- Cartesian trajectory concept

# Important Historical Debugging Lessons

Several issues were already discovered and should not be reintroduced.

1. Servo limits must not silently clamp IK results.

Bad:
constrain calculated servo angles and continue.

Good:
reject the entire foot target if any joint would exceed its limit.

Otherwise one joint moves while another does not, and the foot does not follow the requested Cartesian trajectory.

2. Validate ALL THREE calculated joint commands before commanding any servo.

Do not:
- move hip
- discover knee is invalid
- abort

Instead:
- solve all joint angles
- validate all joint limits
- validate all resulting servo commands
- only then command the servos

3. Serial printing affected timing during early tests.

Heavy `Serial.print()` calls accidentally slowed trajectory loops.

Motion timing should be explicit, not dependent on logging.

4. MG996R servos have backlash and deadband.

Very small Cartesian trajectories may appear jerky even when IK is correct.

5. `Servo.write(angle)` was originally used for calibration.

Later code moved toward `writeMicroseconds()` for finer resolution.

Typical mapping currently used:

0 deg   -> 500 us
180 deg -> 2500 us

Do not command invalid angles such as -60 deg or 260 deg through this mapping.

Joint limits are relative to neutral.
Raw servo angles still need to remain in valid physical servo range.

6. The old hip geometry was incorrect.

Earlier versions treated the hip as:
- vertical-axis yaw
or
- an intersecting roll joint

Those models are obsolete.

CURRENT geometry:
- hip axis = forward/backward
- hip offset = 5.92 cm downward
- hip = roll joint

# PlatformIO Migration

The project has migrated from Arduino IDE to:

VS Code + PlatformIO

Framework:
Arduino

Library:
ESP32Servo

Expected PlatformIO configuration includes:

framework = arduino

lib_deps =
    madhephaestus/ESP32Servo

monitor_speed = 115200

Use:

#include <Arduino.h>

in C++ source files.

# Intended Software Architecture

Avoid putting everything in `main.cpp`.

Current target structure:

include/
    RobotConfig.h
    Joint.h
    Leg.h
    Kinematics.h
    Motion.h

src/
    main.cpp
    Joint.cpp
    Leg.cpp
    Kinematics.cpp
    Motion.cpp

Responsibilities:

RobotConfig
- common geometry constants
- PWM constants
- global robot configuration values

Joint
- Servo object
- pin
- neutral
- direction
- joint limits
- conversion from mathematical angle to physical servo command
- validation
- servo output

Leg
- hip Joint
- thigh Joint
- knee Joint
- current Cartesian foot position
- motion state
- name / ID
- enabled state

Kinematics
- shared 2D IK
- shared 3D IK
- no leg-specific servo calibration logic

Motion
- non-blocking Cartesian interpolation
- based on millis()
- no long blocking delays

main.cpp
- configure legs
- attach enabled legs
- initialize stance
- run update loops
- future sensor hooks

# Four-Leg Architecture

Leg IDs:

FRONT_RIGHT
FRONT_LEFT
REAR_RIGHT
REAR_LEFT

The front-right leg currently has real calibration values.

The other three legs should exist in the codebase but remain disabled until:
- pins are assigned
- neutral values are calibrated
- servo directions are determined
- safe limits are measured

Do not assume mirrored legs have identical raw servo values.

# Per-Leg Coordinate Convention

Every leg should receive logical Cartesian targets such as:

setFootPosition(
    leg,
    x,
    y,
    z
)

where:

x > 0 = forward
y > 0 = outward
z > 0 = downward

The caller should not care whether a physical servo is reversed.

For example:

FRONT_RIGHT +X
FRONT_LEFT  +X
REAR_RIGHT  +X
REAR_LEFT   +X

must all mean physically forward.

This consistency is essential before gait development.

# Motion Architecture

Long-term motion should be NON-BLOCKING.

Avoid:

delay(1000)

inside gait logic.

Preferred architecture:

loop()
{
    updateGait();
    updateAllLegs();
    updateIMU();
    updateLidar();
    updateDepthCamera();
    updateSafety();
}

Leg motion should use:

millis()

and interpolate between:

start position
target position
duration

Smoothstep interpolation has already been considered:

s = t*t*(3 - 2*t)

Cartesian interpolation should happen before IK.

# Planned Gait Progression

Do NOT jump directly to an aggressive trot.

Development order:

1. Calibrate all 12 servos.
2. Verify each individual leg can execute the same logical XYZ targets.
3. Put all four legs into a static standing pose.
4. Verify coordinated four-leg movements.
5. Implement a slow crawl gait.
6. Validate stability and body support.
7. Add IMU feedback.
8. Implement trot using diagonal pairs:
   - front-left + rear-right
   - front-right + rear-left
9. Tune stride length, step height, stance height, and timing.

# Future Sensor Architecture

The ESP32 should remain the low-level motion controller.

Expected future split:

Higher-level SBC
(Raspberry Pi / Jetson / similar)
        |
        | UART / CAN / USB / other link
        v
ESP32
        |
        +-- gait execution
        +-- servo control
        +-- IMU
        +-- watchdog
        +-- safety
        +-- possibly lightweight sensor handling

Depth camera and heavy LiDAR processing should generally happen on the SBC rather than directly on the ESP32.

High-level autonomy should eventually send commands such as:

desiredForwardVelocity
desiredLateralVelocity
desiredYawRate

or possibly:

desired foothold XYZ

It should NOT directly send raw servo angles.

Desired control stack:

LiDAR / depth camera
        ↓
perception
        ↓
navigation / obstacle avoidance
        ↓
desired robot velocity
        ↓
gait planner
        ↓
four Cartesian foot targets
        ↓
3D IK
        ↓
12 joint angles
        ↓
servo calibration layer
        ↓
12 MG996R servos

# Current Development State

Working:
- one physical leg
- servo calibration
- thigh/knee 2D IK
- new 3DOF geometry understood
- hip roll model
- Cartesian target concept
- single-leg movement
- oval tracing
- initial stepping trajectories
- PlatformIO migration
- modular architecture direction

In progress:
- four-leg software framework
- calibration of remaining three legs
- GPIO assignments
- static four-leg stance
- non-blocking motion integration

Not implemented yet:
- complete four-leg locomotion
- crawl gait
- trot gait
- IMU stabilization
- LiDAR
- depth camera
- SBC communication
- obstacle avoidance
- body pose control

# Engineering Priorities

When modifying this project:

1. Preserve the coordinate conventions.
2. Keep kinematics separate from hardware calibration.
3. Do not duplicate IK per leg.
4. Do not silently clamp invalid IK targets.
5. Validate entire IK solutions before moving servos.
6. Prefer non-blocking control.
7. Keep future perception integration in mind.
8. Avoid unnecessary abstraction before four-leg locomotion works.
9. Favor debuggability and clear serial diagnostics.
10. Never assume the physical robot matches a mathematical model without validation.

# Immediate Next Milestone

The immediate goal is NOT autonomous navigation.

The next milestone is:

A four-leg controller in PlatformIO where:
- all four Leg objects exist
- each leg has independent servo calibration
- placeholder pins are allowed for uncalibrated legs
- all legs share the same 3D IK
- each leg accepts local XYZ targets
- movement is non-blocking
- a safe static standing pose can be commanded
- the codebase is ready for the first crawl gait

Before changing code, inspect the existing PlatformIO project and map the current implementation against this context. Do not rewrite working pieces unnecessarily. Preserve proven behavior and refactor incrementally.