# Quadruped Four-Leg Controller Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Refactor the working single-leg ESP32 controller into a modular four-leg PlatformIO framework with shared 3D IK, per-leg calibration, non-blocking Cartesian motion, and future sensor hooks.

**Architecture:** `Joint` owns servo calibration/output, `Leg` composes three joints plus Cartesian motion state, `Kinematics` provides shared stateless 2D/3D IK, and `Motion` provides non-blocking interpolation. `main.cpp` only configures the four legs and runs update hooks.

**Tech Stack:** PlatformIO, Arduino framework for ESP32, C++17, ESP32Servo, PlatformIO native tests for stateless kinematics/motion math where practical.

**Spec:** `docs/superpowers/specs/2026-09-25-quadruped-four-leg-controller-design.md`

## Global Constraints

- Hip roll axis runs forward/backward.
- Hip roll axis to thigh pitch axis offset: `5.92 cm`, pointing straight down at hip neutral.
- Thigh length: `7.08 cm`.
- Shin length: `11.0 cm`.
- Local leg coordinates: `+X` forward, `+Y` outward from the robot body, `+Z` downward.
- Mirroring is handled only through per-joint direction/calibration; IK equations are shared across all legs.
- Placeholder GPIO pins use `-1`; unconfigured joints are never attached or commanded.
- Validate all three joint angles and converted servo angles before commanding any servo.
- Cartesian locomotion code must not use `delay()`.
- Front-right retains the current known calibration; other three legs remain disabled/unconfigured until pin/calibration values are supplied.

## Review Focus

- Target exactly at or inside the 5.92 cm hip offset must be rejected without commanding any servo.
- Targets outside thigh+shin reach (`18.08 cm`) or inside minimum reach (`3.92 cm`) must be rejected consistently.
- A leg with any `-1` GPIO must remain unattached and ignore motion commands safely.
- Mirrored direction values must alter only servo mapping, never the shared IK result.
- A Cartesian target with one invalid joint must not partially move the other two joints.

---

## File Map

- Create `include/RobotConfig.h` — shared geometry, PWM bounds, leg IDs, update constants, and placeholder configuration values.
- Create `include/Joint.h` / `src/Joint.cpp` — one-servo abstraction and validation/output.
- Create `include/Leg.h` / `src/Leg.cpp` — three-joint leg abstraction and Cartesian state.
- Create `include/Kinematics.h` / `src/Kinematics.cpp` — stateless 2D/3D IK.
- Create `include/Motion.h` / `src/Motion.cpp` — non-blocking Cartesian move state and updates.
- Replace `src/main.cpp` — four-leg configuration, startup, update loop, temporary single-leg test, and future sensor hooks.
- Create `test/test_kinematics/test_main.cpp` — native unit tests for 2D/3D IK and reachability.
- Create `test/test_motion/test_main.cpp` — native unit tests for interpolation math/state progression where practical.
- Modify `platformio.ini` — add `ESP32Servo`, enable project C++ standard if needed, and add a `native` test environment.

---

### Task 1: Shared Configuration and Stateless Kinematics

**Files:**
- Create: `include/RobotConfig.h`
- Create: `include/Kinematics.h`
- Create: `src/Kinematics.cpp`
- Create: `test/test_kinematics/test_main.cpp`
- Modify: `platformio.ini`

**Interfaces:**
- Consumes: no project-local interfaces.
- Produces: `struct JointAngles { float hip; float thigh; float knee; };`, `bool solveIK2D(float x, float z, float &thighAngle, float &kneeAngle);`, `bool solveIK3D(float x, float y, float z, JointAngles &out);`.

- [ ] **Step 1: Write failing native tests for confirmed geometry and reachability**

  Cover:
  - `solveIK3D(0, 0, 17, out)` succeeds and produces `hip ~= 0`.
  - `solveIK3D(0, 2, 17, out)` succeeds and produces `hip ~= atan2(2,17)` in degrees.
  - a target with `sqrt(y^2+z^2) <= 5.92` fails.
  - a 2D target beyond `7.08 + 11.0 = 18.08 cm` fails.
  - a 2D target inside `abs(11.0 - 7.08) = 3.92 cm` fails.

- [ ] **Step 2: Run the kinematics test and verify it fails**

  Run: `pio test -e native -f test_kinematics`

  Expected: FAIL because the kinematics interfaces do not exist yet.

- [ ] **Step 3: Implement `RobotConfig.h` constants and `Kinematics` interfaces**

  Exact constants:
  - `HIP_OFFSET_CM = 5.92f`
  - `THIGH_LENGTH_CM = 7.08f`
  - `SHIN_LENGTH_CM = 11.0f`
  - local axes documented as `+X` forward, `+Y` outward, `+Z` down.

  `solveIK3D()` must compute:
  - `radialDistance = sqrt(y*y + z*z)`
  - `hip = degrees(atan2(y, z))`
  - `legPlaneZ = radialDistance - HIP_OFFSET_CM`
  - then call `solveIK2D(x, legPlaneZ, ...)`.

- [ ] **Step 4: Run kinematics tests and verify they pass**

  Run: `pio test -e native -f test_kinematics`

  Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add include/RobotConfig.h include/Kinematics.h src/Kinematics.cpp test/test_kinematics/test_main.cpp platformio.ini
git commit -m "feat: add shared quadruped kinematics"
```

---

### Task 2: Joint and Leg Abstractions

**Files:**
- Create: `include/Joint.h`
- Create: `src/Joint.cpp`
- Create: `include/Leg.h`
- Create: `src/Leg.cpp`

**Interfaces:**
- Consumes: `JointAngles`, `solveIK3D(...)`, PWM constants from `RobotConfig.h`.
- Produces: `Joint::configure(...)`, `Joint::attach()`, `Joint::validate(float jointAngle) const`, `Joint::command(float jointAngle)`, `Leg::configure(...)`, `Leg::attach()`, `Leg::setFootPosition(float x, float y, float z)`, and Cartesian current-position accessors.

- [ ] **Step 1: Add host-testable validation coverage for joint mapping**

  Add tests for a pure helper exposed from `Joint.h` or a small validation helper that proves:
  - `servoAngle = neutral + direction * jointAngle`.
  - direction `+1` and `-1` produce mirrored servo outputs for the same joint angle.
  - joint-space min/max are enforced.
  - converted servo angles outside `0..180` are rejected.

- [ ] **Step 2: Run validation tests and verify they fail**

  Run: `pio test -e native`

  Expected: FAIL because joint mapping/validation does not exist yet.

- [ ] **Step 3: Implement `Joint`**

  `Joint` owns:
  - `Servo servo`
  - `int pin`
  - `float neutral`
  - `int direction`
  - `float minAngle`, `maxAngle`
  - `bool attached`

  Rules:
  - `pin == -1` means unconfigured and `attach()` must return false without touching hardware.
  - `command()` validates joint and servo angles before PWM output.
  - no IK logic belongs in `Joint`.

- [ ] **Step 4: Implement `Leg` with all-or-nothing Cartesian validation**

  `Leg::setFootPosition(x,y,z)` must:
  1. solve shared 3D IK,
  2. validate hip, thigh, and knee commands first,
  3. command none of the servos if any validation fails,
  4. command all three only after the complete target is valid,
  5. update current Cartesian position only after successful output.

- [ ] **Step 5: Add tests for disabled/unconfigured legs and all-or-nothing validation**

  Exercise:
  - a leg containing any placeholder/unconfigured joint cannot be attached as an active leg,
  - an invalid knee target cannot leave hip/thigh reported as successfully commanded,
  - direction changes do not alter `solveIK3D()` output.

- [ ] **Step 6: Run all native tests and verify they pass**

  Run: `pio test -e native`

  Expected: PASS.

- [ ] **Step 7: Build the ESP32 target**

  Run: `pio run`

  Expected: SUCCESS with `ESP32Servo` linked for the embedded environment.

- [ ] **Step 8: Commit**

```bash
git add include/Joint.h src/Joint.cpp include/Leg.h src/Leg.cpp test platformio.ini
git commit -m "feat: add reusable joint and leg abstractions"
```

---

### Task 3: Non-Blocking Cartesian Motion

**Files:**
- Create: `include/Motion.h`
- Create: `src/Motion.cpp`
- Create: `test/test_motion/test_main.cpp`
- Modify: `include/Leg.h`
- Modify: `src/Leg.cpp`

**Interfaces:**
- Consumes: `Leg::setFootPosition(float,float,float)` and current Cartesian position.
- Produces: `struct FootPosition { float x; float y; float z; };`, `struct LegMotionState`, `void startLegMove(Leg&, FootPosition target, uint32_t durationMs, uint32_t nowMs);`, `void updateLegMotion(Leg&, uint32_t nowMs);`, `bool isLegMoving(const Leg&)`.

- [ ] **Step 1: Write failing interpolation tests**

  Cover:
  - start position is preserved at `t=0`,
  - target is reached exactly at `elapsed >= duration`,
  - halfway smoothstep produces the expected midpoint factor `0.5`,
  - zero-duration moves resolve immediately without division by zero,
  - no API uses `delay()`.

- [ ] **Step 2: Run motion tests and verify they fail**

  Run: `pio test -e native -f test_motion`

  Expected: FAIL because motion APIs do not exist yet.

- [ ] **Step 3: Implement non-blocking motion state**

  Use `millis()` supplied by caller through `nowMs`; keep interpolation deterministic for tests. Use smoothstep `s = t*t*(3 - 2*t)` and call `Leg::setFootPosition()` with interpolated XYZ.

- [ ] **Step 4: Run motion tests and verify they pass**

  Run: `pio test -e native -f test_motion`

  Expected: PASS.

- [ ] **Step 5: Build ESP32 target**

  Run: `pio run`

  Expected: SUCCESS and no locomotion module contains `delay()`.

- [ ] **Step 6: Commit**

```bash
git add include/Motion.h src/Motion.cpp include/Leg.h src/Leg.cpp test/test_motion/test_main.cpp
git commit -m "feat: add non-blocking leg motion"
```

---

### Task 4: Four-Leg PlatformIO Integration

**Files:**
- Replace: `src/main.cpp`
- Modify: `include/RobotConfig.h`

**Interfaces:**
- Consumes: `Joint`, `Leg`, `solveIK3D`, `startLegMove`, `updateLegMotion`.
- Produces: four configured `Leg` instances (`frontRight`, `frontLeft`, `rearRight`, `rearLeft`) and main update hooks `updateGait()`, `updateIMU()`, `updateLidar()`, `updateDepthCamera()`, `updateSafety()`.

- [ ] **Step 1: Define four-leg configuration**

  Front-right uses the current known values:
  - pins `18`, `19`, `21`
  - neutral angles `88`, `86`, `94`
  - directions `+1`, `-1`, `+1`
  - joint limits hip `[-20,20]`, thigh `[-80,80]`, knee `[0,95]`

  Front-left, rear-right, and rear-left use pin `-1` placeholders and remain disabled/unattached.

- [ ] **Step 2: Implement startup behavior**

  `setup()` must:
  - initialize Serial at `115200`,
  - configure all four legs,
  - attach only fully configured/enabled legs,
  - command front-right to a known reachable starting stance,
  - never command placeholder legs.

- [ ] **Step 3: Implement non-blocking `loop()` orchestration**

  `loop()` calls, in order:
  - motion updates for enabled legs,
  - temporary front-right motion test,
  - `updateGait()` placeholder,
  - `updateIMU()` placeholder,
  - `updateLidar()` placeholder,
  - `updateDepthCamera()` placeholder,
  - `updateSafety()` placeholder.

  No `delay()` is allowed in `loop()` or locomotion code.

- [ ] **Step 4: Build the embedded project**

  Run: `pio run`

  Expected: SUCCESS.

- [ ] **Step 5: Run all native tests**

  Run: `pio test -e native`

  Expected: PASS.

- [ ] **Step 6: Hardware smoke test the front-right leg**

  Upload: `pio run -t upload`

  Verify:
  - only GPIO `18/19/21` servos attach,
  - front-right reaches the same logical XYZ targets as the former single-leg implementation,
  - placeholder legs remain inactive,
  - Serial reports no partial-command or workspace errors for the chosen test path.

- [ ] **Step 7: Commit**

```bash
git add src/main.cpp include/RobotConfig.h
git commit -m "feat: integrate four-leg controller framework"
```

---

### Task 5: Static Four-Leg Stance API and Final Verification

**Files:**
- Modify: `include/Motion.h`
- Modify: `src/Motion.cpp`
- Modify: `src/main.cpp`

**Interfaces:**
- Consumes: four `Leg` instances and per-leg Cartesian motion APIs.
- Produces: `bool commandStandingPose(FootPosition target)` or equivalent API that attempts the same logical local target on all enabled/configured legs without changing shared IK equations.

- [ ] **Step 1: Add a standing-pose helper that skips disabled legs safely**

  The helper uses the same local XYZ target for each enabled leg; no left/right-specific IK branch is allowed.

- [ ] **Step 2: Add/extend tests for disabled-leg skipping and mirrored direction independence**

  Verify a disabled placeholder leg does not cause the standing-pose command to fail solely because it has `-1` pins, while any enabled leg with an invalid target causes the overall stance command to report failure.

- [ ] **Step 3: Run all tests**

  Run: `pio test -e native`

  Expected: PASS.

- [ ] **Step 4: Build embedded firmware**

  Run: `pio run`

  Expected: SUCCESS.

- [ ] **Step 5: Commit**

```bash
git add include/Motion.h src/Motion.cpp src/main.cpp test
git commit -m "feat: add static four-leg stance API"
```

---

## Final Verification

- [ ] Run `pio test -e native` — expected PASS.
- [ ] Run `pio run` — expected SUCCESS.
- [ ] Search locomotion code for `delay(` — expected no matches outside optional setup/debug-only code.
- [ ] Confirm front-right preserves current geometry/calibration behavior.
- [ ] Confirm other three legs can be activated later only by filling in GPIO/calibration/direction/limit values.
- [ ] Confirm `main.cpp` remains free for future gait, IMU, LiDAR, depth-camera, and safety updates.
