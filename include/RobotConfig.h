#pragma once

#include <cstdint>

namespace RobotConfig {
constexpr float HIP_OFFSET_CM = 5.92f;
constexpr float THIGH_LENGTH_CM = 7.08f;
constexpr float SHIN_LENGTH_CM = 11.0f;

// Library-supported envelope only; each joint defines its own mapping below.
constexpr int SERVO_ABSOLUTE_MIN_US = 500;
constexpr int SERVO_ABSOLUTE_MAX_US = 2500;
constexpr int SERVO_FREQUENCY_HZ = 50;
constexpr std::uint32_t MOTION_UPDATE_MS = 20;

// Local leg coordinates used by every leg:
// +X = robot forward, +Y = outward from the robot body, +Z = downward.
enum class LegId : std::uint8_t {
  FrontRight = 0,
  FrontLeft,
  RearRight,
  RearLeft
};

constexpr int UNASSIGNED_PIN = -1;

struct JointConfig {
  int pin;
  float neutral;
  int direction;
  float minAngle;
  float maxAngle;
  int minPulseUs;
  int maxPulseUs;
};

struct LegConfig {
  bool enabled;
  JointConfig hip;
  JointConfig thigh;
  JointConfig knee;
};

// Preserve current neutrals/directions. Temporary diagnostic pulse endpoints
// change physical scaling: recheck all neutrals before loading the leg.
constexpr LegConfig FRONT_RIGHT{
    true,
    {18, 97.0f, -1, -20.0f, 20.0f, 1000, 2000},
    {19, 86.0f, -1, -45.0f, 45.0f, 1000, 2000},
    {21, 95.0f, +1, 0.0f, 80.0f, 1000, 2000}};

// Uncalibrated: replace each joint's pin, neutral, direction and limits before
// enabling that leg. The 90-degree neutrals and +1 directions are placeholders.
constexpr LegConfig FRONT_LEFT{
    false,
    {UNASSIGNED_PIN, 90.0f, +1, -20.0f, 20.0f, 1000, 2000}, // needs fixing
    {UNASSIGNED_PIN, 96.0f, +1, -45.0f, 45.0f, 1000, 2000},
    {UNASSIGNED_PIN, 92.0f, -1, 0.0f, 80.0f, 1000, 2000}};

constexpr LegConfig REAR_RIGHT{
    false,
    {UNASSIGNED_PIN, 80.0f, -1, -20.0f, 20.0f, 1000, 2000},
    {UNASSIGNED_PIN, 88.0f, -1, -45.0f, 45.0f, 1000, 2000},
    {UNASSIGNED_PIN, 93.0f, +1, 0.0f, 80.0f, 1000, 2000}};

constexpr LegConfig REAR_LEFT{
    false,
    {UNASSIGNED_PIN, 96.0f, +1, -20.0f, 20.0f, 1000, 2000},
    {UNASSIGNED_PIN, 80.0f, +1, -45.0f, 45.0f, 1000, 2000},
    {UNASSIGNED_PIN, 93.0f, -1, 0.0f, 80.0f, 1000, 2000}};

// Local foot target in cm from the hip roll axis.
constexpr float STANCE_X_CM = 0.0f;
constexpr float STANCE_Y_CM = 0.0f;
constexpr float STANCE_Z_CM = 20.0f;
}
