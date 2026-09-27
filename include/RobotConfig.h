#pragma once

#include <cstdint>

namespace RobotConfig {
constexpr float HIP_OFFSET_CM = 5.92f;
constexpr float THIGH_LENGTH_CM = 7.08f;
constexpr float SHIN_LENGTH_CM = 11.0f;

constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2500;
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
};

struct LegConfig {
  bool enabled;
  JointConfig hip;
  JointConfig thigh;
  JointConfig knee;
};

// Confirmed front-right calibration. Angles are degrees relative to joint neutral.
constexpr LegConfig FRONT_RIGHT{
    true,
    {18, 88.0f, +1, -20.0f, 20.0f},
    {19, 86.0f, -1, -80.0f, 80.0f},
    {21, 94.0f, +1, 0.0f, 95.0f}};

// Uncalibrated: replace each joint's pin, neutral, direction and limits before
// enabling that leg. The 90-degree neutrals and +1 directions are placeholders.
constexpr LegConfig FRONT_LEFT{
    false,
    {UNASSIGNED_PIN, 90.0f, +1, -20.0f, 20.0f},
    {UNASSIGNED_PIN, 90.0f, +1, -80.0f, 80.0f},
    {UNASSIGNED_PIN, 90.0f, +1, 0.0f, 95.0f}};

constexpr LegConfig REAR_RIGHT{
    false,
    {UNASSIGNED_PIN, 90.0f, +1, -20.0f, 20.0f},
    {UNASSIGNED_PIN, 90.0f, +1, -80.0f, 80.0f},
    {UNASSIGNED_PIN, 90.0f, +1, 0.0f, 95.0f}};

constexpr LegConfig REAR_LEFT{
    false,
    {UNASSIGNED_PIN, 90.0f, +1, -20.0f, 20.0f},
    {UNASSIGNED_PIN, 90.0f, +1, -80.0f, 80.0f},
    {UNASSIGNED_PIN, 90.0f, +1, 0.0f, 95.0f}};

// Local foot target in cm from the hip roll axis.
constexpr float STANCE_X_CM = 0.0f;
constexpr float STANCE_Y_CM = 0.0f;
constexpr float STANCE_Z_CM = 20.0f;
}
