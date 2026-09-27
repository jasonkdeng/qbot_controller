#pragma once

#include "Joint.h"

#include <cstdint>

struct FootPosition {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

struct LegMotionState {
  FootPosition start{};
  FootPosition target{};
  std::uint32_t startTimeMs = 0;
  std::uint32_t durationMs = 0;
  bool moving = false;
};

enum class JointType : std::uint8_t {
  Hip = 0,
  Thigh,
  Knee
};

class Leg {
public:
  explicit Leg(const char *name = "leg", bool enabled = true);

  void configureJoint(JointType type, int pin, float neutral, int direction,
                      float minAngle, float maxAngle);
  bool attach();
  bool setFootPosition(float x, float y, float z);

  bool isEnabled() const { return enabled_; }
  void setEnabled(bool enabled) { enabled_ = enabled; }
  bool isFullyConfigured() const;
  const char *name() const { return name_; }

  const FootPosition &currentPosition() const { return currentPosition_; }
  void setCurrentPositionForMotion(const FootPosition &position) { currentPosition_ = position; }
  LegMotionState &motionState() { return motionState_; }
  const LegMotionState &motionState() const { return motionState_; }

  Joint &hip() { return hip_; }
  Joint &thigh() { return thigh_; }
  Joint &knee() { return knee_; }
  const Joint &hip() const { return hip_; }
  const Joint &thigh() const { return thigh_; }
  const Joint &knee() const { return knee_; }

private:
  Joint &jointFor(JointType type);

  const char *name_;
  bool enabled_;
  Joint hip_;
  Joint thigh_;
  Joint knee_;
  FootPosition currentPosition_{};
  LegMotionState motionState_{};
};
