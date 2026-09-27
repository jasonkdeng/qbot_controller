#include "Leg.h"
#include "Kinematics.h"

Leg::Leg(const char *name, bool enabled) : name_(name), enabled_(enabled) {}

Joint &Leg::jointFor(JointType type) {
  switch (type) {
    case JointType::Hip: return hip_;
    case JointType::Thigh: return thigh_;
    default: return knee_;
  }
}

void Leg::configureJoint(JointType type, int pin, float neutral, int direction,
                         float minAngle, float maxAngle) {
  jointFor(type).configure(pin, neutral, direction, minAngle, maxAngle);
}

bool Leg::isFullyConfigured() const {
  return hip_.isConfigured() && thigh_.isConfigured() && knee_.isConfigured();
}

bool Leg::attach() {
  if (!enabled_ || !isFullyConfigured()) {
    return false;
  }
  const bool hipOk = hip_.attach();
  const bool thighOk = thigh_.attach();
  const bool kneeOk = knee_.attach();
  return hipOk && thighOk && kneeOk;
}

bool Leg::setFootPosition(float x, float y, float z) {
  if (!enabled_ || !isFullyConfigured()) {
    return false;
  }

  JointAngles angles{};
  if (!solveIK3D(x, y, z, angles)) {
    return false;
  }

  float hipServo = 0.0f;
  float thighServo = 0.0f;
  float kneeServo = 0.0f;
  if (!hip_.validate(angles.hip, hipServo) ||
      !thigh_.validate(angles.thigh, thighServo) ||
      !knee_.validate(angles.knee, kneeServo)) {
    return false;
  }

  if (!hip_.command(angles.hip) ||
      !thigh_.command(angles.thigh) ||
      !knee_.command(angles.knee)) {
    return false;
  }

  currentPosition_ = {x, y, z};
  return true;
}
