#include "Leg.h"
#include "Kinematics.h"
#include <cmath>

Leg::Leg(const char *name, bool enabled) : name_(name), enabled_(enabled) {}

Joint &Leg::jointFor(JointType type) {
  switch (type) {
    case JointType::Hip: return hip_;
    case JointType::Thigh: return thigh_;
    default: return knee_;
  }
}

void Leg::configureJoint(JointType type, int pin, float neutral, int direction,
                         float minAngle, float maxAngle, int minPulseUs, int maxPulseUs) {
  jointFor(type).configure(pin, neutral, direction, minAngle, maxAngle, minPulseUs, maxPulseUs);
}

bool Leg::isFullyConfigured() const {
  return hip_.isConfigured() && thigh_.isConfigured() && knee_.isConfigured();
}

bool Leg::attach() {
  if (!enabled_ || !isFullyConfigured()) {
    return false;
  }
  if (!hip_.calculate(0).valid() || !thigh_.calculate(0).valid() || !knee_.calculate(0).valid() ||
      hip_.pin() == thigh_.pin() || hip_.pin() == knee_.pin() || thigh_.pin() == knee_.pin())
    return false;
  const bool hipOk = hip_.attach();
  const bool thighOk = thigh_.attach();
  const bool kneeOk = knee_.attach();
  if (hipOk && thighOk && kneeOk) return true;
  hip_.detach(); thigh_.detach(); knee_.detach();
  return false;
}

bool Leg::setFootPosition(float x, float y, float z) {
  if (!enabled_ || !isFullyConfigured() ||
      !hip_.isAttached() || !thigh_.isAttached() || !knee_.isAttached()) {
    return false;
  }

  JointAngles angles{};
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
      !solveIK3D(x, y, z, angles)) {
    return false;
  }
  if (!commandJointAngles(angles)) return false;
  currentPosition_ = {x, y, z};
  return true;
}

bool Leg::commandJointAngles(const JointAngles &angles) {
  if (!enabled_ || !hip_.isAttached() || !thigh_.isAttached() || !knee_.isAttached()) return false;
  const auto hipOutput = hip_.calculate(angles.hip);
  const auto thighOutput = thigh_.calculate(angles.thigh);
  const auto kneeOutput = knee_.calculate(angles.knee);
  if (!hipOutput.valid() || !thighOutput.valid() || !kneeOutput.valid()) {
    return false;
  }

  if (!hip_.command(angles.hip) ||
      !thigh_.command(angles.thigh) ||
      !knee_.command(angles.knee)) {
    return false;
  }

  return true;
}
