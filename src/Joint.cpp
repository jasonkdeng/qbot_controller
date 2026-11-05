#include "Joint.h"
#include <cmath>
#include <limits>

namespace {
bool validPulseRange(int low, int high) {
  return low >= RobotConfig::SERVO_ABSOLUTE_MIN_US &&
         high <= RobotConfig::SERVO_ABSOLUTE_MAX_US && low < high;
}

bool mapPulse(float angle, int low, int high, int &pulse) {
  const float value = low + (angle / 180.0f) * (high - low);
  if (!std::isfinite(value) || static_cast<double>(value) < std::numeric_limits<int>::min() ||
      static_cast<double>(value) > std::numeric_limits<int>::max()) return false;
  pulse = static_cast<int>(value); // Preserve existing truncation convention.
  return value >= low && value <= high && pulse >= low && pulse <= high;
}
}

void Joint::configure(int pin, float neutral, int direction, float minAngle, float maxAngle,
                      int minPulseUs, int maxPulseUs) {
  detach();
  pin_ = pin;
  neutral_ = neutral;
  direction_ = direction; // Reject invalid values instead of silently normalizing.
  minAngle_ = minAngle;
  maxAngle_ = maxAngle;
  minPulseUs_ = minPulseUs;
  maxPulseUs_ = maxPulseUs;
  lastCommandedJointAngle_ = 0;
  lastCommandedPulseUs_ = -1;
}

bool Joint::calculatePulse(float angle, int low, int high, int &pulse) {
  pulse = -1;
  if (!validPulseRange(low, high) || !std::isfinite(angle) || angle < 0 || angle > 180)
    return false;
  return mapPulse(angle, low, high, pulse);
}

JointOutput Joint::calculate(float angle) const {
  JointOutput out;
  if (!std::isfinite(angle)) { out.error = "non-finite joint angle"; return out; }
  if (!isConfigured()) { out.error = "unassigned pin"; return out; }
  if (!std::isfinite(neutral_) || neutral_ < 0 || neutral_ > 180 ||
      (direction_ != 1 && direction_ != -1) || !std::isfinite(minAngle_) ||
      !std::isfinite(maxAngle_) || minAngle_ > maxAngle_) {
    out.error = "invalid calibration"; return out;
  }
  if (!validPulseRange(minPulseUs_, maxPulseUs_)) {
    out.error = "invalid pulse range"; return out;
  }
  out.servoAngle = neutral_ + direction_ * angle;
  const bool pulseValid = mapPulse(out.servoAngle, minPulseUs_, maxPulseUs_, out.pulseUs);
  // Retain diagnostic values even if mathematical limits reject this target.
  if (angle < minAngle_ || angle > maxAngle_) out.error = "joint limit";
  else if (!std::isfinite(out.servoAngle) || out.servoAngle < 0 || out.servoAngle > 180)
    out.error = "servo angle limit";
  else if (!pulseValid) out.error = "pulse limit";
  else out.error = nullptr;
  return out;
}

bool Joint::validate(float angle, float &servoAngle, int &pulse) const {
  const auto out = calculate(angle);
  servoAngle = out.servoAngle;
  pulse = out.pulseUs;
  return out.valid();
}

bool Joint::attach() {
  if (attached_) return true;
  const auto initial = calculate(0);
  if (!initial.valid()) return false;
#ifdef ARDUINO
  servo_.setPeriodHertz(RobotConfig::SERVO_FREQUENCY_HZ);
  servo_.attach(pin_, minPulseUs_, maxPulseUs_);
  attached_ = servo_.attached();
  if (!attached_) return false;
  // Library attachment enables PWM first. This immediate neutral write reduces
  // the initialization window; absence of an electrical transient is unverified.
  servo_.writeMicroseconds(initial.pulseUs);
#else
  attached_ = true;
#endif
  lastCommandedJointAngle_ = 0;
  lastCommandedPulseUs_ = initial.pulseUs;
  return true;
}

void Joint::detach() {
#ifdef ARDUINO
  if (attached_) servo_.detach();
#endif
  attached_ = false;
}

bool Joint::command(float angle) {
  const auto out = calculate(angle);
  if (!attached_ || !out.valid()) return false;
#ifdef ARDUINO
  servo_.writeMicroseconds(out.pulseUs);
#endif
  lastCommandedJointAngle_ = angle;
  lastCommandedPulseUs_ = out.pulseUs;
  return true;
}

// Future calibration could use neutralPulseUs + direction * angle * usPerDegree.
// Keep the requested neutral-angle model and per-joint endpoints for now.
