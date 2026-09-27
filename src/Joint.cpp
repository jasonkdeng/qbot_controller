#include "Joint.h"

#include <cmath>

void Joint::configure(int pin, float neutral, int direction, float minAngle, float maxAngle) {
  pin_ = pin;
  neutral_ = neutral;
  direction_ = direction >= 0 ? 1 : -1;
  minAngle_ = minAngle;
  maxAngle_ = maxAngle;
  attached_ = false;
  lastCommandedJointAngle_ = 0.0f;
}

bool Joint::attach() {
  if (!isConfigured()) {
    return false;
  }
#ifdef ARDUINO
  servo_.setPeriodHertz(RobotConfig::SERVO_FREQUENCY_HZ);
  servo_.attach(pin_, RobotConfig::SERVO_MIN_US, RobotConfig::SERVO_MAX_US);
  attached_ = servo_.attached();
#else
  attached_ = true;
#endif
  return attached_;
}

void Joint::detach() {
#ifdef ARDUINO
  servo_.detach();
#endif
  attached_ = false;
}

bool Joint::validate(float jointAngle, float &servoAngleOut) const {
  if (!isConfigured() || !std::isfinite(jointAngle) || !std::isfinite(neutral_)) {
    return false;
  }
  if (jointAngle < minAngle_ || jointAngle > maxAngle_) {
    return false;
  }

  const float servoAngle = neutral_ + static_cast<float>(direction_) * jointAngle;
  if (servoAngle < 0.0f || servoAngle > 180.0f) {
    return false;
  }

  servoAngleOut = servoAngle;
  return true;
}

bool Joint::command(float jointAngle) {
  if (!attached_) {
    return false;
  }

  float servoAngle = 0.0f;
  if (!validate(jointAngle, servoAngle)) {
    return false;
  }

#ifdef ARDUINO
  const int pulse = static_cast<int>(
      RobotConfig::SERVO_MIN_US +
      (servoAngle / 180.0f) * (RobotConfig::SERVO_MAX_US - RobotConfig::SERVO_MIN_US));
  servo_.writeMicroseconds(pulse);
#endif

  lastCommandedJointAngle_ = jointAngle;
  return true;
}
