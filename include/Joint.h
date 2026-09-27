#pragma once

#include "RobotConfig.h"

#ifdef ARDUINO
#include <ESP32Servo.h>
#endif

class Joint {
public:
  Joint() = default;

  void configure(int pin, float neutral, int direction, float minAngle, float maxAngle);
  bool attach();
  bool validate(float jointAngle, float &servoAngleOut) const;
  bool command(float jointAngle);

  bool isConfigured() const { return pin_ != RobotConfig::UNASSIGNED_PIN; }
  bool isAttached() const { return attached_; }
  int pin() const { return pin_; }
  float neutral() const { return neutral_; }
  int direction() const { return direction_; }
  float minAngle() const { return minAngle_; }
  float maxAngle() const { return maxAngle_; }
  float lastCommandedJointAngle() const { return lastCommandedJointAngle_; }

private:
  int pin_ = RobotConfig::UNASSIGNED_PIN;
  float neutral_ = 90.0f;
  int direction_ = 1;
  float minAngle_ = 0.0f;
  float maxAngle_ = 0.0f;
  bool attached_ = false;
  float lastCommandedJointAngle_ = 0.0f;
#ifdef ARDUINO
  Servo servo_;
#endif
};
