#pragma once

#include "RobotConfig.h"
#include <limits>

#ifdef ARDUINO
#include <ESP32Servo.h>
#endif

struct JointOutput {
  float servoAngle = std::numeric_limits<float>::quiet_NaN();
  int pulseUs = -1; // Not calculable; never written to hardware.
  const char *error = "not calculated";
  bool valid() const { return error == nullptr; }
};

class Joint {
public:
  Joint() = default;

  void configure(int pin, float neutral, int direction, float minAngle, float maxAngle,
                 int minPulseUs, int maxPulseUs);
  bool attach();
  void detach();
  JointOutput calculate(float jointAngle) const;
  bool validate(float jointAngle, float &servoAngleOut, int &pulseUsOut) const;
  static bool calculatePulse(float servoAngle, int minPulseUs, int maxPulseUs, int &pulseUsOut);
  bool command(float jointAngle);

  bool isConfigured() const { return pin_ >= 0; }
  bool isAttached() const { return attached_; }
  int pin() const { return pin_; }
  float neutral() const { return neutral_; }
  int direction() const { return direction_; }
  float minAngle() const { return minAngle_; }
  float maxAngle() const { return maxAngle_; }
  int minPulseUs() const { return minPulseUs_; }
  int maxPulseUs() const { return maxPulseUs_; }
  int lastCommandedPulseUs() const { return lastCommandedPulseUs_; }
  float lastCommandedJointAngle() const { return lastCommandedJointAngle_; }

private:
  int pin_ = RobotConfig::UNASSIGNED_PIN;
  float neutral_ = 90.0f;
  int direction_ = 1;
  float minAngle_ = 0.0f;
  float maxAngle_ = 0.0f;
  int minPulseUs_ = 0;
  int maxPulseUs_ = 0;
  int lastCommandedPulseUs_ = -1;
  bool attached_ = false;
  float lastCommandedJointAngle_ = 0.0f;
#ifdef ARDUINO
  Servo servo_;
#endif
};
