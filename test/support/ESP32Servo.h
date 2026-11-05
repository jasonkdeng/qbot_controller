#pragma once
#include <vector>

// Hardware boundary substitute: records the real Joint/Leg code's PWM calls.
struct ServoEvent { char kind; int pin; int value; };
inline std::vector<ServoEvent> servoEvents;
inline int failAttachPin = -1;
class Servo {
  int pin_ = -1;
public:
  void setPeriodHertz(int) {}
  int attach(int pin, int, int) {
    servoEvents.push_back({'a', pin, 0});
    if (pin != failAttachPin) pin_ = pin;
    return pin_ >= 0 ? 1 : 0;
  }
  bool attached() const { return pin_ >= 0; }
  void writeMicroseconds(int pulse) { servoEvents.push_back({'w', pin_, pulse}); }
  void detach() { servoEvents.push_back({'d', pin_, 0}); pin_ = -1; }
};
