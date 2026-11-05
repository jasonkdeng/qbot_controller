#include <cassert>
#include <iostream>
#include "../../src/calibration.cpp"

static void send(const char *command) {
  Serial.input = std::string(command) + "\n";
  Serial.output.clear();
  loop();
}

int main() {
  setup();
  send("leg FR");
  send("joint thigh");
  send("arm 166"); // Old -80-degree command must be rejected before attach.
  assert(servoEvents.empty());
  send("arm 86");
  assert(servoEvents.size() == 3);
  assert(servoEvents[1].value == 1477 && servoEvents[2].value == 1477);
  servoEvents.clear();
  send("step 5");
  assert(servoEvents.size() == 1 && servoEvents[0].value == 1505);
  for (int i = 0; i < 8; ++i) send("step 5");
  assert(rawAngle == 131);
  servoEvents.clear();
  send("step 5");
  assert(servoEvents.empty() && rawAngle == 131);
  send("off");
  servoEvents.clear();
  joints[selectedJoint].config.minPulseUs = 2100;
  send("arm 86");
  assert(servoEvents.empty());
  std::cout << "calibration safety tests passed\n";
}
