#include <cassert>
#include <iostream>
#include "../../src/single_leg_ik.cpp"

static void send(const char *command) {
  Serial.input = std::string(command) + "\n";
  Serial.output.clear();
  while (Serial.available()) loop();
}

int main() {
  setup();
  assert(servoEvents.empty());
  send("check 0 0 20");
  assert(Serial.output.find("Target XYZ:") != std::string::npos);
  assert(Serial.output.find("pulse=1538 OK") != std::string::npos);
  assert(Serial.output.find("REJECTED: joint limit") != std::string::npos);
  assert(Serial.output.find("pulse=1972 OK") != std::string::npos);
  assert(servoEvents.empty());
  send("arm");
  assert(servoEvents.empty());
  send("move 0 0 20");
  assert(servoEvents.empty()); // Rejected pose must not even initialize neutrals.
  send("joint thigh -30");
  assert(servoEvents.size() == 3); // Attach, configured neutral, requested target.
  assert(servoEvents[0].kind == 'a' && servoEvents[0].pin == 19);
  assert(servoEvents[1].value == 1477);
  assert(servoEvents[2].value == 1644);
  assert(Serial.output.find("servo=116.00 pulse=1644 OK") != std::string::npos);
  servoEvents.clear();
  send("joint thigh -80");
  assert(servoEvents.empty());
  send("move nan 0 22");
  assert(servoEvents.empty());
  assert(Serial.output.find("pulse=N/A") != std::string::npos);
  send("move 0 0 22 extra");
  assert(servoEvents.empty());
  send("move 0 0 22");
  assert(footPositionKnown);
  for (const auto &event : servoEvents)
    if (event.kind == 'w') assert(event.value >= 1000 && event.value <= 2000);
  servoEvents.clear();
  send("move 0 1 20");
  assert(servoEvents.empty()); // Even hip must remain unchanged when thigh rejects.
  send("off");
  assert(!leg.hip().isAttached() && !leg.thigh().isAttached() && !leg.knee().isAttached());
  servoEvents.clear();

  // Invalid neutral must fail before any attachment or write.
  leg.configureJoint(JointType::Knee, 21, 190, 1, 0, 80, 1000, 2000);
  assert(!leg.attach());
  assert(servoEvents.empty());
  configure(JointType::Knee, RobotConfig::FRONT_RIGHT.knee);
  failAttachPin = 21;
  assert(!leg.attach());
  assert(!leg.hip().isAttached() && !leg.thigh().isAttached() && !leg.knee().isAttached());
  std::cout << "serial and PWM boundary tests passed\n";
}
