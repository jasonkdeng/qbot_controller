#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include "Joint.h"
#include "Leg.h"

static void configureFrontRight(Leg &leg) {
  const auto &c = RobotConfig::FRONT_RIGHT;
  leg.configureJoint(JointType::Hip, c.hip.pin, c.hip.neutral, c.hip.direction,
                     c.hip.minAngle, c.hip.maxAngle, c.hip.minPulseUs, c.hip.maxPulseUs);
  leg.configureJoint(JointType::Thigh, c.thigh.pin, c.thigh.neutral, c.thigh.direction,
                     c.thigh.minAngle, c.thigh.maxAngle, c.thigh.minPulseUs, c.thigh.maxPulseUs);
  leg.configureJoint(JointType::Knee, c.knee.pin, c.knee.neutral, c.knee.direction,
                     c.knee.minAngle, c.knee.maxAngle, c.knee.minPulseUs, c.knee.maxPulseUs);
}

int main() {
  Joint thigh;
  thigh.configure(19, 86, -1, -45, 45, 1000, 2000);
  const float angles[] = {-45, -30, -15, 0, 15, 30, 45};
  const int pulses[] = {1727, 1644, 1561, 1477, 1394, 1311, 1227};
  for (int i = 0; i < 7; ++i) {
    const auto output = thigh.calculate(angles[i]);
    assert(output.valid());
    assert(output.pulseUs == pulses[i]);
  }
  assert(!thigh.calculate(-80).valid());
  assert(thigh.calculate(-80).pulseUs == 1922); // In pulse range, outside joint range.
  assert(!thigh.calculate(46).valid());
  assert(!thigh.calculate(std::numeric_limits<float>::quiet_NaN()).valid());
  assert(!thigh.calculate(std::numeric_limits<float>::infinity()).valid());
  assert(thigh.attach());
  assert(thigh.lastCommandedPulseUs() == 1477);
  assert(thigh.command(-30));
  assert(!thigh.command(-80));
  assert(thigh.lastCommandedPulseUs() == 1644);
  assert(thigh.attach()); // Idempotent: no unwanted neutral reset.
  assert(thigh.lastCommandedPulseUs() == 1644);

  Joint invalid;
  invalid.configure(19, 86, 0, -45, 45, 1000, 2000);
  assert(!invalid.attach());
  invalid.configure(19, 86, -1, -45, 45, 2000, 1000);
  assert(!invalid.attach());
  invalid.configure(19, 86, -1, -45, 45, 400, 2600);
  assert(!invalid.attach());
  invalid.configure(19, 181, -1, -45, 45, 1000, 2000);
  assert(!invalid.attach());
  invalid.configure(19, 86, -1, 5, 45, 1000, 2000);
  assert(!invalid.attach()); // Neutral excluded from mathematical range.
  invalid.configure(19, 86, -1, std::numeric_limits<float>::quiet_NaN(), 45, 1000, 2000);
  assert(!invalid.attach());
  invalid.configure(-2, 86, -1, -45, 45, 1000, 2000);
  assert(!invalid.attach());
  invalid.configure(19, 175, 1, -20, 20, 1000, 2000);
  assert(!invalid.calculate(10).valid());
  invalid.configure(19, 90, 1, -90, 90, 1100, 1900);
  assert(invalid.calculate(-90).pulseUs == 1100);
  assert(invalid.calculate(90).pulseUs == 1900);

  Leg leg("front-right");
  configureFrontRight(leg);
  assert(leg.attach());
  assert(leg.setFootPosition(0, 0, 22));
  const int hipPulse = leg.hip().lastCommandedPulseUs();
  const int thighPulse = leg.thigh().lastCommandedPulseUs();
  const int kneePulse = leg.knee().lastCommandedPulseUs();
  // Hip target changes, but thigh violates the new limit: no joint may move.
  assert(!leg.setFootPosition(0, 1, 20));
  assert(leg.hip().lastCommandedPulseUs() == hipPulse);
  assert(leg.thigh().lastCommandedPulseUs() == thighPulse);
  assert(leg.knee().lastCommandedPulseUs() == kneePulse);
  assert(leg.currentPosition().z == 22);
  assert(!leg.setFootPosition(std::numeric_limits<float>::quiet_NaN(), 0, 22));
  leg.knee().detach();
  assert(!leg.setFootPosition(1, 0, 22));
  assert(leg.hip().lastCommandedPulseUs() == hipPulse);
  assert(leg.thigh().lastCommandedPulseUs() == thighPulse);

  Leg badLeg("bad");
  configureFrontRight(badLeg);
  badLeg.configureJoint(JointType::Knee, 21, 190, 1, 0, 80, 1000, 2000);
  assert(!badLeg.attach());
  assert(!badLeg.hip().isAttached());
  assert(!badLeg.thigh().isAttached());
  std::cout << "servo safety tests passed\n";
}
