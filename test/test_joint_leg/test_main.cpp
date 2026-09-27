#include <cassert>
#include <cmath>
#include <iostream>
#include "Joint.h"
#include "Leg.h"
#include "Kinematics.h"

static bool near(float a, float b, float eps = 0.001f) { return std::fabs(a-b) <= eps; }

int main() {
  Joint j;
  j.configure(1, 90.0f, +1, -20.0f, 20.0f);
  float servoAngle = 0.0f;
  assert(j.validate(10.0f, servoAngle));
  assert(near(servoAngle, 100.0f));

  j.configure(1, 90.0f, -1, -20.0f, 20.0f);
  assert(j.validate(10.0f, servoAngle));
  assert(near(servoAngle, 80.0f));
  assert(!j.validate(25.0f, servoAngle));

  j.configure(1, 170.0f, +1, -20.0f, 20.0f);
  assert(!j.validate(20.0f, servoAngle));

  Leg disabled("placeholder", false);
  disabled.configureJoint(JointType::Hip, -1, 90, +1, -20, 20);
  disabled.configureJoint(JointType::Thigh, -1, 90, +1, -80, 80);
  disabled.configureJoint(JointType::Knee, -1, 90, +1, 0, 95);
  assert(!disabled.attach());
  assert(!disabled.setFootPosition(0,0,17));

  JointAngles a{}, b{};
  assert(solveIK3D(0, 2, 17, a));
  assert(solveIK3D(0, 2, 17, b));
  assert(near(a.hip,b.hip) && near(a.thigh,b.thigh) && near(a.knee,b.knee));

  std::cout << "joint/leg tests passed\n";
  return 0;
}
