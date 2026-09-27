#include <cassert>
#include <cmath>
#include <iostream>
#include "Motion.h"

static bool near(float a, float b, float eps = 0.001f) { return std::fabs(a-b) <= eps; }

int main() {
  Leg leg("test", true);
  leg.configureJoint(JointType::Hip, 1, 90, +1, -30, 30);
  leg.configureJoint(JointType::Thigh, 2, 90, -1, -90, 90);
  leg.configureJoint(JointType::Knee, 3, 60, +1, 0, 120);
  assert(leg.attach());

  // Seed with a reachable current position.
  assert(leg.setFootPosition(0.0f, 0.0f, 17.0f));

  startLegMove(leg, FootPosition{2.0f, 0.0f, 17.0f}, 1000, 100);
  assert(isLegMoving(leg));
  updateLegMotion(leg, 100);
  assert(near(leg.currentPosition().x, 0.0f));

  updateLegMotion(leg, 600);
  assert(near(leg.currentPosition().x, 1.0f, 0.02f));

  updateLegMotion(leg, 1100);
  assert(!isLegMoving(leg));
  assert(near(leg.currentPosition().x, 2.0f, 0.01f));

  startLegMove(leg, FootPosition{0.0f, 0.0f, 17.0f}, 0, 1200);
  assert(!isLegMoving(leg));
  assert(near(leg.currentPosition().x, 0.0f, 0.01f));

  assert(near(smoothstep01(0.5f), 0.5f));


  Leg enabled2("enabled2", true);
  enabled2.configureJoint(JointType::Hip, 4, 90, +1, -30, 30);
  enabled2.configureJoint(JointType::Thigh, 5, 90, -1, -90, 90);
  enabled2.configureJoint(JointType::Knee, 6, 60, +1, 0, 120);
  assert(enabled2.attach());

  Leg disabled("disabled", false);
  disabled.configureJoint(JointType::Hip, -1, 90, +1, -20, 20);
  disabled.configureJoint(JointType::Thigh, -1, 90, +1, -80, 80);
  disabled.configureJoint(JointType::Knee, -1, 90, +1, 0, 95);

  Leg *stanceLegs[] = {&leg, &enabled2, &disabled};
  assert(commandStandingPose(stanceLegs, 3, FootPosition{0.0f, 0.0f, 17.0f}));
  assert(!commandStandingPose(stanceLegs, 3, FootPosition{0.0f, 0.0f, 5.0f}));

  std::cout << "motion tests passed\n";
  return 0;
}
