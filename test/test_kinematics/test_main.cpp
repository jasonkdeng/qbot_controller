#include <cassert>
#include <cmath>
#include <iostream>
#include "Kinematics.h"

static bool near(float a, float b, float eps = 0.05f) {
  return std::fabs(a - b) <= eps;
}

int main() {
  JointAngles out{};

  assert(solveIK3D(0.0f, 0.0f, 17.0f, out));
  assert(near(out.hip, 0.0f));

  assert(solveIK3D(0.0f, 2.0f, 17.0f, out));
  const float expectedHip = std::atan2(2.0f, 17.0f) * 180.0f / 3.14159265358979323846f;
  assert(near(out.hip, expectedHip));

  assert(!solveIK3D(0.0f, 0.0f, 5.92f, out));

  float thigh = 0.0f;
  float knee = 0.0f;
  assert(!solveIK2D(0.0f, 18.09f, thigh, knee));
  assert(!solveIK2D(0.0f, 3.91f, thigh, knee));

  std::cout << "kinematics tests passed\n";
  return 0;
}
