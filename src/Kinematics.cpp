#include "Kinematics.h"
#include "RobotConfig.h"

#include <algorithm>
#include <cmath>

namespace {
constexpr float PI_F = 3.14159265358979323846f;

float radToDeg(float radians) {
  return radians * 180.0f / PI_F;
}

float clampUnit(float value) {
  return std::max(-1.0f, std::min(1.0f, value));
}
}

bool solveIK2D(float x, float z, float &thighAngle, float &kneeAngle) {
  using namespace RobotConfig;

  const float distanceSquared = x * x + z * z;
  const float distance = std::sqrt(distanceSquared);
  const float maxReach = THIGH_LENGTH_CM + SHIN_LENGTH_CM;
  const float minReach = std::fabs(SHIN_LENGTH_CM - THIGH_LENGTH_CM);

  if (distance > maxReach || distance < minReach || distance <= 0.0f) {
    return false;
  }

  float cosKnee =
      (THIGH_LENGTH_CM * THIGH_LENGTH_CM +
       SHIN_LENGTH_CM * SHIN_LENGTH_CM - distanceSquared) /
      (2.0f * THIGH_LENGTH_CM * SHIN_LENGTH_CM);
  cosKnee = clampUnit(cosKnee);

  const float internalKnee = std::acos(cosKnee);
  kneeAngle = 180.0f - radToDeg(internalKnee);

  const float targetAngle = std::atan2(x, z);
  float cosThigh =
      (THIGH_LENGTH_CM * THIGH_LENGTH_CM + distanceSquared -
       SHIN_LENGTH_CM * SHIN_LENGTH_CM) /
      (2.0f * THIGH_LENGTH_CM * distance);
  cosThigh = clampUnit(cosThigh);

  const float thighTriangleAngle = std::acos(cosThigh);
  thighAngle = radToDeg(targetAngle - thighTriangleAngle);

  return true;
}

bool solveIK3D(float x, float y, float z, JointAngles &out) {
  using namespace RobotConfig;

  const float radialDistance = std::sqrt(y * y + z * z);
  if (radialDistance <= HIP_OFFSET_CM) {
    return false;
  }

  out.hip = radToDeg(std::atan2(y, z));
  const float legPlaneZ = radialDistance - HIP_OFFSET_CM;

  return solveIK2D(x, legPlaneZ, out.thigh, out.knee);
}
