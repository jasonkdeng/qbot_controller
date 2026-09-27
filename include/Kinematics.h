#pragma once

struct JointAngles {
  float hip;
  float thigh;
  float knee;
};

bool solveIK2D(float x, float z, float &thighAngle, float &kneeAngle);
bool solveIK3D(float x, float y, float z, JointAngles &out);
