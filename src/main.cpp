#include <Arduino.h>

#include "Leg.h"
#include "Motion.h"
#include "RobotConfig.h"

namespace {

Leg frontRight("front-right", RobotConfig::FRONT_RIGHT.enabled);
Leg frontLeft("front-left", RobotConfig::FRONT_LEFT.enabled);
Leg rearRight("rear-right", RobotConfig::REAR_RIGHT.enabled);
Leg rearLeft("rear-left", RobotConfig::REAR_LEFT.enabled);

Leg *allLegs[] = {&frontRight, &frontLeft, &rearRight, &rearLeft};
constexpr int LEG_COUNT = sizeof(allLegs) / sizeof(allLegs[0]);

constexpr FootPosition START_STANCE{RobotConfig::STANCE_X_CM,
                                    RobotConfig::STANCE_Y_CM,
                                    RobotConfig::STANCE_Z_CM};

void configureJoint(Leg &leg, JointType type, const RobotConfig::JointConfig &config) {
  leg.configureJoint(type, config.pin, config.neutral, config.direction,
                     config.minAngle, config.maxAngle);
}

void configureLeg(Leg &leg, const RobotConfig::LegConfig &config) {
  configureJoint(leg, JointType::Hip, config.hip);
  configureJoint(leg, JointType::Thigh, config.thigh);
  configureJoint(leg, JointType::Knee, config.knee);
  leg.setEnabled(config.enabled);
}

void configureRobot() {
  configureLeg(frontRight, RobotConfig::FRONT_RIGHT);
  configureLeg(frontLeft, RobotConfig::FRONT_LEFT);
  configureLeg(rearRight, RobotConfig::REAR_RIGHT);
  configureLeg(rearLeft, RobotConfig::REAR_LEFT);
}

void attachEnabledLegs() {
  for (Leg *leg : allLegs) {
    if (!leg->isEnabled()) {
      continue;
    }
    if (!leg->attach()) {
      Serial.print("Failed to attach leg: ");
      Serial.println(leg->name());
    }
  }
}

void updateAllLegMotion(std::uint32_t nowMs) {
  for (Leg *leg : allLegs) {
    if (leg->isEnabled()) {
      updateLegMotion(*leg, nowMs);
    }
  }
}

void updateGait() {
  // Future crawl/trot state machine.
}

void updateIMU() {
  // Future body attitude feedback.
}

void updateLidar() {
  // Future LiDAR packet handling or SBC communication.
}

void updateDepthCamera() {
  // Future depth-camera / SBC communication.
}

void updateSafety() {
  // Future watchdog, E-stop and fall/fault handling.
}

}  // namespace

void setup() {
  Serial.begin(115200);
  configureRobot();
  attachEnabledLegs();

  if (!commandStandingPose(allLegs, LEG_COUNT, START_STANCE)) {
    Serial.println("Standing pose rejected by an enabled leg.");
  }
}

void loop() {
  const std::uint32_t nowMs = millis();

  updateAllLegMotion(nowMs);
  updateGait();
  updateIMU();
  updateLidar();
  updateDepthCamera();
  updateSafety();
}
