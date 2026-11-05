#include <Arduino.h>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include "Leg.h"

namespace {
Leg leg("front-right", RobotConfig::FRONT_RIGHT.enabled);
bool armed = false;
bool footPositionKnown = false;
char line[96];
size_t length = 0;
bool overflow = false;

void configure(JointType type, const RobotConfig::JointConfig &c) {
  leg.configureJoint(type, c.pin, c.neutral, c.direction, c.minAngle, c.maxAngle,
                     c.minPulseUs, c.maxPulseUs);
}

void off() {
  leg.hip().detach(); leg.thigh().detach(); leg.knee().detach();
  armed = false;
  footPositionKnown = false;
}

bool report(const char *name, const Joint &joint, float angle) {
  const auto out = joint.calculate(angle);
  Serial.printf("%-5s joint=%.2f servo=%.2f pulse=%d %s%s\n", name, angle,
      out.servoAngle, out.pulseUs, out.valid() ? "OK" : "REJECTED: ",
      out.valid() ? "" : out.error);
  return out.valid();
}

bool calculate(FootPosition p, JointAngles &angles) {
  Serial.printf("Target XYZ: %.2f %.2f %.2f\n", p.x, p.y, p.z);
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !solveIK3D(p.x, p.y, p.z, angles)) {
    Serial.println("IK REJECTED: invalid coordinate or unreachable target.");
    Serial.println("hip   joint=N/A servo=N/A pulse=N/A REJECTED: no IK solution\n"
                   "thigh joint=N/A servo=N/A pulse=N/A REJECTED: no IK solution\n"
                   "knee  joint=N/A servo=N/A pulse=N/A REJECTED: no IK solution");
    return false;
  }
  const bool hipOk = report("hip", leg.hip(), angles.hip);
  const bool thighOk = report("thigh", leg.thigh(), angles.thigh);
  const bool kneeOk = report("knee", leg.knee(), angles.knee);
  return hipOk && thighOk && kneeOk;
}

void status() {
  Serial.printf("Armed=%s; PWM hip/thigh/knee=%d/%d/%d\n", armed ? "yes" : "no",
                leg.hip().isAttached(), leg.thigh().isAttached(), leg.knee().isAttached());
  if (footPositionKnown) {
    const auto &p = leg.currentPosition();
    Serial.printf("Last commanded XYZ: %.2f %.2f %.2f (not measured)\n", p.x, p.y, p.z);
  } else Serial.println("No current Cartesian command (off or individual joint test).");
}

bool parseNumber(char *text, float &out) {
  if (!text) return false;
  char *end = nullptr;
  out = std::strtof(text, &end);
  return end != text && *end == '\0'; // calculate() reports non-finite values explicitly.
}

void execute(char *text) {
  char *cmd = std::strtok(text, " \t");
  if (!cmd) return;
  if (!std::strcmp(cmd, "check") || !std::strcmp(cmd, "move")) {
    FootPosition p;
    if (!parseNumber(std::strtok(nullptr, " \t"), p.x) ||
        !parseNumber(std::strtok(nullptr, " \t"), p.y) ||
        !parseNumber(std::strtok(nullptr, " \t"), p.z) || std::strtok(nullptr, " \t")) {
      Serial.println("Use check x y z or move x y z, in cm. Output unchanged."); return;
    }
    JointAngles angles{};
    if (!calculate(p, angles)) { Serial.println("Target rejected; output unchanged."); return; }
    if (!std::strcmp(cmd, "check")) return;
    if (!armed) { Serial.println("Arm first; output unchanged."); return; }
    // Full target validation above precedes attachment, including neutral output.
    if (!leg.attach()) { Serial.println("Attach failed; target not commanded."); return; }
    if (!leg.commandJointAngles(angles)) {
      Serial.println("Target rejected; output unchanged."); return;
    }
    leg.setCurrentPositionForMotion(p);
    footPositionKnown = true;
    Serial.println("Target commanded once. Wait for physical settling.");
    return;
  }
  if (!std::strcmp(cmd, "joint")) {
    char *name = std::strtok(nullptr, " \t");
    float angle = 0;
    if (!name || !parseNumber(std::strtok(nullptr, " \t"), angle) ||
        std::strtok(nullptr, " \t")) {
      Serial.println("Use joint hip|thigh|knee <mathematical degrees>."); return;
    }
    Joint *joint = !std::strcmp(name, "hip") ? &leg.hip() :
                   !std::strcmp(name, "thigh") ? &leg.thigh() :
                   !std::strcmp(name, "knee") ? &leg.knee() : nullptr;
    if (!joint) { Serial.println("Unknown joint; output unchanged."); return; }
    if (!report(name, *joint, angle)) { Serial.println("Joint rejected; output unchanged."); return; }
    if (!armed || !leg.isEnabled()) { Serial.println("Arm an enabled leg first; output unchanged."); return; }
    if (!joint->attach() || !joint->command(angle)) {
      Serial.println("Joint output failed."); return;
    }
    footPositionKnown = false;
    Serial.println("Joint commanded once; other joint outputs unchanged."); return;
  }
  if (std::strtok(nullptr, " \t")) { Serial.println("Unexpected argument."); return; }
  if (!std::strcmp(cmd, "help")) {
    Serial.println("check x y z | arm | move x y z | joint hip|thigh|knee angle | stop | off | status\n"
                   "Local cm: +X forward, +Y outward, +Z down.\n"
                   "arm enables commands only. First attach writes configured neutral, then target.\n"
                   "joint uses mathematical degrees; try thigh -30, -15, 0, 15, 30 individually.\n"
                   "No interpolation. stop holds the current target; off releases PWM.\n"
                   "Pulse endpoints remap physical angles: recheck neutral with the leg supported.");
  } else if (!std::strcmp(cmd, "arm")) {
    armed = leg.isEnabled(); status();
  } else if (!std::strcmp(cmd, "off")) {
    off(); status();
  } else if (!std::strcmp(cmd, "stop")) {
    Serial.println("No trajectory; current targets remain active. off releases PWM.");
  } else if (!std::strcmp(cmd, "status")) status();
  else Serial.println("Unknown command; use help.");
}
}

void setup() {
  Serial.begin(115200);
  configure(JointType::Hip, RobotConfig::FRONT_RIGHT.hip);
  configure(JointType::Thigh, RobotConfig::FRONT_RIGHT.thigh);
  configure(JointType::Knee, RobotConfig::FRONT_RIGHT.knee);
  Serial.println("Front-right bounded servo diagnostics. PWM off. Type help.");
}

void loop() {
  for (int n = 0; n < 96 && Serial.available(); ++n) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (overflow) Serial.println("Line too long; discarded.");
      else if (length) { line[length] = '\0'; execute(line); }
      length = 0; overflow = false;
    } else if (c == '\b' || c == 127) {
      if (!overflow && length) --length;
    } else if (!overflow) {
      if (length < sizeof(line) - 1) line[length++] = c;
      else overflow = true;
    }
  }
}
