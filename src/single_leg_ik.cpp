#include <Arduino.h>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include "Kinematics.h"
#include "Leg.h"
#include "Motion.h"

namespace {
Leg leg("front-right", RobotConfig::FRONT_RIGHT.enabled);
constexpr FootPosition HOME{RobotConfig::STANCE_X_CM, RobotConfig::STANCE_Y_CM,
                            RobotConfig::STANCE_Z_CM};
constexpr uint32_t MOVE_MS = 2000;
constexpr float MAX_MOVE_CM = 3.0f;
bool armed = false;
bool moving = false;
FootPosition start{}, target{};
uint32_t started = 0, lastUpdate = 0;
char line[96];
size_t length = 0;
bool overflow = false;

void configure(JointType type, const RobotConfig::JointConfig &c) {
  leg.configureJoint(type, c.pin, c.neutral, c.direction, c.minAngle, c.maxAngle);
}

void off() {
  moving = false;
  leg.hip().detach(); leg.thigh().detach(); leg.knee().detach();
  armed = false;
}

bool validate(FootPosition p, bool report) {
  JointAngles a{};
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !solveIK3D(p.x, p.y, p.z, a)) {
    if (report) Serial.println("Rejected: outside IK workspace or non-finite target.");
    return false;
  }
  const float angles[] = {a.hip, a.thigh, a.knee};
  const Joint *joints[] = {&leg.hip(), &leg.thigh(), &leg.knee()};
  const char *names[] = {"hip", "thigh", "knee"};
  bool valid = true;
  for (int i = 0; i < 3; ++i) {
    float raw = 0;
    const bool ok = joints[i]->validate(angles[i], raw);
    if (report) Serial.printf("%s joint=%.2f servo=%.2f %s\n", names[i], angles[i],
        joints[i]->neutral() + joints[i]->direction() * angles[i], ok ? "OK" : "REJECTED");
    valid = valid && ok;
  }
  return valid;
}

FootPosition interpolate(float s) {
  return {start.x + (target.x - start.x) * s,
          start.y + (target.y - start.y) * s,
          start.z + (target.z - start.z) * s};
}

void status() {
  Serial.printf("PWM=%s moving=%s\n", armed ? "on" : "off", moving ? "yes" : "no");
  if (armed) {
    const auto &p = leg.currentPosition();
    Serial.printf("Last commanded XYZ: %.2f %.2f %.2f cm (not measured)\n", p.x, p.y, p.z);
  }
}

bool parseNumber(char *text, float &out) {
  if (!text) return false;
  char *end = nullptr;
  out = std::strtof(text, &end);
  return end != text && *end == '\0' && std::isfinite(out);
}

void execute(char *text) {
  char *cmd = std::strtok(text, " \t");
  if (!cmd) return;
  if (!std::strcmp(cmd, "check") || !std::strcmp(cmd, "move")) {
    FootPosition p;
    if (!parseNumber(std::strtok(nullptr, " \t"), p.x) ||
        !parseNumber(std::strtok(nullptr, " \t"), p.y) ||
        !parseNumber(std::strtok(nullptr, " \t"), p.z) || std::strtok(nullptr, " \t")) {
      Serial.println("Use check x y z or move x y z, in cm."); return;
    }
    if (!validate(p, true) || !std::strcmp(cmd, "check")) return;
    if (!armed || moving) { Serial.println("Arm first; wait for completion or use stop."); return; }
    start = leg.currentPosition(); target = p;
    const float dx = p.x - start.x, dy = p.y - start.y, dz = p.z - start.z;
    if (std::sqrt(dx * dx + dy * dy + dz * dz) > MAX_MOVE_CM) {
      Serial.println("Rejected: use moves of at most 3 cm for this bench test."); return;
    }
    for (int i = 0; i <= 100; ++i) {
      if (!validate(interpolate(i / 100.0f), false)) {
        Serial.println("Rejected: an intermediate position exceeds limits."); return;
      }
    }
    started = lastUpdate = millis(); moving = true;
    Serial.println("Moving over 2 seconds."); return;
  }
  if (std::strtok(nullptr, " \t")) { Serial.println("Unexpected argument."); return; }
  if (!std::strcmp(cmd, "help")) {
    Serial.println("check x y z | arm | move x y z | stop | off | status\n"
                   "Local cm: +X forward, +Y outward, +Z down from hip axis.\n"
                   "arm immediately commands the configured stance. Support the leg.\n"
                   "stop holds the last command; off releases PWM/holding torque.");
  } else if (!std::strcmp(cmd, "arm")) {
    if (armed) { Serial.println("Already armed."); return; }
    if (!validate(HOME, true)) return;
    if (!leg.attach() || !leg.setFootPosition(HOME.x, HOME.y, HOME.z)) {
      off(); Serial.println("Arming failed; PWM detached."); return;
    }
    armed = true; status();
  } else if (!std::strcmp(cmd, "stop")) {
    moving = false; status();
  } else if (!std::strcmp(cmd, "off")) {
    off(); status();
  } else if (!std::strcmp(cmd, "status")) status();
  else Serial.println("Unknown command; use help.");
}
} // namespace

void setup() {
  Serial.begin(115200);
  configure(JointType::Hip, RobotConfig::FRONT_RIGHT.hip);
  configure(JointType::Thigh, RobotConfig::FRONT_RIGHT.thigh);
  configure(JointType::Knee, RobotConfig::FRONT_RIGHT.knee);
  Serial.println("Front-right 3D IK test. PWM off. Type help.");
}

void loop() {
  // Bound serial work so motion updates cannot be starved by continuous input.
  for (int n = 0; n < 96 && Serial.available(); ++n) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (overflow) Serial.println("Line too long; discarded.");
      else if (length) { line[length] = '\0'; execute(line); }
      length = 0; overflow = false;
    } else if (!overflow) {
      if (length < sizeof(line) - 1) line[length++] = c;
      else overflow = true;
    }
  }
  const uint32_t now = millis();
  if (!moving || now - lastUpdate < RobotConfig::MOTION_UPDATE_MS) return;
  lastUpdate = now;
  const uint32_t elapsed = now - started;
  const FootPosition p = elapsed >= MOVE_MS ? target :
      interpolate(smoothstep01(static_cast<float>(elapsed) / MOVE_MS));
  if (!validate(p, false) || !leg.setFootPosition(p.x, p.y, p.z)) {
    moving = false;
    Serial.println("Motion stopped: target rejected; holding last command.");
  } else if (elapsed >= MOVE_MS) {
    moving = false; Serial.println("Move complete."); status();
  }
}
