#include <Arduino.h>
#include <ESP32Servo.h>
#include <driver/gpio.h>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "RobotConfig.h"

// Standalone firmware: build/upload with `pio run -e leg_calibration -t upload`.
// Nothing attaches until an explicit `arm <raw-degrees>` command.
namespace {
struct Calibration {
  RobotConfig::JointConfig config;
  bool neutralSet = false;
  bool directionSet = false;
  bool minSet = false;
  bool maxSet = false;
};

const char *LEG_NAMES[] = {"FRONT_RIGHT", "FRONT_LEFT", "REAR_RIGHT", "REAR_LEFT"};
const RobotConfig::LegConfig LEG_CONFIGS[] = {
    RobotConfig::FRONT_RIGHT, RobotConfig::FRONT_LEFT,
    RobotConfig::REAR_RIGHT, RobotConfig::REAR_LEFT};
const char *JOINT_NAMES[] = {"hip", "thigh", "knee"};
Calibration joints[3];
Servo servo;
int selectedLeg = -1;
int selectedJoint = 0;
float rawAngle = 90.0f;
char line[96];
size_t lineLength = 0;
bool overflow = false;

void stop() {
  if (servo.attached()) servo.detach();
}

void help() {
  Serial.println(
      "leg FR|FL|RR|RL  (detaches and discards this session's calibration)\n"
      "joint hip|thigh|knee  (detaches previous joint)\n"
      "pin <GPIO>  (while detached; choose a free output pin on your board)\n"
      "arm <raw degrees 0..180>  (attaches and moves immediately)\n"
      "step <-5..5 degrees>  (small raw-servo adjustment)\n"
      "neutral  (mark current position as mathematical zero)\n"
      "direction 1|-1  (positive joint angle increases/decreases raw angle)\n"
      "min / max  (mark current joint angle as a measured safe limit)\n"
      "zero  (return gradually to the marked neutral using step)\n"
      "status / export / off / help\n"
      "Calibration is RAM-only. Export and copy results before reset.\n"
      "Support the leg: off detaches PWM and releases holding torque.");
}

bool number(const char *text, float &value) {
  if (!text) return false;
  char *end = nullptr;
  value = std::strtof(text, &end);
  return end != text && *end == '\0' && std::isfinite(value);
}

void writeRaw(float angle) {
  rawAngle = angle;
  const int pulse = static_cast<int>(RobotConfig::SERVO_MIN_US +
      angle / 180.0f * (RobotConfig::SERVO_MAX_US - RobotConfig::SERVO_MIN_US));
  servo.writeMicroseconds(pulse);
}

void status() {
  if (selectedLeg < 0) {
    Serial.println("Select a leg first: leg FL");
    return;
  }
  Serial.printf("%s; selected %s; PWM %s\n", LEG_NAMES[selectedLeg],
                JOINT_NAMES[selectedJoint], servo.attached() ? "on" : "off");
  for (int i = 0; i < 3; ++i) {
    const Calibration &j = joints[i];
    Serial.printf("%s: pin=%d neutral=%.2f direction=%d limits=[%.2f, %.2f] "
                  "marked neutral/direction/min/max=%d/%d/%d/%d\n",
                  JOINT_NAMES[i], j.config.pin, j.config.neutral, j.config.direction,
                  j.config.minAngle, j.config.maxAngle,
                  j.neutralSet, j.directionSet, j.minSet, j.maxSet);
  }
  if (servo.attached()) Serial.printf("Current raw command: %.2f degrees\n", rawAngle);
}

bool complete(const Calibration &j) {
  const auto &c = j.config;
  const float rawMin = c.neutral + c.direction * c.minAngle;
  const float rawMax = c.neutral + c.direction * c.maxAngle;
  return j.neutralSet && j.directionSet && j.minSet && j.maxSet &&
      c.pin >= 0 && c.minAngle <= 0 && c.maxAngle >= 0 &&
      c.minAngle < c.maxAngle && rawMin >= 0 && rawMin <= 180 &&
      rawMax >= 0 && rawMax <= 180;
}

void exportConfig() {
  for (const auto &j : joints) {
    if (!complete(j)) {
      Serial.println("Export rejected: mark neutral, direction and both safe limits for every joint.");
      return;
    }
  }
  Serial.printf("constexpr LegConfig %s{\n    false, // Enable after individual-leg verification.\n",
                LEG_NAMES[selectedLeg]);
  for (int i = 0; i < 3; ++i) {
    const auto &c = joints[i].config;
    Serial.printf("    {%d, %.2ff, %+d, %.2ff, %.2ff}%s\n", c.pin, c.neutral,
                  c.direction, c.minAngle, c.maxAngle, i == 2 ? "};" : ",");
  }
}

void execute(char *input) {
  char *cmd = std::strtok(input, " \t");
  char *arg = std::strtok(nullptr, " \t");
  if (!cmd) return;
  if (std::strtok(nullptr, " \t")) {
    Serial.println("Too many arguments.");
    return;
  }
  if (!std::strcmp(cmd, "off")) { stop(); Serial.println("PWM off."); return; }
  if (!std::strcmp(cmd, "help")) { help(); return; }
  if (!std::strcmp(cmd, "status")) { status(); return; }
  if (!std::strcmp(cmd, "leg")) {
    const char *codes[] = {"FR", "FL", "RR", "RL"};
    for (int i = 0; i < 4; ++i) {
      if (arg && !std::strcmp(arg, codes[i])) {
        stop();
        selectedLeg = i;
        selectedJoint = 0;
        joints[0] = Calibration{LEG_CONFIGS[i].hip};
        joints[1] = Calibration{LEG_CONFIGS[i].thigh};
        joints[2] = Calibration{LEG_CONFIGS[i].knee};
        status();
        return;
      }
    }
    Serial.println("Use leg FR, FL, RR or RL.");
    return;
  }
  if (selectedLeg < 0) { Serial.println("Select a leg first."); return; }
  if (!std::strcmp(cmd, "joint")) {
    for (int i = 0; i < 3; ++i) {
      if (arg && !std::strcmp(arg, JOINT_NAMES[i])) {
        stop(); selectedJoint = i; status(); return;
      }
    }
    Serial.println("Use joint hip, thigh or knee.");
    return;
  }
  if (!std::strcmp(cmd, "export")) { exportConfig(); return; }
  Calibration &j = joints[selectedJoint];
  float value = 0;
  if (!std::strcmp(cmd, "pin")) {
    if (servo.attached() || !number(arg, value) || value < 0 || value > 48 ||
        std::floor(value) != value || !GPIO_IS_VALID_OUTPUT_GPIO(static_cast<int>(value))) {
      Serial.println("Detach first; specify a valid output GPIO."); return;
    }
    for (int i = 0; i < 3; ++i) {
      if (i != selectedJoint && joints[i].config.pin == static_cast<int>(value)) {
        Serial.println("GPIO already assigned to another joint."); return;
      }
    }
    j.config.pin = static_cast<int>(value);
    j.neutralSet = j.directionSet = j.minSet = j.maxSet = false;
  } else if (!std::strcmp(cmd, "arm")) {
    if (servo.attached() || j.config.pin < 0 || !number(arg, value) || value < 0 || value > 180) {
      Serial.println("Assign pin first; detach before arm <0..180>."); return;
    }
    servo.setPeriodHertz(RobotConfig::SERVO_FREQUENCY_HZ);
    servo.attach(j.config.pin, RobotConfig::SERVO_MIN_US, RobotConfig::SERVO_MAX_US);
    if (!servo.attached()) { Serial.println("Attach failed."); return; }
    writeRaw(value);
  } else if (!std::strcmp(cmd, "direction")) {
    if (!number(arg, value) || (value != 1 && value != -1)) {
      Serial.println("Use direction 1 or direction -1."); return;
    }
    j.config.direction = static_cast<int>(value);
    j.directionSet = true;
    j.minSet = j.maxSet = false;
  } else if (!std::strcmp(cmd, "zero")) {
    if (!j.neutralSet) { Serial.println("Mark neutral first."); return; }
    Serial.printf("Neutral raw angle is %.2f. Use step commands to approach it.\n", j.config.neutral);
    return;
  } else if (!std::strcmp(cmd, "step") || !std::strcmp(cmd, "neutral") ||
             !std::strcmp(cmd, "min") || !std::strcmp(cmd, "max")) {
    if (!servo.attached()) { Serial.println("Arm the selected joint first."); return; }
    if (!std::strcmp(cmd, "step")) {
      if (!number(arg, value) || std::fabs(value) > 5 || rawAngle + value < 0 || rawAngle + value > 180) {
        Serial.println("Step must be within -5..5 and result within 0..180."); return;
      }
      writeRaw(rawAngle + value);
    } else if (!std::strcmp(cmd, "neutral")) {
      j.config.neutral = rawAngle;
      j.neutralSet = true;
      j.minSet = j.maxSet = false;
    } else {
      if (!j.neutralSet || !j.directionSet) { Serial.println("Mark neutral and set direction first."); return; }
      const float angle = (rawAngle - j.config.neutral) / j.config.direction;
      if (!std::strcmp(cmd, "min")) {
        if (angle > 0) { Serial.println("Minimum must be at or below zero."); return; }
        j.config.minAngle = angle; j.minSet = true;
      } else {
        if (angle < 0) { Serial.println("Maximum must be at or above zero."); return; }
        j.config.maxAngle = angle; j.maxSet = true;
      }
    }
  } else { Serial.println("Unknown command. Use help."); return; }
  status();
}
} // namespace

void setup() {
  Serial.begin(115200);
  Serial.println("Leg calibration ready. PWM off. Type help.");
}

void loop() {
  while (Serial.available()) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (overflow) Serial.println("Line too long; command discarded.");
      else if (lineLength) { line[lineLength] = '\0'; execute(line); }
      lineLength = 0; overflow = false;
    } else if (c == '\b' || c == 127) {
      if (!overflow && lineLength) --lineLength;
    } else if (!overflow) {
      if (lineLength < sizeof(line) - 1) line[lineLength++] = c;
      else overflow = true;
    }
  }
}
