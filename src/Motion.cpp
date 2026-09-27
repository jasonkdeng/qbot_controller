#include "Motion.h"

#include <algorithm>

float smoothstep01(float t) {
  const float clamped = std::max(0.0f, std::min(1.0f, t));
  return clamped * clamped * (3.0f - 2.0f * clamped);
}

void startLegMove(Leg &leg, FootPosition target, std::uint32_t durationMs, std::uint32_t nowMs) {
  LegMotionState &state = leg.motionState();
  state.start = leg.currentPosition();
  state.target = target;
  state.startTimeMs = nowMs;
  state.durationMs = durationMs;

  if (durationMs == 0) {
    leg.setFootPosition(target.x, target.y, target.z);
    state.moving = false;
    return;
  }

  state.moving = true;
}

void updateLegMotion(Leg &leg, std::uint32_t nowMs) {
  LegMotionState &state = leg.motionState();
  if (!state.moving) {
    return;
  }

  const std::uint32_t elapsed = nowMs - state.startTimeMs;
  if (elapsed >= state.durationMs) {
    if (leg.setFootPosition(state.target.x, state.target.y, state.target.z)) {
      state.moving = false;
    }
    return;
  }

  const float t = static_cast<float>(elapsed) / static_cast<float>(state.durationMs);
  const float s = smoothstep01(t);
  FootPosition p{
      state.start.x + (state.target.x - state.start.x) * s,
      state.start.y + (state.target.y - state.start.y) * s,
      state.start.z + (state.target.z - state.start.z) * s};

  leg.setFootPosition(p.x, p.y, p.z);
}

bool isLegMoving(const Leg &leg) {
  return leg.motionState().moving;
}


bool commandStandingPose(Leg *legs[], int legCount, FootPosition target) {
  bool success = true;
  for (int i = 0; i < legCount; ++i) {
    Leg *leg = legs[i];
    if (leg == nullptr || !leg->isEnabled()) {
      continue;
    }
    if (!leg->setFootPosition(target.x, target.y, target.z)) {
      success = false;
    }
  }
  return success;
}
