#pragma once

#include "Leg.h"

#include <cstdint>

float smoothstep01(float t);
void startLegMove(Leg &leg, FootPosition target, std::uint32_t durationMs, std::uint32_t nowMs);
void updateLegMotion(Leg &leg, std::uint32_t nowMs);
bool isLegMoving(const Leg &leg);


// Commands the same logical local foot pose to every enabled leg.
// Disabled legs are ignored. Returns false if any enabled leg rejects the target.
bool commandStandingPose(Leg *legs[], int legCount, FootPosition target);
