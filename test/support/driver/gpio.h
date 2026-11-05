#pragma once
// Native substitute only; actual firmware uses ESP-IDF's pin capability check.
#define GPIO_IS_VALID_OUTPUT_GPIO(pin) ((pin) >= 0 && (pin) <= 33)
