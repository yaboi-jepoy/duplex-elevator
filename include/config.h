#pragma once
#include <Arduino.h>

// Analog button ladders (3.3V -> 10k pull-up -> ADC;
// each button connects ADC to GND via its own resistor).
constexpr uint8_t PIN_BUTTON_A = 1;
constexpr uint8_t PIN_BUTTON_B = 2;

constexpr uint8_t MOTOR_A_PINS[4] = {4, 5, 6, 7};   // IN1, IN2, IN3, IN4
constexpr uint8_t MOTOR_B_PINS[4] = {8, 9, 10, 11};

// All ADC values are in millivolts; calibrate from serial readings.
constexpr int FLOOR1_MIN_MV = 150;
constexpr int FLOOR1_MAX_MV = 520;
constexpr int FLOOR2_MIN_MV = 600;
constexpr int FLOOR2_MAX_MV = 1080;
constexpr int FLOOR3_MIN_MV = 1150;
constexpr int FLOOR3_MAX_MV = 1650;

// The car starts at MIN_HEIGHT_MM (Floor 1). Floor 2 and Floor 3 are
// automatically spaced between MIN_HEIGHT_MM and MAX_HEIGHT_MM.
constexpr float MIN_HEIGHT_MM = 0.0f;
constexpr float MAX_HEIGHT_MM = 381.0f; // About 15 inches of travel.
constexpr uint8_t FLOOR_COUNT = 3;
constexpr float SPOOL_DIAMETER_MM = 25.0f;
constexpr uint32_t MOTOR_HALFSTEPS_PER_REV = 4096;
constexpr float MOTOR_SPEED_RPM = 15.0f;
constexpr uint32_t HALFSTEP_INTERVAL_US = static_cast<uint32_t>(
    60000000.0f / (MOTOR_SPEED_RPM * MOTOR_HALFSTEPS_PER_REV));
constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;

static_assert(MAX_HEIGHT_MM > MIN_HEIGHT_MM, "MAX_HEIGHT_MM must exceed MIN_HEIGHT_MM");
static_assert(MOTOR_SPEED_RPM > 0.0f, "MOTOR_SPEED_RPM must be positive");
