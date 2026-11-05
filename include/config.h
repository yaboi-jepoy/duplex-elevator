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

// Start with the car physically at Floor 1. Adjust after testing spool.
constexpr int32_t STEPS_PER_FLOOR = 1024;
constexpr uint32_t HALFSTEP_INTERVAL_US = 3000;
constexpr uint32_t BUTTON_DEBOUNCE_MS = 50;
