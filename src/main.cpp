#include <Arduino.h>
#include <math.h>
#include <stdarg.h>
#include "config.h"

// Standard half-step phase sequence for 28BYJ-48 via ULN2003 IN1..IN4.
static constexpr uint8_t PHASES[8][4] = {
    {1,0,0,0}, {1,1,0,0}, {0,1,0,0}, {0,1,1,0},
    {0,0,1,0}, {0,0,1,1}, {0,0,0,1}, {1,0,0,1}
};

struct Elevator {
    char name;
    uint8_t adcPin;
    const uint8_t *motorPins;
    int currentFloor = 1;
    int targetFloor = 1;
    int32_t positionSteps = 0;
    int32_t targetSteps = 0;
    uint8_t phase = 0;
    bool moving = false;
    int candidateButton = 0;
    int stableButton = 0;
    uint32_t candidateSince = 0;
    uint32_t lastStepUs = 0;

    Elevator(char name, uint8_t adcPin, const uint8_t *motorPins)
        : name(name), adcPin(adcPin), motorPins(motorPins) {}
};

static Elevator carA{'A', PIN_BUTTON_A, MOTOR_A_PINS};
static Elevator carB{'B', PIN_BUTTON_B, MOTOR_B_PINS};
static SemaphoreHandle_t serialMutex;

static void logLine(const char *fmt, ...) {
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(250)) != pdTRUE) return;
    char message[192];
    va_list args;
    va_start(args, fmt);
    vsnprintf(message, sizeof(message), fmt, args);
    va_end(args);
    Serial.printf("[%8lu ms][Core %d] %s\n",
                  (unsigned long)millis(), xPortGetCoreID(), message);
    xSemaphoreGive(serialMutex);
}

static int decodeFloor(int mv) {
    if (mv >= FLOOR1_MIN_MV && mv <= FLOOR1_MAX_MV) return 1;
    if (mv >= FLOOR2_MIN_MV && mv <= FLOOR2_MAX_MV) return 2;
    if (mv >= FLOOR3_MIN_MV && mv <= FLOOR3_MAX_MV) return 3;
    return 0; // No button, or an invalid/ambiguous voltage.
}

static void writePhase(const Elevator &e, uint8_t phase) {
    for (int i = 0; i < 4; i++)
        digitalWrite(e.motorPins[i], PHASES[phase][i]);
}

static void releaseMotor(const Elevator &e) {
    for (int i = 0; i < 4; i++) digitalWrite(e.motorPins[i], LOW);
}

static int32_t stepsAtHeight(float heightMm) {
    const float spoolCircumferenceMm = PI * SPOOL_DIAMETER_MM;
    const float cableRevolutions = (heightMm - MIN_HEIGHT_MM) / spoolCircumferenceMm;
    return static_cast<int32_t>(lroundf(cableRevolutions * MOTOR_HALFSTEPS_PER_REV));
}

static int32_t stepsForFloor(int floor) {
    const int clampedFloor = constrain(floor, 1, FLOOR_COUNT);
    const float floorHeightMm = MIN_HEIGHT_MM +
        (MAX_HEIGHT_MM - MIN_HEIGHT_MM) * (clampedFloor - 1) / (FLOOR_COUNT - 1);
    return stepsAtHeight(floorHeightMm);
}

static void requestFloor(Elevator &e, int floor) {
    e.targetFloor = floor;
    e.targetSteps = stepsForFloor(floor);
    if (e.positionSteps == e.targetSteps) {
        e.moving = false;
    } else {
        e.moving = true;
    }
}

static void pollButtons(Elevator &e, uint32_t nowMs) {
    const int voltageMv = analogReadMilliVolts(e.adcPin);
    const int rawButton = decodeFloor(voltageMv);

    if (rawButton != e.candidateButton) {
        e.candidateButton = rawButton;
        e.candidateSince = nowMs;
    }
    if (rawButton != e.stableButton &&
        (uint32_t)(nowMs - e.candidateSince) >= BUTTON_DEBOUNCE_MS) {
        e.stableButton = rawButton;
        if (rawButton >= 1 && rawButton <= FLOOR_COUNT) {
            logLine("Car %c: Floor %d button pressed (%d mV)", e.name, rawButton, voltageMv);
            requestFloor(e, rawButton);
        }
    }
}

static void updateMotor(Elevator &e, uint32_t nowUs) {
    if (!e.moving) return;
    if ((uint32_t)(nowUs - e.lastStepUs) < HALFSTEP_INTERVAL_US) return;
    e.lastStepUs = nowUs;

    int direction = (e.targetSteps > e.positionSteps) ? 1 : -1;
    e.phase = (uint8_t)((e.phase + (direction > 0 ? 1 : 7)) & 7);
    writePhase(e, e.phase);
    e.positionSteps += direction;

    if (e.positionSteps == e.targetSteps) {
        e.moving = false;
        e.currentFloor = e.targetFloor;
        releaseMotor(e); // Demo only: removes holding torque.
    }
}

static void elevatorTask(void *arg) {
    Elevator &e = *static_cast<Elevator *>(arg);
    for (;;) {
        const uint32_t nowMs = millis();
        pollButtons(e, nowMs);
        updateMotor(e, micros());
        vTaskDelay(1); // Yield; both tasks run independently.
    }
}

void setup() {
    Serial.begin(115200);
    delay(1800);
    serialMutex = xSemaphoreCreateMutex();
    if (serialMutex == nullptr) { Serial.println("Mutex creation failed"); abort(); }

    analogReadResolution(12);
    analogSetPinAttenuation(PIN_BUTTON_A, ADC_11db);
    analogSetPinAttenuation(PIN_BUTTON_B, ADC_11db);

    for (uint8_t pin : MOTOR_A_PINS) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }
    for (uint8_t pin : MOTOR_B_PINS) { pinMode(pin, OUTPUT); digitalWrite(pin, LOW); }

    logLine("Configured travel: %.1f mm, spool: %.1f mm, floor spacing: %.1f mm",
            MAX_HEIGHT_MM - MIN_HEIGHT_MM, SPOOL_DIAMETER_MM,
            (MAX_HEIGHT_MM - MIN_HEIGHT_MM) / 2.0f);
    logLine("Motor speed: %.1f RPM (%lu us per half-step)",
            MOTOR_SPEED_RPM, static_cast<unsigned long>(HALFSTEP_INTERVAL_US));
    logLine("Target positions: F1=%ld, F2=%ld, F3=%ld half-steps",
            static_cast<long>(stepsForFloor(1)),
            static_cast<long>(stepsForFloor(2)),
            static_cast<long>(stepsForFloor(3)));

    BaseType_t a = xTaskCreatePinnedToCore(elevatorTask, "ElevatorTaskA", 4096,
                                          &carA, 2, nullptr, 0);
    BaseType_t b = xTaskCreatePinnedToCore(elevatorTask, "ElevatorTaskB", 4096,
                                          &carB, 2, nullptr, 1);
    if (a != pdPASS || b != pdPASS) {
        logLine("ERROR: Cannot create elevator tasks!");
        abort();
    }
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
