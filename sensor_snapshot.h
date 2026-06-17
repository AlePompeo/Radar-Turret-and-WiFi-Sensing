#pragma once
#include "radar/ld2450.h"
#include "imu/bno055_imu.h"
#include "buzzer/alert.h"
#include "servo/servo_tracker.h"

// Plain-data snapshot shared between FreeRTOS tasks.
// All fields are written under gMutex in main.cpp; readers take the mutex too.
struct SensorSnapshot {
    RadarTarget     targets[LD2450::MAX_TARGETS];
    uint8_t         activeTargets;
    float           nearestMm;
    HeadOrientation head;
    AlertLevel      alert;
    float           csiScore, rssiScore, presenceScore;
    bool            humanPresent;
    int8_t          lastRssi;
    float           servoAngleDeg;
    ServoTrackMode  servoMode;
    ServoState      servoState;
};
