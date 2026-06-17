#include "servo_tracker.h"
#include "../config.h"
#include <ESP32Servo.h>
#include <math.h>

static Servo _servo;

void ServoTracker::begin() {
    _servo.setPeriodHertz(50);
    bool ok = _servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
    Serial.printf("[SERVO] attach GPIO%d stop=%d min=%d max=%d -> %s\n",
                  SERVO_PIN, SERVO_STOP_US, SERVO_MIN_US, SERVO_MAX_US,
                  ok ? "OK" : "FAIL");
    _servo.writeMicroseconds(SERVO_STOP_US + _stopTrim);
    Serial.printf("[SERVO] stopped (neutral %d us)\n", SERVO_STOP_US + _stopTrim);
}

void ServoTracker::update(const LD2450& radar) {
    const int stopUs = SERVO_STOP_US + _stopTrim;
    uint32_t  now    = millis();

    float azimuth  = selectAzimuth(radar);
    bool  hasTarget = !isnan(azimuth);
    int   targetDeviation = 0;

    if (hasTarget) {
        _lastTargetMs  = now;
        _targetAzimuth = azimuth;
        _state         = ServoState::TRACKING;

        // Hysteresis deadband: enter stop zone at DEADBAND_DEG, leave only at REACTIVATE_DEG.
        // Prevents jitter while still centering precisely.
        if (fabsf(azimuth) <= SERVO_DEADBAND_DEG)
            _inDeadband = true;
        else if (fabsf(azimuth) >= SERVO_REACTIVATE_DEG)
            _inDeadband = false;

        if (!_inDeadband) {
            float dir   = SERVO_INVERT ? -1.0f : 1.0f;
            targetDeviation = (int)(dir * SERVO_KP * azimuth);
            targetDeviation = constrain(targetDeviation, -SERVO_MAX_DEV_NEG, SERVO_MAX_DEV_POS);
        }

    } else {
        _targetAzimuth = 0.0f;

        if (_state == ServoState::TRACKING &&
            (now - _lastTargetMs) >= SERVO_IDLE_MS) {
            _state      = ServoState::IDLE;
            _inDeadband = false;
        }
        // IDLE: targetDeviation stays 0 → servo decelerates to stop via slew rate
    }

    // Slew rate limiter on OUTPUT — prevents sudden direction flips
    int slewDelta = targetDeviation - _currentDeviation;
    slewDelta = constrain(slewDelta, -SERVO_SLEW_US_PER_CALL, SERVO_SLEW_US_PER_CALL);
    _currentDeviation += slewDelta;

    _servo.writeMicroseconds(stopUs + _currentDeviation);
}

void ServoTracker::sweep() {
    Serial.println("[SERVO] sweep start");
    // For continuous rotation: pulse < 1500 = one dir, > 1500 = other dir
    struct { int us; int ms; } steps[] = {
        {SERVO_MIN_US,  600},   // full speed dir A
        {SERVO_STOP_US, 300},   // stop
        {SERVO_MAX_US,  600},   // full speed dir B
        {SERVO_STOP_US, 300},   // stop
    };
    for (auto& s : steps) {
        _servo.writeMicroseconds(s.us);
        Serial.printf("[SERVO] sweep -> %d us\n", s.us);
        delay(s.ms);
    }
    Serial.println("[SERVO] sweep done");
}

float ServoTracker::selectAzimuth(const LD2450& radar) const {
    switch (_mode) {
        case ServoTrackMode::HIGHEST_THREAT: {
            float bestScore = -1.0f;
            const RadarTarget* best = nullptr;
            for (uint8_t i = 0; i < LD2450::MAX_TARGETS; i++) {
                const RadarTarget& t = radar.target(i);
                if (!t.valid) continue;
                float s = threatScore(t);
                if (s > bestScore) { bestScore = s; best = &t; }
            }
            return best ? best->angle_deg() : NAN;
        }
        case ServoTrackMode::NEAREST: {
            float minDist = 1e9f;
            const RadarTarget* nearest = nullptr;
            for (uint8_t i = 0; i < LD2450::MAX_TARGETS; i++) {
                const RadarTarget& t = radar.target(i);
                if (!t.valid) continue;
                float d = t.distance_mm();
                if (d < minDist) { minDist = d; nearest = &t; }
            }
            return nearest ? nearest->angle_deg() : NAN;
        }
        case ServoTrackMode::AVERAGE: {
            float sumX = 0, sumY = 0;
            uint8_t n = 0;
            for (uint8_t i = 0; i < LD2450::MAX_TARGETS; i++) {
                const RadarTarget& t = radar.target(i);
                if (!t.valid) continue;
                sumX += t.x_mm; sumY += t.y_mm; n++;
            }
            if (n == 0) return NAN;
            return atan2f(sumX / n, sumY / n) * 57.2958f;
        }
    }
    return NAN;
}

float ServoTracker::threatScore(const RadarTarget& t) const {
    float distScore  = 8000.0f / (t.distance_mm() + 1.0f);
    float speedScore = max(0.0f, -(float)t.speed_cms) * 0.05f;
    return distScore + speedScore;
}
