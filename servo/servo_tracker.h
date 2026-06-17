#pragma once
#include <Arduino.h>
#include "../config.h"
#include "../radar/ld2450.h"
#include "../imu/bno055_imu.h"

enum class ServoTrackMode : uint8_t {
    HIGHEST_THREAT = 0,
    NEAREST        = 1,
    AVERAGE        = 2
};

enum class ServoState : uint8_t {
    IDLE,       // no target ever seen, or search timed out
    TRACKING,   // target visible, following it
    SEARCHING   // unused — kept for web UI compatibility
};

class ServoTracker {
public:
    void begin();
    void update(const LD2450& radar);
    void sweep();  // diagnostic: 0° → 90° → 180° → 90°, call once

    // Returns display angle 0-180° (90=center) for web UI
    float           currentAngleDeg() const { return _targetAzimuth + 90.0f; }
    ServoTrackMode  mode()            const { return _mode; }
    ServoState      state()           const { return _state; }
    void            setMode(ServoTrackMode m) { _mode = m; }
    void            setStopTrim(int us)       { _stopTrim = us; }

private:
    ServoTrackMode _mode             = ServoTrackMode::HIGHEST_THREAT;
    ServoState     _state            = ServoState::IDLE;
    float          _targetAzimuth   = 0.0f;
    int            _currentDeviation = 0;
    bool           _inDeadband       = false;
    uint32_t       _lastTargetMs     = 0;
    int            _stopTrim         = SERVO_STOP_TRIM;

    // Returns the radar azimuth to track, or NaN if no valid target.
    float selectAzimuth(const LD2450& radar) const;
    float threatScore(const RadarTarget& t) const;
};
