#pragma once
#include <Arduino.h>

struct RadarTarget {
    int16_t  x_mm;        // negative = left, positive = right
    int16_t  y_mm;        // positive = forward
    int16_t  speed_cms;   // negative = approaching, positive = receding
    uint16_t res_mm;
    bool     valid;
    float    cachedDist;  // precomputed sqrtf — avoids repeated calls per frame
    float    cachedAngle; // precomputed atan2f — avoids repeated calls per frame

    // angle in degrees: 0 = straight ahead, negative = left, positive = right
    float distance_mm() const { return cachedDist; }
    float angle_deg()   const { return cachedAngle; }
};

class LD2450 {
public:
    static constexpr uint8_t MAX_TARGETS = 3;

    void begin(HardwareSerial& serial);
    bool update();   // returns true when a new frame is parsed

    const RadarTarget& target(uint8_t i) const { return _targets[i]; }
    uint8_t activeTargets() const;
    float   nearestDistance() const;  // 0 if no valid targets

private:
    HardwareSerial* _serial = nullptr;
    RadarTarget     _targets[MAX_TARGETS]{};
    uint8_t         _buf[30]{};
    uint8_t         _bufIdx = 0;

    bool parseFrame();
    void sendMultiTargetMode();
};
