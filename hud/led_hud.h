#pragma once
#include <FastLED.h>
#include "../radar/ld2450.h"
#include "../imu/bno055_imu.h"

class LedHud {
public:
    void begin();
    // Updates both strips based on radar targets, compensating for head yaw
    void update(const LD2450& radar, const HeadOrientation& head);
    void clear();

private:
    CRGB _left[8];
    CRGB _right[8];

    void    showTarget(const RadarTarget& t, float headYaw);
    CRGB    distColor(float dist_mm) const;
    // Maps abs(angle) 0–90° to LED index 0–7 (0 = centre, 7 = far side)
    uint8_t angleToBin(float abs_angle_deg) const;
};
