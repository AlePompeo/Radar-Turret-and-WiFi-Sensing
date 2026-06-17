#include "led_hud.h"
#include "../config.h"

void LedHud::begin() {
    FastLED.addLeds<WS2812, LED_LEFT_PIN,  GRB>(_left,  LED_COUNT);
    FastLED.addLeds<WS2812, LED_RIGHT_PIN, GRB>(_right, LED_COUNT);
    FastLED.setBrightness(LED_BRIGHTNESS);
    clear();
}

void LedHud::update(const LD2450& radar, const HeadOrientation& head) {
    fill_solid(_left,  LED_COUNT, CRGB::Black);
    fill_solid(_right, LED_COUNT, CRGB::Black);

    for (uint8_t i = 0; i < LD2450::MAX_TARGETS; i++) {
        if (radar.target(i).valid)
            showTarget(radar.target(i), head.yaw);
    }
    FastLED.show();
}

void LedHud::clear() {
    fill_solid(_left,  LED_COUNT, CRGB::Black);
    fill_solid(_right, LED_COUNT, CRGB::Black);
    FastLED.show();
}

void LedHud::showTarget(const RadarTarget& t, float headYaw) {
    // Compensate: world_angle = radar_angle – headYaw (headYaw rotates the frame)
    float angle = t.angle_deg() - headYaw;
    // Normalise to -180..180
    while (angle >  180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;

    uint8_t bin   = angleToBin(fabsf(angle));
    CRGB    color = distColor(t.distance_mm());

    if (angle < 0) _left[7 - bin]  = color;  // bin 0 = centre, 7 = far left
    else           _right[bin]     = color;  // bin 0 = centre, 7 = far right
}

CRGB LedHud::distColor(float dist_mm) const {
    if (dist_mm < DIST_NEAR)   return CRGB::Red;
    if (dist_mm < DIST_MEDIUM) return CRGB::Yellow;
    return CRGB::Green;
}

uint8_t LedHud::angleToBin(float abs_angle_deg) const {
    // Clamp 0–90° → index 0–7
    float clamped = min(abs_angle_deg, 90.0f);
    return (uint8_t)(clamped / 90.0f * 7.0f + 0.5f);
}
