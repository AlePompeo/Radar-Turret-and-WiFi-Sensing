#include "alert.h"
#include "../config.h"

void Buzzer::begin() {
    ledcSetup(BUZZER_LEDC_CH, 1000, 8);
    ledcAttachPin(BUZZER_PIN, BUZZER_LEDC_CH);
    ledcWrite(BUZZER_LEDC_CH, 0);  // silent
}

void Buzzer::setLevel(AlertLevel level) {
    if (_level == level) return;
    _level = level;
    if (level == AlertLevel::NONE) {
        ledcWrite(BUZZER_LEDC_CH, 0);
        _on = false;
    } else {
        ledcSetup(BUZZER_LEDC_CH, toneFreq(), 8);  // reconfigure only on level change
    }
    _lastToggle = 0;
}

void Buzzer::update() {
    if (_level == AlertLevel::NONE) return;

    uint32_t now = millis();
    if (now - _lastToggle >= periodMs()) {
        _lastToggle = now;
        _on = !_on;
        ledcWrite(BUZZER_LEDC_CH, _on ? 128 : 0);
    }
}

uint32_t Buzzer::periodMs() const {
    switch (_level) {
        case AlertLevel::NEAR:   return 100;
        case AlertLevel::MEDIUM: return 350;
        case AlertLevel::FAR:    return 900;
        default:                 return 0;
    }
}

uint16_t Buzzer::toneFreq() const {
    switch (_level) {
        case AlertLevel::NEAR:   return BUZZ_FREQ_NEAR;
        case AlertLevel::MEDIUM: return BUZZ_FREQ_MED;
        case AlertLevel::FAR:    return BUZZ_FREQ_FAR;
        default:                 return 0;
    }
}
