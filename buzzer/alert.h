#pragma once
#include <Arduino.h>

enum class AlertLevel { NONE, FAR, MEDIUM, NEAR };

class Buzzer {
public:
    void begin();
    void setLevel(AlertLevel level);
    void update();   // non-blocking, call every loop()

private:
    AlertLevel _level     = AlertLevel::NONE;
    uint32_t   _lastToggle = 0;
    bool       _on         = false;

    uint32_t periodMs() const;
    uint16_t toneFreq()  const;
};
