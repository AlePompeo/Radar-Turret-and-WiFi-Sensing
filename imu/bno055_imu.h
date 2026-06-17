#pragma once
#include <Wire.h>
#include <Adafruit_BNO055.h>

struct HeadOrientation {
    float yaw;    // 0–360°
    float pitch;  // degrees
    float roll;   // degrees
};

class IMU {
public:
    bool begin();
    HeadOrientation read();
    bool ready() const { return _ready; }

private:
    Adafruit_BNO055 _bno{55, 0x28};
    bool _ready = false;
};
