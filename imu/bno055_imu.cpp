#include "bno055_imu.h"

bool IMU::begin() {
    if (!_bno.begin()) return false;
    _bno.setExtCrystalUse(true);
    _ready = true;
    return true;
}

HeadOrientation IMU::read() {
    sensors_event_t event;
    _bno.getEvent(&event);
    return {
        (float)event.orientation.x,   // yaw
        (float)event.orientation.y,   // pitch
        (float)event.orientation.z    // roll
    };
}
