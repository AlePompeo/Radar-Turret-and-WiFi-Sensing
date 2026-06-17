#pragma once
#include <ESPAsyncWebServer.h>
#include "../sensor_snapshot.h"
#include "../servo/servo_tracker.h"

class WebApp {
public:
    void setServoTracker(ServoTracker* t) { _servo = t; }
    void begin();
    void pushUpdate(const SensorSnapshot& snap);

private:
    AsyncWebServer _server{80};
    AsyncWebSocket _ws{"/ws"};
    ServoTracker*  _servo = nullptr;

    int buildJson(const SensorSnapshot& snap, char* buf, size_t size) const;
};
