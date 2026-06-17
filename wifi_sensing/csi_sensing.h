#pragma once
#include <Arduino.h>
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

class CsiSensing {
public:
    void begin();
    void update();

    bool   humanPresent()  const { return _present; }
    float  presenceScore() const { return _score; }
    float  csiScore()      const { return _csiScore; }
    float  rssiScore()     const { return _rssiScore; }
    int8_t lastRssi()      const { return _lastRssi; }
    bool   csiCalibrated() const { return _csiCalib; }
    bool   rssiCalibrated()const { return _rssiCalib; }

    void _onCsi(wifi_csi_info_t* info);

private:
    // CSI variance window
    static constexpr uint8_t  CSI_WIN    = 40;
    static constexpr float    CSI_THRESH = 2.5f;

    portMUX_TYPE _csiMux  = portMUX_INITIALIZER_UNLOCKED;
    float    _csiWin[CSI_WIN]{};
    uint8_t  _csiIdx      = 0;
    uint8_t  _csiCount    = 0;
    float    _csiBaseline = 0.0f;
    bool     _csiCalib    = false;
    uint32_t _calStart    = 0;
    float    _csiScore    = 0.0f;

    // RSSI variance from connected AP clients
    static constexpr uint8_t  RSSI_WIN    = 20;
    static constexpr uint32_t RSSI_PERIOD = 200;  // ms

    float    _rssiWin[RSSI_WIN]{};
    uint8_t  _rssiIdx      = 0;
    uint8_t  _rssiCount    = 0;
    float    _rssiBaseline = 0.0f;
    bool     _rssiCalib    = false;
    uint32_t _lastRssiMs   = 0;
    int8_t   _lastRssi     = -100;
    float    _rssiScore    = 0.0f;

    float  _score   = 0.0f;
    bool   _present = false;

    void  sampleRssi();
    float variance(const float* buf, uint8_t n) const;
    float meanAmplitude(const wifi_csi_info_t* info) const;
};
