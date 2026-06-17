#include "csi_sensing.h"
#include <math.h>

static CsiSensing* s_csi = nullptr;

// Keep callback minimal — runs inside the WiFi driver task.
// Heavy computation (variance) stays in update() on the main loop.
static void csiRxCb(void* ctx, wifi_csi_info_t* info) {
    (void)ctx;
    if (s_csi && info && info->buf && info->len > 0)
        s_csi->_onCsi(info);
}

void CsiSensing::begin() {
    s_csi = this;

    wifi_csi_config_t cfg{};
    cfg.lltf_en           = true;
    cfg.htltf_en          = true;
    cfg.stbc_htltf2_en    = true;
    cfg.ltf_merge_en      = true;
    cfg.channel_filter_en = false;
    cfg.manu_scale        = false;

    esp_wifi_set_csi_config(&cfg);
    esp_wifi_set_csi_rx_cb(csiRxCb, nullptr);
    esp_wifi_set_csi(true);

    _calStart = millis();
}

void CsiSensing::update() {
    uint32_t now = millis();

    // Copy CSI window under spinlock — callback writes from Core 0 / WiFi driver task
    float   csiSnap[CSI_WIN];
    uint8_t csiCnt;
    portENTER_CRITICAL(&_csiMux);
    memcpy(csiSnap, _csiWin, sizeof(_csiWin));
    csiCnt = _csiCount;
    portEXIT_CRITICAL(&_csiMux);

    // CSI calibration: first 5 s must be still
    if (!_csiCalib && now - _calStart > 5000 && csiCnt >= CSI_WIN) {
        _csiBaseline = variance(csiSnap, csiCnt);
        _csiCalib    = true;
    }
    if (_csiCalib) {
        float v  = variance(csiSnap, csiCnt);
        float th = _csiBaseline * CSI_THRESH + 0.01f;
        _csiScore = min(v / th, 1.0f);
    }

    // RSSI sampling from connected stations
    if (now - _lastRssiMs >= RSSI_PERIOD) {
        sampleRssi();
        _lastRssiMs = now;
    }
    if (!_rssiCalib && _csiCalib && _rssiCount >= RSSI_WIN) {
        _rssiBaseline = variance(_rssiWin, _rssiCount);
        _rssiCalib    = true;
    }
    if (_rssiCalib) {
        float v  = variance(_rssiWin, _rssiCount);
        float th = _rssiBaseline * 3.0f + 0.5f;
        _rssiScore = min(v / th, 1.0f);
    }

    float csi = _csiCalib  ? _csiScore  : 0.0f;
    float rss = _rssiCalib ? _rssiScore : 0.0f;
    _score   = max(csi, rss);
    _present = _score > 0.5f;
}

// Called from WiFi driver task (Core 0) — must be fast, no heap alloc, no heavy math.
// Uses portENTER_CRITICAL_ISR (spinlock only, no interrupt disable) — safe from task context.
void CsiSensing::_onCsi(wifi_csi_info_t* info) {
    int n = info->len / 2;
    if (n == 0) return;
    float sum = 0.0f;
    const int8_t* buf = info->buf;
    for (int i = 0; i < n; i++) {
        float re = buf[2*i + 1];
        float im = buf[2*i];
        sum += re*re + im*im;
    }
    float sample = sum / n;
    portENTER_CRITICAL_ISR(&_csiMux);
    _csiWin[_csiIdx] = sample;
    _csiIdx = (_csiIdx + 1) % CSI_WIN;
    if (_csiCount < CSI_WIN) _csiCount++;
    portEXIT_CRITICAL_ISR(&_csiMux);
}

void CsiSensing::sampleRssi() {
    wifi_sta_list_t sta{};
    if (esp_wifi_ap_get_sta_list(&sta) != ESP_OK || sta.num == 0) return;
    _lastRssi = sta.sta[0].rssi;
    _rssiWin[_rssiIdx] = (float)_lastRssi;
    _rssiIdx = (_rssiIdx + 1) % RSSI_WIN;
    if (_rssiCount < RSSI_WIN) _rssiCount++;
}

float CsiSensing::variance(const float* buf, uint8_t n) const {
    if (n < 2) return 0.0f;
    float mean = 0.0f;
    for (uint8_t i = 0; i < n; i++) mean += buf[i];
    mean /= n;
    float var = 0.0f;
    for (uint8_t i = 0; i < n; i++) { float d = buf[i] - mean; var += d*d; }
    return var / n;
}
