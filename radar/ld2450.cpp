#include "ld2450.h"
#include "../config.h"
#include <string.h>
#include <math.h>

// Frame: AA FF 03 00 | [target0 8B][target1 8B][target2 8B] | 55 CC
// LD2450 coordinate encoding — all fields are sign-magnitude (asymmetric sign-bit polarity):
//   X:     bit15=0 → positive (right),   bit15=1 → negative (left)     [same as Speed]
//   Y:     bit15=1 → positive (forward), bit15=0 → negative (behind)   [inverted]
//   Speed: bit15=0 → positive (receding),bit15=1 → negative (approach)
static constexpr uint8_t FRAME_LEN  = 30;
static constexpr uint8_t HDR0 = 0xAA, HDR1 = 0xFF, HDR2 = 0x03, HDR3 = 0x00;
static constexpr uint8_t TAL0 = 0x55, TAL1 = 0xCC;

static int16_t decodeCoord(uint8_t lo, uint8_t hi) {
    uint16_t raw = (uint16_t)lo | ((uint16_t)hi << 8);
    if (raw & 0x8000) return  (int16_t)(raw & 0x7FFF);
    return                   -(int16_t)(raw & 0x7FFF);
}

static int16_t decodeSpeed(uint8_t lo, uint8_t hi) {
    uint16_t raw = (uint16_t)lo | ((uint16_t)hi << 8);
    if (raw & 0x8000) return -(int16_t)(raw & 0x7FFF);
    return                    (int16_t)(raw & 0x7FFF);
}

void LD2450::begin(HardwareSerial& serial) {
    _serial = &serial;
    _bufIdx = 0;
    sendMultiTargetMode();
}

void LD2450::sendMultiTargetMode() {
    // LD2450 config frame format: [FD FC FB FA] [len_lo len_hi] [cmd_lo cmd_hi] [val...] [04 03 02 01]
    static const uint8_t kEnterCfg[]    = {0xFD,0xFC,0xFB,0xFA, 0x04,0x00, 0xFF,0x00, 0x01,0x00, 0x04,0x03,0x02,0x01};
    static const uint8_t kMultiTgt[]    = {0xFD,0xFC,0xFB,0xFA, 0x02,0x00, 0x90,0x00, 0x04,0x03,0x02,0x01};
    // Disable zone filter (cmd 0x07, zone_type=0x00): detect full field of view
    static const uint8_t kNoZoneFilter[]= {0xFD,0xFC,0xFB,0xFA, 0x1C,0x00, 0x07,0x00, 0x00,0x00,
                                            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
                                            0x04,0x03,0x02,0x01};
    static const uint8_t kExitCfg[]     = {0xFD,0xFC,0xFB,0xFA, 0x02,0x00, 0xFE,0x00, 0x04,0x03,0x02,0x01};

    _serial->write(kEnterCfg,     sizeof(kEnterCfg));     delay(50);
    _serial->write(kMultiTgt,     sizeof(kMultiTgt));     delay(50);
    _serial->write(kNoZoneFilter, sizeof(kNoZoneFilter)); delay(50);
    _serial->write(kExitCfg,      sizeof(kExitCfg));      delay(100);

    while (_serial->available()) _serial->read();  // flush ACK bytes
}

bool LD2450::update() {
    while (_serial->available()) {
        uint8_t b = _serial->read();

        if (_bufIdx < FRAME_LEN) {
            _buf[_bufIdx++] = b;
        }

        if (_bufIdx == FRAME_LEN) {
            bool valid = _buf[0] == HDR0 && _buf[1] == HDR1 &&
                         _buf[2] == HDR2 && _buf[3] == HDR3 &&
                         _buf[28] == TAL0 && _buf[29] == TAL1;
            if (valid) {
                _bufIdx = 0;
                return parseFrame();
            }
            // slide window by 1 to re-sync
            memmove(_buf, _buf + 1, FRAME_LEN - 1);
            _bufIdx = FRAME_LEN - 1;
        }
    }
    return false;
}

bool LD2450::parseFrame() {
    // Computed once at first call — RADAR_AZIMUTH_OFFSET_DEG is a compile-time constant.
    static const float kCos = cosf(RADAR_AZIMUTH_OFFSET_DEG * 0.017453f);
    static const float kSin = sinf(RADAR_AZIMUTH_OFFSET_DEG * 0.017453f);

    for (uint8_t i = 0; i < MAX_TARGETS; i++) {
        uint8_t  base = 4 + i * 8;
        int16_t  x    = decodeSpeed(_buf[base],   _buf[base+1]);
        int16_t  y    = decodeCoord(_buf[base+2], _buf[base+3]);
        int16_t  spd  = decodeSpeed(_buf[base+4], _buf[base+5]);
        uint16_t res  = (uint16_t)(_buf[base+6] | (_buf[base+7] << 8));

        int16_t xc = (int16_t)(x * kCos - y * kSin);
        int16_t yc = (int16_t)(x * kSin + y * kCos);

        // Compute dist and angle once here — all callers read the cached values.
        float dist  = sqrtf((float)xc*xc + (float)yc*yc);
        float angle = atan2f((float)xc, (float)yc) * 57.2958f;
        bool  valid = (xc != 0 || yc != 0) && dist >= RADAR_MIN_DIST_MM;

        _targets[i] = {xc, yc, spd, res, valid, dist, angle};
    }
    return true;
}

uint8_t LD2450::activeTargets() const {
    uint8_t n = 0;
    for (const auto& t : _targets) if (t.valid) n++;
    return n;
}

float LD2450::nearestDistance() const {
    float nearest = 1e9f;
    for (const auto& t : _targets)
        if (t.valid) nearest = min(nearest, t.distance_mm());
    return nearest < 1e9f ? nearest : 0.0f;
}
