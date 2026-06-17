#pragma once

// LD2450 radar – UART2
#define RADAR_RX_PIN     26
#define RADAR_TX_PIN     27
#define RADAR_BAUD       256000
// Mounting angle correction: if targets appear N° to the RIGHT when you're
// directly in front, set this to +N. Applies a 2D rotation to all targets.
#define RADAR_AZIMUTH_OFFSET_DEG  12.0f

// BNO055 IMU – I2C
#define IMU_SDA_PIN      21
#define IMU_SCL_PIN      22

// WS2812 LED strips (8 LEDs each)
#define LED_LEFT_PIN     13
#define LED_RIGHT_PIN    12
#define LED_COUNT        8
#define LED_BRIGHTNESS   40

// Passive buzzer
#define BUZZER_PIN       25
#define BUZZER_LEDC_CH   14   // fixed LEDC channel — must not overlap ESP32Servo channels (0-13)

// WiFi soft-AP
#define WIFI_SSID        "Sensing"
#define WIFI_PASS        "defence123"

// Minimum valid target distance — filters near-field ghost echoes after fast departures (mm)
#define RADAR_MIN_DIST_MM  700

// Detection distance thresholds (mm)
#define DIST_NEAR        1000   // < 1 m  → CRITICAL
#define DIST_MEDIUM      3000   // < 3 m  → WARNING
#define DIST_FAR         8000   // < 8 m  → ALERT

// Buzzer tone frequencies (Hz)
#define BUZZ_FREQ_NEAR   2000
#define BUZZ_FREQ_MED    1500
#define BUZZ_FREQ_FAR    1000

// Web update interval (ms)
#define WEB_PUSH_MS      150

// SM-S430R continuous rotation servo tracker
#define SERVO_PIN          33      // GPIO for servo signal
#define SERVO_STOP_US      1500   // nominal neutral pulse (servo stops)
#define SERVO_STOP_TRIM    0      // ±us offset to calibrate this servo's true neutral
                                  // if servo drifts CW at rest → increase (e.g. +20)
                                  // if servo drifts CCW at rest → decrease (e.g. -20)
#define SERVO_MIN_US       1000   // full speed direction A
#define SERVO_MAX_US       2000   // full speed direction B
// P-controller: us = SERVO_STOP_US + SERVO_KP * azimuth_deg
#define SERVO_KP               4.0f
#define SERVO_DEADBAND_DEG     8.0f   // stop rotating when within ±9° of center
#define SERVO_REACTIVATE_DEG   14.0f  // resume rotating when error exceeds ±12° (hysteresis)
#define SERVO_IDLE_MS          20    // ms with no targets before stopping
// Set to true if rotation direction is inverted on your mount
#define SERVO_INVERT         true
// Slew rate on servo OUTPUT command (µs per ~10ms call).
// Limits how fast the pulse width can change → smooth acceleration, no sudden direction flip.
#define SERVO_SLEW_US_PER_CALL 5
// Asymmetric speed caps to compensate physical CW/CCW speed difference of the servo.
// With SERVO_INVERT=true: NEG = left-to-right (physically faster), POS = right-to-left (slower).
// Reduce SERVO_MAX_DEV_NEG until both directions feel equally fast.
#define SERVO_MAX_DEV_POS      110   // slow direction cap (µs from neutral)
#define SERVO_MAX_DEV_NEG       30   // fast direction cap — increase if too slow L→R
// Search sweep: activates only when target is lost during active tracking.
// Sweeps ±20° (mapped to ±SERVO_SEARCH_DEV_US) until target is reacquired.
#define SERVO_SEARCH_DEV_US    35     // search speed (35/500 × 360 ≈ 25 deg/s)
#define SERVO_SEARCH_SWEEP_MS  800    // time per direction: 25 deg/s × 0.8s ≈ 20°
#define SERVO_SEARCH_TIMEOUT_MS 2000  // stop searching after 2 s
