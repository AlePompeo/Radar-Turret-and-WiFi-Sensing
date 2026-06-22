# Radar Turret

ESP32 platform for human presence detection in defence scenarios. Fuses a 24 GHz mmWave radar with WiFi Channel State Information (CSI) sensing to detect targets both in line-of-sight and through walls. A continuous-rotation servo turret auto-tracks the highest-threat target, a dual WS2812 LED strip provides a heads-up display and a passive buzzer gives audible range alerts. All sensor data streams in real time to a mobile-friendly web dashboard over WiFi.

---

## Table of contents

1. [Hardware](#hardware)
2. [System architecture](#system-architecture)
3. [Subsystems](#subsystems)
   - [Radar — LD2450](#radar--ld2450)
   - [WiFi CSI sensing](#wifi-csi-sensing)
   - [LED HUD](#led-hud)
   - [Buzzer alerts](#buzzer-alerts)
   - [Servo turret](#servo-turret)
   - [Web dashboard](#web-dashboard)
4. [FreeRTOS task model](#freertos-task-model)
5. [Build & flash](#build--flash)
6. [Network access](#network-access)
7. [Web API](#web-api)
8. [Configuration reference](#configuration-reference)

---

## Hardware

| Component | Part | GPIO | Interface |
|-----------|------|------|-----------|
| MCU | ESP32 DevKit | — | — |
| Radar | HLK-LD2450 24 GHz mmWave | RX 26 / TX 27 | UART2 @ 256000 baud |
| IMU | Adafruit BNO055 9-DOF | SDA 21 / SCL 22 | I2C @ 0x28 |
| LED strip left | WS2812 × 8 | 13 | FastLED (GRB) |
| LED strip right | WS2812 × 8 | 12 | FastLED (GRB) |
| Passive buzzer | — | 25 | LEDC channel 14 |
| Servo | SM-S430R continuous rotation | 33 | PWM 50 Hz via ESP32Servo |

> **LEDC channel** — the buzzer uses channel 14 to avoid collision with ESP32Servo, which claims channels 0–13.

---

## System architecture

```
┌─────────────────────────────────────────────────────────┐
│                        ESP32                            │
│                                                         │
│  Core 1                          Core 0                 │
│  ┌──────────────────────┐        ┌──────────────────┐   │
│  │ taskIO  (prio 8)     │        │ taskWeb (prio 2) │   │
│  │  LD2450 radar        │        │  WebSocket push  │   │
│  │  BNO055 IMU          │        │  150 ms interval │   │
│  │  WS2812 LED HUD      │        └────────┬─────────┘   │
│  │  Passive buzzer      │                 │             │
│  │  Servo tracker       │        ┌────────┴─────────┐   │
│  └──────────┬───────────┘        │ AsyncWebServer   │   │
│             │  mutex             │  port 80         │   │
│  ┌──────────┴───────────┐        └──────────────────┘   │
│  │ taskSense (prio 3)   │                               │
│  │  WiFi CSI variance   │                               │
│  │  RSSI variance       │                               │
│  └──────────────────────┘                               │
│             │  mutex                                    │
│      SensorSnapshot (shared struct)                     │
└─────────────────────────────────────────────────────────┘
         │                    │
   LD2450 radar          WiFi AP "Sensing"
   BNO055 I2C            → browser dashboard
   WS2812 strips
   Servo PWM
```

---

## Subsystems

### Radar — LD2450

The HLK-LD2450 is a 24 GHz FMCW radar that resolves up to **3 simultaneous targets**, providing X/Y position (mm), radial speed (cm/s) and measurement resolution.

**Initialisation sequence sent over UART2 at startup:**

| Command | Effect |
|---------|--------|
| Enter config mode | Unlocks register writes |
| Multi-target mode (`0x90`) | Enables tracking of up to 3 targets simultaneously |
| Disable zone filter (`0x07`, type 0) | Full 360° field of view, no exclusion zones |
| Exit config mode | Returns to data stream |

**Frame format** (30 bytes, 10 Hz):

```
AA FF 03 00 | [target0 × 8 bytes] [target1 × 8 bytes] [target2 × 8 bytes] | 55 CC
```

Each 8-byte target block contains X (2 B), Y (2 B), speed (2 B) and resolution (2 B), all in sign-magnitude encoding with asymmetric sign-bit polarity. The driver re-syncs automatically on frame misalignment by sliding the receive buffer by one byte.

**Coordinate system** (after azimuth rotation correction):

```
         0° (forward)
              │
  -90°────────┼────────+90°
  (left)      │       (right)
```

A configurable `RADAR_AZIMUTH_OFFSET_DEG` applies a 2D rotation matrix at parse time to compensate for physical mounting angle. Ghost echoes closer than `RADAR_MIN_DIST_MM` (700 mm default) are filtered as near-field artefacts.

**Threat score** used by the servo tracker:

```
score = 8000 / (distance_mm + 1) + max(0, -speed_cms) × 0.05
```

Higher score → closer or faster-approaching target.

---

### WiFi CSI sensing

Detects human presence **through walls** by analysing statistical fluctuations in the WiFi channel caused by body movements, without requiring any additional hardware beyond the ESP32's own radio.

Two independent signal paths are fused:

#### CSI variance

The ESP32 records raw Complex Channel State Information from every received 802.11 packet. For each packet the mean subcarrier amplitude power (`Σ(re² + im²) / n`) is added to a 40-sample circular window. Variance of that window is compared against a calibrated baseline.

- **Calibration**: first 5 s after boot (device must be still); baseline variance is stored.
- **Score**: `variance / (baseline × 2.5 + 0.01)`, clamped to `[0, 1]`.

The CSI callback runs in the WiFi driver task (Core 0) under a spinlock — it only pushes the amplitude sample into the ring buffer. Variance is computed in `taskSense` on Core 1 to avoid blocking the driver.

#### RSSI variance

Every 200 ms the RSSI of the first connected station is sampled into a 20-sample window. Variance against a calibrated baseline detects micro-movements that modulate signal strength.

- **Score**: `variance / (baseline × 3.0 + 0.5)`, clamped to `[0, 1]`.
- Calibration starts only after CSI is calibrated (both windows must be stable-empty first).

#### Fused presence decision

```
presence_score = max(csi_score, rssi_score)
human_present  = presence_score > 0.5
```

The maximum (not average) is used so that either channel alone is sufficient to trigger detection.

---

### LED HUD

Two WS2812 strips of 8 LEDs each are mounted on the left and right temples of the glasses. Each valid radar target lights one LED on the strip corresponding to its side (left / right), at a bin position that encodes azimuth angle.

**Angle-to-bin mapping:**

```
abs_angle °  →  LED index (0 = centre, 7 = outermost)
    0°       →  0
   45°       →  3–4
   90°       →  7   (clamp)
```

Formula: `bin = round(abs_angle / 90 × 7)`, clamped to 0–7.

**Colour coding by distance:**

| Distance | Colour |
|----------|--------|
| < 1 m (NEAR) | Red |
| 1 m – 3 m (MEDIUM) | Yellow |
| > 3 m (FAR / WiFi) | Green |

**Head compensation:** the BNO055 yaw angle is subtracted from each target's radar azimuth before mapping to a bin, so the HUD shows world-fixed directions regardless of where the user is looking.

---

### Buzzer alerts

The passive buzzer is driven by LEDC PWM (50% duty cycle, 8-bit). Alert level is recomputed every 10 ms in `taskIO` based on the nearest radar target distance and the WiFi presence flag.

| Level | Condition | Frequency | Blink period |
|-------|-----------|-----------|--------------|
| CRITICAL | Radar < 1 m | 2000 Hz | 100 ms |
| WARNING | Radar < 3 m | 1500 Hz | 350 ms |
| ALERT | Radar < 8 m **or** WiFi presence | 1000 Hz | 900 ms |
| NONE | No threat | — | silent |

Frequency is only reconfigured via `ledcSetup` when the level changes, not every tick, to avoid audio glitches.

---

### Servo turret

An SM-S430R continuous-rotation servo sweeps to keep the selected radar target centred in the field of view. The servo is controlled by PWM pulse width: `1500 µs` = stop, `< 1500` = one direction, `> 1500` = other direction.

**Tracking modes** (selectable from the web UI):

| Mode | Behaviour |
|------|-----------|
| `HIGHEST_THREAT` | Tracks the target with the highest threat score (default) |
| `NEAREST` | Tracks the closest valid target |
| `AVERAGE` | Points toward the centroid of all valid targets |

**Control loop** (runs every 10 ms in `taskIO`):

1. Compute target azimuth via the selected mode.
2. Apply **hysteresis deadband**: enter stop zone at ±8°, resume only when error exceeds ±14°. Prevents oscillation around centre.
3. Compute deviation: `deviation_us = KP × azimuth_deg` (KP = 4.0), constrained to asymmetric speed caps (`SERVO_MAX_DEV_POS` / `SERVO_MAX_DEV_NEG`) to compensate for the servo's physical CW/CCW speed difference.
4. Apply **slew rate limiter**: output pulse width can change by at most `SERVO_SLEW_US_PER_CALL` (5 µs) per call → smooth acceleration, no sudden direction flips.
5. Write `SERVO_STOP_US + SERVO_STOP_TRIM + currentDeviation` to the servo.

When no target is visible for `SERVO_IDLE_MS` (20 ms), the servo decelerates to stop via the slew limiter. `SERVO_STOP_TRIM` is a runtime-adjustable ±µs offset to calibrate the true neutral of the specific servo unit.

---

### Web dashboard

A single-page application served from flash (no SD card). Connects via WebSocket to receive live sensor data at 150 ms intervals. Dark monospace theme optimised for outdoor readability.

**Pages:**

**Radar** — polar canvas (300 × 280 px) showing:
- Range rings at 1 m, 2 m, 3 m, 4 m with labels
- Azimuth guide lines at ±30° and ±60°
- Coloured target dots (red/yellow/green by distance) with distance labels in cm
- Servo pointing line colour-coded by state: grey (IDLE), blue (TRACKING), orange (SEARCHING)
- Per-target readout: distance, angle, speed with approach/recede arrow
- Servo tracking mode selector buttons

**Presence** — WiFi through-wall sensing display:
- Glowing circle indicator (green = detected, dark = clear)
- Three progress bars: CSI score, RSSI score, fused presence score
- 60-sample scrolling history graph of fused presence score
- Live station RSSI in dBm

**Telemetry** — raw sensor values:
- IMU yaw / pitch / roll in degrees
- Current alert level string
- Nearest radar target distance in mm
- WiFi presence boolean

---

## FreeRTOS task model

| Task | Core | Priority | Stack | Period |
|------|------|----------|-------|--------|
| `taskIO` | 1 | 8 | 8192 B | 10 ms (`vTaskDelayUntil`) |
| `taskSense` | 1 | 3 | 4096 B | 50 ms |
| `taskWeb` | 0 | 2 | 8192 B | push every 150 ms |

All three tasks share a single `SensorSnapshot` struct protected by a FreeRTOS mutex. Each task holds the mutex only for the minimum time needed to copy data — no computation occurs under the lock.

The CSI receive callback runs in the WiFi driver task (Core 0) and uses a spinlock (`portMUX_TYPE`) rather than a FreeRTOS mutex, as it may execute in an ISR-adjacent context.

---

## Build & flash

Requires [PlatformIO](https://platformio.org/) (VS Code extension or CLI).

```bash
# Flash firmware
pio run -t upload

# Open serial monitor (115200 baud)
pio device monitor
```

Serial output on boot:

```
[INFO] AP IP: 192.168.4.1
[SERVO] attach GPIO33 stop=1500 min=1000 max=2000 -> OK
[SERVO] stopped (neutral 1500 us)
```

If the BNO055 is not detected, IMU fields will read `0.0` and the HUD will not apply head compensation. All other subsystems continue operating.

---

## Network access

The ESP32 boots as a WiFi soft-AP:

| Field | Value |
|-------|-------|
| SSID | `Sensing` |
| Password | `defence123` |
| AP IP | `192.168.4.1` |
| Dashboard | `http://192.168.4.1` |

Connect a phone or laptop to the `Sensing` network and open the dashboard URL. The WebSocket reconnects automatically if the connection drops.

---

## Web API

### WebSocket — `/ws`

Push-only (server → client), JSON, every 150 ms:

```json
{
  "radar": {
    "targets": [
      { "x": 450, "y": 1200, "speed": -35, "dist": 1282, "angle": 20.5, "valid": true },
      { "x": 0,   "y": 0,    "speed": 0,   "dist": 0,    "angle": 0.0,  "valid": false },
      { "x": 0,   "y": 0,    "speed": 0,   "dist": 0,    "angle": 0.0,  "valid": false }
    ],
    "active": 1,
    "nearest": 1282
  },
  "imu":   { "yaw": 12.3, "pitch": -2.1, "roll": 0.8 },
  "wifi":  { "present": false, "score": 0.12, "csi": 0.08, "rssi": 0.15, "rssi_ref": -62 },
  "servo": { "angle": 110.5, "mode": 0, "state": 1 },
  "alert": "MEDIUM"
}
```

Units: distances in mm, angles in degrees, speed in cm/s (negative = approaching). `servo.angle` is 0–180° with 90° = centre. `servo.mode`: 0 = HIGHEST_THREAT, 1 = NEAREST, 2 = AVERAGE. `servo.state`: 0 = IDLE, 1 = TRACKING, 2 = SEARCHING.

### REST endpoints

| Method | Path | Params | Effect |
|--------|------|--------|--------|
| GET | `/servo` | `mode=0\|1\|2` | Change servo tracking mode |
| GET | `/servo/trim` | `us=±N` (max ±100) | Adjust servo neutral pulse to stop drift |
| GET | `/servo/sweep` | — | Run diagnostic sweep (CW → stop → CCW → stop), output on Serial |

---

## Configuration reference

All parameters are in `config.h`. No recompilation required for trim and mode — these can be set live via the web API.

### Pins

| Define | Default | Description |
|--------|---------|-------------|
| `RADAR_RX_PIN` | 26 | LD2450 UART RX |
| `RADAR_TX_PIN` | 27 | LD2450 UART TX |
| `RADAR_BAUD` | 256000 | LD2450 baud rate |
| `IMU_SDA_PIN` | 21 | BNO055 I2C SDA |
| `IMU_SCL_PIN` | 22 | BNO055 I2C SCL |
| `LED_LEFT_PIN` | 13 | Left WS2812 strip data |
| `LED_RIGHT_PIN` | 12 | Right WS2812 strip data |
| `LED_COUNT` | 8 | LEDs per strip |
| `LED_BRIGHTNESS` | 40 | FastLED global brightness (0–255) |
| `BUZZER_PIN` | 25 | Passive buzzer GPIO |
| `BUZZER_LEDC_CH` | 14 | LEDC channel (must not overlap ESP32Servo) |
| `SERVO_PIN` | 33 | Servo PWM GPIO |

### Radar

| Define | Default | Description |
|--------|---------|-------------|
| `RADAR_AZIMUTH_OFFSET_DEG` | 12.0 | Mounting angle correction (°). Positive = sensor is rotated CW |
| `RADAR_MIN_DIST_MM` | 700 | Minimum valid distance; filters near-field ghost echoes |

### Detection thresholds

| Define | Default | Description |
|--------|---------|-------------|
| `DIST_NEAR` | 1000 mm | Below this → CRITICAL alert |
| `DIST_MEDIUM` | 3000 mm | Below this → WARNING alert |
| `DIST_FAR` | 8000 mm | Below this (or WiFi presence) → ALERT |

### Buzzer

| Define | Default | Description |
|--------|---------|-------------|
| `BUZZ_FREQ_NEAR` | 2000 Hz | CRITICAL tone |
| `BUZZ_FREQ_MED` | 1500 Hz | WARNING tone |
| `BUZZ_FREQ_FAR` | 1000 Hz | ALERT tone |

### Servo

| Define | Default | Description |
|--------|---------|-------------|
| `SERVO_STOP_US` | 1500 | Nominal neutral pulse (servo stops) |
| `SERVO_STOP_TRIM` | 0 | ±µs offset for this servo's true neutral. Increase if drifting CW, decrease if CCW |
| `SERVO_MIN_US` | 1000 | Full speed, direction A |
| `SERVO_MAX_US` | 2000 | Full speed, direction B |
| `SERVO_KP` | 4.0 | P-controller gain (µs per degree of azimuth error) |
| `SERVO_DEADBAND_DEG` | 8.0 | Stop rotating within ±8° of centre |
| `SERVO_REACTIVATE_DEG` | 14.0 | Resume rotating when error exceeds ±14° (hysteresis) |
| `SERVO_IDLE_MS` | 20 | ms with no targets before stopping |
| `SERVO_INVERT` | true | Invert rotation direction |
| `SERVO_SLEW_US_PER_CALL` | 5 | Max pulse width change per 10 ms call (smooth acceleration) |
| `SERVO_MAX_DEV_POS` | 110 µs | Speed cap, slow direction |
| `SERVO_MAX_DEV_NEG` | 30 µs | Speed cap, fast direction (tune to match physical speeds) |
| `SERVO_SEARCH_DEV_US` | 35 µs | Search sweep speed |
| `SERVO_SEARCH_SWEEP_MS` | 800 | Time per sweep direction (≈ 20° arc) |
| `SERVO_SEARCH_TIMEOUT_MS` | 2000 | Stop searching after 2 s with no reacquisition |

### Web

| Define | Default | Description |
|--------|---------|-------------|
| `WEB_PUSH_MS` | 150 | WebSocket push interval |
| `WIFI_SSID` | `"Sensing"` | Soft-AP network name |
| `WIFI_PASS` | `"defence123"` | Soft-AP password |
