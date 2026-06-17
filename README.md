# Radar Turret & WiFi Sensing Glasses

ESP32-based smart glasses for human presence detection. Combines a 24 GHz mmWave radar, WiFi CSI through-wall sensing, IMU head compensation, a servo turret, LED HUD and a real-time web dashboard.

---

## Hardware

| Component | Part | Interface |
|-----------|------|-----------|
| MCU | ESP32 (DevKit) | — |
| Radar | HLK-LD2450 24 GHz | UART2 (GPIO 26/27) |
| IMU | BNO055 9-DOF | I2C (GPIO 21/22) |
| LED HUD | WS2812 × 2 strips (8 LED each) | GPIO 13, 12 |
| Buzzer | Passive buzzer | GPIO 25 (LEDC ch 14) |
| Servo | SM-S430R continuous rotation | GPIO 33 |

---

## Features

**Radar detection (LD2450)**
Tracks up to 3 targets simultaneously up to 8 m. Computes distance, azimuth angle and approach speed per target. Azimuth offset configurable for mounting angle correction.

**WiFi CSI through-wall sensing**
Analyses Channel State Information variance and RSSI fluctuation from nearby 2.4 GHz traffic to detect presence behind walls without line of sight.

**Alert levels**

| Level | Condition | Buzzer |
|-------|-----------|--------|
| CRITICAL | Radar < 1 m | 2000 Hz |
| WARNING | Radar < 3 m | 1500 Hz |
| ALERT | Radar < 8 m or WiFi presence | 1000 Hz |

**LED HUD**
Left and right WS2812 strips indicate presence and closest target azimuth, compensated for head orientation via BNO055.

**Servo turret**
Continuous-rotation servo tracks the nearest / most threatening target using a P-controller with deadband hysteresis and slew-rate limiting. Configurable tracking modes: nearest, highest threat, average.

**Web dashboard**
AsyncWebServer on port 80 with WebSocket push at 150 ms. Shows live radar map, WiFi presence scores and full sensor telemetry.

---

## Firmware architecture

Three FreeRTOS tasks share a mutex-protected `SensorSnapshot` struct:

```
Core 1 — taskIO    (priority 8)   radar + IMU + HUD + buzzer + servo  [10 ms loop]
Core 1 — taskSense (priority 3)   WiFi CSI + RSSI scoring             [50 ms loop]
Core 0 — taskWeb   (priority 2)   WebSocket push                      [150 ms push]
```

---

## Build & flash

Requires [PlatformIO](https://platformio.org/).

```bash
pio run -t upload
pio device monitor
```

The ESP32 boots as a WiFi soft-AP:

| Field | Value |
|-------|-------|
| SSID | `Sensing` |
| Password | `defence123` |
| Dashboard | `http://192.168.4.1` |

---

## Configuration

All hardware pins, thresholds and tuning parameters are in `config.h`.

Key values:

```c
RADAR_AZIMUTH_OFFSET_DEG   // mounting angle correction (degrees)
DIST_NEAR / MEDIUM / FAR   // detection thresholds (mm)
SERVO_KP                   // P-controller gain
SERVO_STOP_TRIM            // neutral pulse calibration (µs)
```
