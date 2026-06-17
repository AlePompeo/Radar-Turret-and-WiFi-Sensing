#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

#include "config.h"
#include "sensor_snapshot.h"
#include "radar/ld2450.h"
#include "imu/bno055_imu.h"
#include "hud/led_hud.h"
#include "buzzer/alert.h"
#include "wifi_sensing/csi_sensing.h"
#include "web/web_server.h"
#include "servo/servo_tracker.h"

static SemaphoreHandle_t gMutex;
static SensorSnapshot    gSnap;

static LD2450        radar;
static IMU           imuSensor;
static LedHud        hud;
static Buzzer        buzzer;
static CsiSensing    csiSensor;
static WebApp        webApp;
static ServoTracker  servoTracker;

static AlertLevel computeAlert(float nearestMm, bool wifiPresent) {
    if (nearestMm > 0 && nearestMm < DIST_NEAR)                 return AlertLevel::NEAR;
    if (nearestMm > 0 && nearestMm < DIST_MEDIUM)               return AlertLevel::MEDIUM;
    if ((nearestMm > 0 && nearestMm < DIST_FAR) || wifiPresent) return AlertLevel::FAR;
    return AlertLevel::NONE;
}

// Core 1, priority 8 — radar + IMU + HUD + buzzer
static void taskIO(void*) {
    TickType_t      lastWake    = xTaskGetTickCount();
    uint32_t        lastImuMs   = 0;
    HeadOrientation head{};
    bool            wifiPresent = false; // read at end of each loop, 10ms stale is fine

    for (;;) {
        // Radar read + servo update first — no blocking code in between
        radar.update();
        servoTracker.update(radar);

        // IMU changes slowly: 50ms cadence is plenty, saves I2C time every loop
        uint32_t now = millis();
        if (imuSensor.ready() && (now - lastImuMs) >= 50) {
            head = imuSensor.read();
            lastImuMs = now;
        }

        float      nearest = radar.nearestDistance();
        AlertLevel alert   = computeAlert(nearest, wifiPresent);
        buzzer.setLevel(alert);
        buzzer.update();
        hud.update(radar, head);

        // Single mutex take per loop: write snapshot + read wifiPresent for next iteration
        xSemaphoreTake(gMutex, portMAX_DELAY);
        for (uint8_t i = 0; i < LD2450::MAX_TARGETS; i++)
            gSnap.targets[i] = radar.target(i);
        gSnap.activeTargets  = radar.activeTargets();
        gSnap.nearestMm      = nearest;
        gSnap.head           = head;
        gSnap.alert          = alert;
        gSnap.servoAngleDeg  = servoTracker.currentAngleDeg();
        gSnap.servoMode      = servoTracker.mode();
        gSnap.servoState     = servoTracker.state();
        wifiPresent          = gSnap.humanPresent;
        xSemaphoreGive(gMutex);

        vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(10));
    }
}

// Core 1, priority 3 — CSI variance + RSSI scoring
static void taskSense(void*) {
    for (;;) {
        csiSensor.update();

        xSemaphoreTake(gMutex, portMAX_DELAY);
        gSnap.csiScore      = csiSensor.csiScore();
        gSnap.rssiScore     = csiSensor.rssiScore();
        gSnap.presenceScore = csiSensor.presenceScore();
        gSnap.humanPresent  = csiSensor.humanPresent();
        gSnap.lastRssi      = csiSensor.lastRssi();
        xSemaphoreGive(gMutex);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Core 0, priority 2 — WebSocket push
static void taskWeb(void*) {
    TickType_t lastPush = xTaskGetTickCount();
    for (;;) {
        if ((xTaskGetTickCount() - lastPush) >= pdMS_TO_TICKS(WEB_PUSH_MS)) {
            lastPush = xTaskGetTickCount();

            SensorSnapshot snap;
            xSemaphoreTake(gMutex, portMAX_DELAY);
            snap = gSnap;
            xSemaphoreGive(gMutex);

            webApp.pushUpdate(snap);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    Serial.begin(115200);

    Serial2.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
    radar.begin(Serial2);

    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    if (!imuSensor.begin()) Serial.println("[WARN] BNO055 not found");

    hud.begin();
    buzzer.begin();

    WiFi.setSleep(false);
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    Serial.print("[INFO] AP IP: ");
    Serial.println(WiFi.softAPIP());

    csiSensor.begin();
    servoTracker.begin();
    webApp.setServoTracker(&servoTracker);
    webApp.begin();

    gMutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(taskIO,    "taskIO",    8192, nullptr, 8, nullptr, 1);
    xTaskCreatePinnedToCore(taskSense, "taskSense", 4096, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(taskWeb,   "taskWeb",   8192, nullptr, 2, nullptr, 0);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
