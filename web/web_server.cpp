#include "web_server.h"
#include <Arduino.h>

static const char INDEX_HTML[] = R"rawhtml(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Smart Glasses</title>
<style>
body{background:#111;color:#0f0;font-family:monospace;margin:0;padding:10px}
h2{margin:0 0 10px}
nav{display:flex;flex-wrap:wrap;gap:6px;margin-bottom:12px}
nav button{background:#222;color:#0f0;border:1px solid #0f0;padding:6px 12px;cursor:pointer;font-family:monospace;font-size:12px}
nav button.active{background:#0f0;color:#000}
.page{display:none}.page.active{display:block}
canvas{border:1px solid #333;display:block;margin:0 auto}
.metric{margin:4px 0}.val{color:#fff}
#presenceCircle{width:70px;height:70px;border-radius:50%;background:#300;border:2px solid #0f0;margin:10px 0;transition:background .3s,box-shadow .3s}
#presenceCircle.on{background:#0f0;box-shadow:0 0 18px #0f0}
.bar-wrap{background:#222;height:8px;width:200px;border:1px solid #333;display:inline-block;vertical-align:middle;margin-left:6px}
.bar-fill{height:100%;background:#0f0;transition:width .2s}
.bar-fill.warn{background:#ff0}.bar-fill.crit{background:#f00}
</style>
</head>
<body>
<h2>Smart Glasses HUD</h2>
<nav>
  <button class="active" onclick="show('radar',this)">Radar</button>
  <button onclick="show('presence',this)">Presence</button>
  <button onclick="show('telem',this)">Telemetry</button>
</nav>

<!-- RADAR -->
<div id="radar" class="page active">
  <canvas id="rc" width="300" height="280"></canvas>
  <div id="rInfo" style="margin-top:6px;font-size:12px">No targets</div>
  <div style="margin-top:8px;font-size:12px">
    Servo:
    <button id="btnThreat" class="active" onclick="setMode(0)">Threat</button>
    <button id="btnNearest" onclick="setMode(1)">Nearest</button>
    <button id="btnAvg" onclick="setMode(2)">Average</button>
    &nbsp;<span id="servoAngle" style="color:#fff">90&deg;</span>
  </div>
</div>

<!-- PRESENCE -->
<div id="presence" class="page">
  <div id="presenceCircle"></div>
  <div id="pLabel" style="margin-bottom:10px">--</div>
  <div class="metric">CSI&nbsp;&nbsp;&nbsp;
    <div class="bar-wrap"><div id="csiBar" class="bar-fill" style="width:0%"></div></div>
    <span class="val" id="csiVal">--</span>
  </div>
  <div class="metric">RSSI&nbsp;&nbsp;
    <div class="bar-wrap"><div id="rssiBar" class="bar-fill" style="width:0%"></div></div>
    <span class="val" id="rssiVal">--</span>
  </div>
  <div class="metric">Fused
    <div class="bar-wrap"><div id="fusedBar" class="bar-fill" style="width:0%"></div></div>
    <span class="val" id="fusedVal">--</span>
  </div>
  <div class="metric" style="margin-top:8px">Station RSSI: <span class="val" id="rssiRef">--</span> dBm</div>
  <canvas id="wc" width="300" height="80" style="margin-top:10px"></canvas>
</div>

<!-- TELEMETRY -->
<div id="telem" class="page">
  <div class="metric">Yaw:     <span class="val" id="yaw">--</span>&deg;</div>
  <div class="metric">Pitch:   <span class="val" id="pitch">--</span>&deg;</div>
  <div class="metric">Roll:    <span class="val" id="roll">--</span>&deg;</div>
  <div class="metric">Alert:   <span class="val" id="alert">--</span></div>
  <div class="metric">Nearest: <span class="val" id="nearest">--</span></div>
  <div class="metric">Presence:<span class="val" id="wPresent">--</span></div>
</div>

<script>
const wifiHist = new Array(60).fill(0);
let _state = null, _dirty = false;

const ws = new WebSocket('ws://' + location.hostname + '/ws');
ws.onmessage = function(e) {
  try { _state = JSON.parse(e.data); _dirty = true; } catch(_) {}
};

(function raf() {
  requestAnimationFrame(raf);
  if (!_dirty || !_state) return;
  _dirty = false;
  const pid = document.querySelector('.page.active')?.id;
  if (pid === 'radar')    drawRadar(_state);
  if (pid === 'presence') drawPresence(_state);
  drawTelem(_state);
})();

// ── Radar ──────────────────────────────────────────────────────────────────
function drawRadar(d) {
  const c = document.getElementById('rc');
  const ctx = c.getContext('2d');
  const cx = 150, cy = 275, scale = 150 / 3000;

  ctx.fillStyle = '#111'; ctx.fillRect(0,0,300,280);

  ctx.strokeStyle = '#222'; ctx.lineWidth = 1;
  [-60,-30,0,30,60].forEach(deg => {
    const r = deg * Math.PI / 180;
    ctx.beginPath(); ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.sin(r)*275, cy - Math.cos(r)*275);
    ctx.stroke();
  });

  ctx.strokeStyle = '#333';
  [[1000,'1m'],[2000,'2m'],[3000,'3m'],[4000,'4m']].forEach(([mm, lbl]) => {
    const pr = mm * scale;
    ctx.beginPath(); ctx.arc(cx, cy, pr, Math.PI, 0, true); ctx.stroke();
    ctx.fillStyle = '#444'; ctx.font = '9px monospace';
    ctx.fillText(lbl, cx + 4, cy - pr + 12);
  });

  d.radar.targets.forEach(t => {
    if (!t.valid) return;
    const px = cx + t.x * scale;
    const py = cy - t.y * scale;
    if (px < 0 || px > 300 || py < 0 || py > 280) return;
    ctx.fillStyle = t.dist < 1000 ? '#f00' : t.dist < 3000 ? '#ff0' : '#0f0';
    ctx.beginPath(); ctx.arc(px, py, 7, 0, 2*Math.PI); ctx.fill();
    ctx.fillStyle = '#aaa'; ctx.font = '10px monospace';
    ctx.fillText(Math.round(t.dist/10)+'cm', px+9, py-3);
  });

  // Servo pointing line — colour by state: 0=IDLE(grey) 1=TRACKING(blue) 2=SEARCHING(orange)
  const sa = d.servo.angle;
  const sr = (sa - 90) * Math.PI / 180;
  const sColor = d.servo.state === 1 ? '#08f' : d.servo.state === 2 ? '#f80' : '#555';
  ctx.strokeStyle = sColor; ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(cx, cy);
  ctx.lineTo(cx + Math.sin(sr)*260, cy - Math.cos(sr)*260);
  ctx.stroke();
  ctx.fillStyle = sColor; ctx.font = '9px monospace';
  ctx.fillText('S', cx + Math.sin(sr)*268 - 4, cy - Math.cos(sr)*268 + 4);

  const stateLabel = ['IDLE','TRACK','SEARCH'][d.servo.state] || '?';
  document.getElementById('servoAngle').textContent = sa.toFixed(1) + '° [' + stateLabel + ']';
  const modes = ['btnThreat','btnNearest','btnAvg'];
  modes.forEach((id,i) => document.getElementById(id).className = (i === d.servo.mode ? 'active' : ''));

  let info = d.radar.active + ' target(s) | nearest: ' +
    (d.radar.nearest > 0 ? Math.round(d.radar.nearest/10)+'cm' : '--') + '<br>';
  d.radar.targets.forEach((t,i) => {
    if (!t.valid) return;
    const dir = t.speed < 0 ? '&#x25BC;' : t.speed > 0 ? '&#x25B2;' : '-';
    info += 'T'+(i+1)+': '+Math.round(t.dist/10)+'cm | '+t.angle.toFixed(1)+'&deg; | '+dir+' '+Math.abs(t.speed)+'cm/s<br>';
  });
  document.getElementById('rInfo').innerHTML = info;
}

// ── Presence ───────────────────────────────────────────────────────────────
function drawPresence(d) {
  const w = d.wifi;
  wifiHist.push(w.score); wifiHist.shift();

  document.getElementById('presenceCircle').className = w.present ? 'on' : '';
  document.getElementById('pLabel').textContent = w.present ? 'PRESENCE DETECTED' : 'Clear';

  function setBar(barId, valId, v) {
    const pct = Math.round((v || 0) * 100);
    const el  = document.getElementById(barId);
    el.style.width  = pct + '%';
    el.className    = 'bar-fill' + (pct > 80 ? ' crit' : pct > 50 ? ' warn' : '');
    document.getElementById(valId).textContent = pct + '%';
  }
  setBar('csiBar',   'csiVal',   w.csi);
  setBar('rssiBar',  'rssiVal',  w.rssi);
  setBar('fusedBar', 'fusedVal', w.score);
  document.getElementById('rssiRef').textContent =
    (w.rssi_ref != null && w.rssi_ref > -100) ? w.rssi_ref : '--';

  const ctx = document.getElementById('wc').getContext('2d');
  ctx.fillStyle = '#111'; ctx.fillRect(0,0,300,80);
  ctx.strokeStyle = '#0f0'; ctx.lineWidth = 1.5;
  ctx.beginPath();
  wifiHist.forEach((v,i) => {
    const x = i*5, y = 80 - v*78;
    i === 0 ? ctx.moveTo(x,y) : ctx.lineTo(x,y);
  });
  ctx.stroke();
}

// ── Telemetry ──────────────────────────────────────────────────────────────
function drawTelem(d) {
  document.getElementById('yaw').textContent    = d.imu.yaw.toFixed(1);
  document.getElementById('pitch').textContent  = d.imu.pitch.toFixed(1);
  document.getElementById('roll').textContent   = d.imu.roll.toFixed(1);
  document.getElementById('alert').textContent  = d.alert;
  document.getElementById('nearest').textContent= d.radar.nearest > 0
    ? Math.round(d.radar.nearest) + ' mm' : '--';
  document.getElementById('wPresent').textContent = d.wifi.present ? 'DETECTED' : 'Clear';
}

// ── Servo mode ────────────────────────────────────────────────────────────
function setMode(m) { fetch('/servo?mode='+m); }

// ── Navigation ─────────────────────────────────────────────────────────────
function show(id, btn) {
  document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
  document.querySelectorAll('nav button').forEach(b => b.classList.remove('active'));
  document.getElementById(id).classList.add('active');
  btn.classList.add('active');
}
</script>
</body>
</html>
)rawhtml";

static const char* alertLabel(AlertLevel a) {
    switch (a) {
        case AlertLevel::NEAR:   return "NEAR";
        case AlertLevel::MEDIUM: return "MEDIUM";
        case AlertLevel::FAR:    return "FAR";
        default:                 return "NONE";
    }
}

void WebApp::begin() {
    _ws.onEvent([](AsyncWebSocket*, AsyncWebSocketClient*, AwsEventType type,
                   void*, uint8_t*, size_t) { (void)type; });
    _server.addHandler(&_ws);

    _server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send(200, "text/html; charset=utf-8", INDEX_HTML);
    });

    // /servo?mode=0|1|2  — change servo tracking mode
    _server.on("/servo", HTTP_GET, [this](AsyncWebServerRequest* req) {
        if (_servo && req->hasParam("mode")) {
            int m = req->getParam("mode")->value().toInt();
            if (m >= 0 && m <= 2)
                _servo->setMode(static_cast<ServoTrackMode>(m));
        }
        req->send(200, "text/plain", "ok");
    });

    // /servo/trim?us=±N — adjust neutral pulse to stop servo drift (e.g. ?us=20)
    _server.on("/servo/trim", HTTP_GET, [this](AsyncWebServerRequest* req) {
        if (_servo && req->hasParam("us")) {
            int trim = req->getParam("us")->value().toInt();
            trim = constrain(trim, -100, 100);
            _servo->setStopTrim(trim);
            Serial.printf("[SERVO] trim set to %+d us (neutral = %d)\n",
                          trim, SERVO_STOP_US + trim);
        }
        req->send(200, "text/plain", "ok");
    });

    // /servo/sweep — hardware test: moves 0→90→180→90, check Serial for output
    _server.on("/servo/sweep", HTTP_GET, [this](AsyncWebServerRequest* req) {
        req->send(200, "text/plain", "sweep started, check Serial monitor");
        if (_servo) {
            // Run sweep on a temporary task so we don't block the web handler
            xTaskCreate([](void* arg) {
                static_cast<ServoTracker*>(arg)->sweep();
                vTaskDelete(nullptr);
            }, "svSweep", 2048, _servo, 5, nullptr);
        }
    });

    _server.begin();
}

void WebApp::pushUpdate(const SensorSnapshot& snap) {
    _ws.cleanupClients();
    if (_ws.count() == 0) return;
    char buf[600];
    int n = buildJson(snap, buf, sizeof(buf));
    _ws.textAll(buf, (size_t)n);
}

int WebApp::buildJson(const SensorSnapshot& snap, char* buf, size_t size) const {
    int n = snprintf(buf, size, "{\"radar\":{\"targets\":[");
    for (uint8_t i = 0; i < LD2450::MAX_TARGETS; i++) {
        const RadarTarget& t = snap.targets[i];
        if (i) buf[n++] = ',';
        n += snprintf(buf+n, size-n,
            "{\"x\":%d,\"y\":%d,\"speed\":%d,\"dist\":%d,\"angle\":%.1f,\"valid\":%s}",
            t.x_mm, t.y_mm, t.speed_cms, (int)t.cachedDist,
            t.cachedAngle, t.valid ? "true" : "false");
    }
    n += snprintf(buf+n, size-n,
        "],\"active\":%u,\"nearest\":%d}"
        ",\"imu\":{\"yaw\":%.1f,\"pitch\":%.1f,\"roll\":%.1f}"
        ",\"wifi\":{\"present\":%s,\"score\":%.2f,\"csi\":%.2f,\"rssi\":%.2f,\"rssi_ref\":%d}"
        ",\"servo\":{\"angle\":%.1f,\"mode\":%u,\"state\":%u}"
        ",\"alert\":\"%s\"}",
        snap.activeTargets, (int)snap.nearestMm,
        snap.head.yaw, snap.head.pitch, snap.head.roll,
        snap.humanPresent ? "true" : "false",
        snap.presenceScore, snap.csiScore, snap.rssiScore, (int)snap.lastRssi,
        snap.servoAngleDeg, (unsigned)snap.servoMode, (unsigned)snap.servoState,
        alertLabel(snap.alert));
    return n;
}
