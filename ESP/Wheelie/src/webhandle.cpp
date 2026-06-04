#include "config.h"
#include <WiFi.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ── HTML dashboard ────────────────────────────────────────────────────────────
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Wheelie Tuning</title>
    <meta name="viewport" content="width=device-width, initial-scale=1" charset="UTF-8">
    <style>
        body { font-family: Arial; text-align: center; background-color: #222; color: #fff; }
        .card { background-color: #333; padding: 15px; border-radius: 10px; flex: 1; min-width: 280px; }
        .card-row { display: flex; gap: 20px; justify-content: center; margin: 20px auto; max-width: 900px; flex-wrap: wrap; }
        button { padding: 10px 20px; font-size: 16px; margin: 5px; cursor: pointer; background: #007BFF; color: white; border: none; border-radius: 5px; }
        button:active { background: #0056b3; }
        button.profile-btn { background: #444; border: 2px solid #555; min-width: 120px; }
        button.profile-btn.active { background: #007BFF; border-color: #007BFF; }
        input { padding: 10px; font-size: 16px; width: 100px; text-align: center; margin-right: 10px; }
        .tune-row { margin: 10px 0; display: flex; justify-content: center; align-items: center; }
        #status { font-weight: bold; color: #FFA500; }
        .profile-row { display: flex; gap: 10px; justify-content: center; flex-wrap: wrap; margin: 10px 0; }
        .profile-info { font-size: 13px; color: #aaa; margin-top: 8px; }
    </style>
</head>
<body>
    <h2>Wheelie LQR Tuning</h2>
    <p>Status: <span id="status">Connecting...</span></p>

    <div class="card-row">

        <!-- ── Profile switcher ── -->
        <div class="card">
            <h3>Height Profile</h3>
            <div class="profile-row">
                <button class="profile-btn" id="pbtn0" onclick="switchProfile(0)">25° Squat</button>
                <button class="profile-btn" id="pbtn1" onclick="switchProfile(1)">45° Low</button>
                <button class="profile-btn" id="pbtn2" onclick="switchProfile(2)">65° Mid</button>
                <button class="profile-btn" id="pbtn3" onclick="switchProfile(3)">85° Stand</button>
            </div>
            <div class="profile-info" id="profileInfo">Active: —</div>
        </div>

        <!-- ── Offset trim ── -->
        <div class="card">
            <h3>Balance Offset</h3>
            <div class="profile-info">Fine-tune the balance point for the active profile</div>
            <div class="tune-row" style="margin-top:14px">
                <label>Offset (rad): </label>
                <input type="number" id="valO" step="0.01">
                <button onclick="setVal('O')">Set</button>
            </div>
            <div style="display:flex;gap:8px;justify-content:center;margin-top:8px">
                <button onclick="nudgeOffset(-0.01)">− 0.01</button>
                <button onclick="nudgeOffset(+0.01)">+ 0.01</button>
            </div>
            <div class="profile-info" id="offsetDisplay">offset = —</div>
        </div>

        <!-- ── Target state ── -->
        <div class="card">
            <h3>Target State</h3>
            <div class="tune-row"><label>Target Pos (m):   </label><input type="number" id="valTP" step="0.01" value="0"><button onclick="setVal('TP')">Set</button></div>
            <div class="tune-row"><label>Target Vel (m/s): </label><input type="number" id="valTV" step="0.01" value="0"><button onclick="setVal('TV')">Set</button></div>
            <div class="tune-row"><label>Target Pitch(rad):</label><input type="number" id="valTA" step="0.001" value="0"><button onclick="setVal('TA')">Set</button></div>
            <div class="tune-row"><label>Target Gyro(rad/s):</label><input type="number" id="valTG" step="0.001" value="0"><button onclick="setVal('TG')">Set</button></div>
        </div>

        <!-- ── K1–K4 gain tuning ── -->
        <div class="card">
            <h3>LQR Gains (K1–K4)</h3>
            <div class="tune-row"><label>K1 (pos):   </label><input type="number" id="valK1" step="0.1"><button onclick="setVal('K1')">Set</button></div>
            <div class="tune-row"><label>K2 (vel):   </label><input type="number" id="valK2" step="0.1"><button onclick="setVal('K2')">Set</button></div>
            <div class="tune-row"><label>K3 (pitch): </label><input type="number" id="valK3" step="0.1"><button onclick="setVal('K3')">Set</button></div>
            <div class="tune-row"><label>K4 (gyro):  </label><input type="number" id="valK4" step="0.1"><button onclick="setVal('K4')">Set</button></div>
        </div>

        <!-- ── Yaw control ── -->
        <div class="card">
            <h3>Yaw Control</h3>
            <div class="tune-row" style="gap:10px">
                <button id="yawToggleBtn" onclick="toggleYaw()">Yaw: ON</button>
                <button onclick="lockHeading()">Lock Heading</button>
            </div>
            <div class="tune-row" style="margin-top:10px">
                <label>Target Yaw (°): </label>
                <input type="number" id="valTY" step="5" value="0" style="width:80px">
                <button onclick="setYaw()">Set</button>
            </div>
            <div style="display:flex;gap:6px;justify-content:center;margin-top:8px">
                <button onclick="nudgeYaw(-15)">−15°</button>
                <button onclick="nudgeYaw(-5)">−5°</button>
                <button onclick="nudgeYaw(+5)">+5°</button>
                <button onclick="nudgeYaw(+15)">+15°</button>
            </div>
            <div class="profile-info" id="yawInfo" style="margin-top:8px">Heading hold active</div>
            <div class="profile-info">
                Heading: <span id="yawHeading">—</span>°
                &nbsp;|&nbsp; Ref: <span id="yawRef">0.0</span>°
                &nbsp;|&nbsp; Error: <span id="yawErrDeg">—</span>°
            </div>
        </div>

    </div>

    <!-- ── State Monitor ── -->
    <div style="max-width:700px;margin:0 auto 20px auto">
        <div class="card">
            <h3>State Monitor</h3>
            <table style="width:100%;border-collapse:collapse;font-size:14px">
                <thead>
                    <tr style="border-bottom:1px solid #555">
                        <th style="padding:6px;text-align:left;color:#aaa">State</th>
                        <th style="padding:6px;color:#4af">Current</th>
                        <th style="padding:6px;color:#fa4">Target</th>
                        <th style="padding:6px;color:#f66">Error</th>
                    </tr>
                </thead>
                <tbody>
                    <tr><td style="padding:5px;text-align:left">Position (m)</td><td id="sm_cur_pos">—</td><td id="sm_tgt_pos">—</td><td id="sm_x1">—</td></tr>
                    <tr style="background:#2a2a2a"><td style="padding:5px;text-align:left">Velocity (m/s)</td><td id="sm_cur_vel">—</td><td id="sm_tgt_vel">—</td><td id="sm_x2">—</td></tr>
                    <tr><td style="padding:5px;text-align:left">Pitch (rad)</td><td id="sm_cur_pitch">—</td><td id="sm_tgt_pitch">—</td><td id="sm_x3">—</td></tr>
                    <tr style="background:#2a2a2a"><td style="padding:5px;text-align:left">Gyro (rad/s)</td><td id="sm_cur_gyro">—</td><td id="sm_tgt_gyro">—</td><td id="sm_x4">—</td></tr>
                    <tr><td style="padding:5px;text-align:left">Yaw (rad)</td><td id="sm_yaw_rad">—</td><td id="sm_yaw_ref">0.000</td><td id="sm_yaw_e">—</td></tr>
                </tbody>
            </table>
            <div class="profile-info" style="margin-top:8px">Trajectory setpoint: <span id="sm_pos_sp">—</span> m</div>
        </div>
    </div>

    <button onclick="resetErrors()">Reset Position (X1)</button>

    <script>
        var gateway = 'ws://' + window.location.hostname + '/ws';
        var websocket;
        var activeProfile = 0;

        var profileLabels = ['25° Squat', '45° Low', '65° Mid', '85° Stand'];
        var profileInfo   = [
            'K3=52.07  K4=4.00  offset=−0.200',
            'K3=56.60  K4=4.00  offset=−0.140',
            'K3=59.79  K4=2.77  offset=−0.070',
            'K3=61.00  K4=4.00  offset= 0.000',
        ];
        // [K1, K2, K3, K4] per profile
        var profileGains = [
            [-8.0, -10.0, 52.07, 4.00],
            [-8.0, -12.0, 56.60, 4.00],
            [-10.0, -15.0, 59.79, 2.77],
            [-8.0, -13.0, 61.00, 4.00],
        ];

        function initWebSocket() {
            websocket = new WebSocket(gateway);
            websocket.onopen  = () => {
                document.getElementById('status').textContent = 'CONNECTED';
                document.getElementById('status').style.color = '#00FF00';
            };
            websocket.onclose = () => {
                document.getElementById('status').textContent = 'DISCONNECTED';
                document.getElementById('status').style.color = '#FF0000';
                setTimeout(initWebSocket, 2000);
            };
            websocket.onmessage = (event) => {
                var d = {};
                event.data.trim().split(' ').forEach(p => {
                    var i = p.indexOf(':');
                    if (i > 0) d[p.slice(0, i)] = parseFloat(p.slice(i + 1));
                });
                function upd(id, val, dec) {
                    var el = document.getElementById(id);
                    if (el && !isNaN(val)) el.textContent = val.toFixed(dec !== undefined ? dec : 3);
                }
                upd('sm_cur_pos',   d.cur_pos,   3);
                upd('sm_tgt_pos',   d.tgt_pos,   3);
                upd('sm_pos_sp',    d.pos_sp,    3);
                upd('sm_x1',        d.x1,        3);
                upd('sm_cur_vel',   d.cur_vel,   3);
                upd('sm_tgt_vel',   d.tgt_vel,   3);
                upd('sm_x2',        d.x2,        3);
                upd('sm_cur_pitch', d.cur_pitch, 4);
                upd('sm_tgt_pitch', d.tgt_pitch, 4);
                upd('sm_x3',        d.x3,        4);
                upd('sm_cur_gyro',  d.cur_gyro,  3);
                upd('sm_tgt_gyro',  d.tgt_gyro,  3);
                upd('sm_x4',        d.x4,        3);
                upd('sm_yaw_rad',   d.yaw_rad,   3);
                upd('sm_yaw_e',     d.yaw_e,     3);
                if (!isNaN(d.yaw_rad)) {
                    currentYawDeg = d.yaw_rad * 180 / Math.PI;
                    var hEl = document.getElementById('yawHeading');
                    if (hEl) hEl.textContent = currentYawDeg.toFixed(1);
                }
                if (!isNaN(d.yaw_e)) {
                    var eEl = document.getElementById('yawErrDeg');
                    if (eEl) eEl.textContent = (d.yaw_e * 180 / Math.PI).toFixed(1);
                }
            };
        }

        // ── Profile switching ──────────────────────────────────────────────────
        function switchProfile(idx) {
            fetch('/set?type=P&val=' + idx)
                .then(() => highlightProfile(idx))
                .catch(err => console.log('Profile switch failed:', err));
        }

        var profileOffsets = [-0.200, -0.140, -0.070, 0.000];
        var currentOffset  = profileOffsets[0];

        function highlightProfile(idx) {
            activeProfile  = idx;
            currentOffset  = profileOffsets[idx];
            for (var i = 0; i < profileLabels.length; i++) {
                document.getElementById('pbtn' + i).classList.toggle('active', i === idx);
            }
            document.getElementById('profileInfo').textContent =
                'Active: ' + profileLabels[idx] + '   ' + profileInfo[idx];
            document.getElementById('valO').value = currentOffset.toFixed(3);
            document.getElementById('offsetDisplay').textContent =
                'offset = ' + currentOffset.toFixed(3) + ' rad';
            // populate K1–K4 inputs
            var g = profileGains[idx];
            document.getElementById('valK1').value = g[0];
            document.getElementById('valK2').value = g[1];
            document.getElementById('valK3').value = g[2];
            document.getElementById('valK4').value = g[3];
        }

        // ── Offset control ─────────────────────────────────────────────────────
        function setVal(type) {
            var val = document.getElementById('val' + type).value;
            currentOffset = parseFloat(val);
            fetch('/set?type=' + type + '&val=' + val)
                .catch(err => console.log('Set failed:', err));
            document.getElementById('offsetDisplay').textContent =
                'offset = ' + parseFloat(val).toFixed(3) + ' rad';
        }

        function nudgeOffset(delta) {
            currentOffset = Math.round((currentOffset + delta) * 1000) / 1000;
            document.getElementById('valO').value = currentOffset.toFixed(3);
            document.getElementById('offsetDisplay').textContent =
                'offset = ' + currentOffset.toFixed(3) + ' rad';
            fetch('/set?type=O&val=' + currentOffset)
                .catch(err => console.log('Nudge failed:', err));
        }

        function resetErrors() {
            fetch('/reset').catch(err => console.log('Reset failed:', err));
        }

        // ── Yaw control ───────────────────────────────────────────────────────
        var yawEnabled  = true;
        var currentYawDeg = 0;
        var targetYawDeg  = 0;

        function syncYawRefDisplay() {
            document.getElementById('valTY').value = targetYawDeg.toFixed(1);
            document.getElementById('yawRef').textContent = targetYawDeg.toFixed(1);
            var refRad = targetYawDeg * Math.PI / 180;
            document.getElementById('sm_yaw_ref').textContent = refRad.toFixed(3);
        }

        function setYaw() {
            targetYawDeg = parseFloat(document.getElementById('valTY').value) || 0;
            syncYawRefDisplay();
            fetch('/set?type=TY&val=' + (targetYawDeg * Math.PI / 180))
                .catch(err => console.log('Set yaw failed:', err));
        }

        function nudgeYaw(delta) {
            targetYawDeg = Math.round((targetYawDeg + delta) * 10) / 10;
            syncYawRefDisplay();
            fetch('/set?type=TY&val=' + (targetYawDeg * Math.PI / 180))
                .catch(err => console.log('Nudge yaw failed:', err));
        }

        function toggleYaw() {
            yawEnabled = !yawEnabled;
            fetch('/set?type=YE&val=' + (yawEnabled ? 1 : 0))
                .catch(err => console.log('Yaw toggle failed:', err));
            document.getElementById('yawToggleBtn').textContent = 'Yaw: ' + (yawEnabled ? 'ON' : 'OFF');
            document.getElementById('yawInfo').textContent = yawEnabled ? 'Heading hold active' : 'Heading hold disabled';
        }

        function lockHeading() {
            targetYawDeg = currentYawDeg;   // snap JS state to current heading
            syncYawRefDisplay();
            fetch('/set?type=YR&val=0')     // firmware snaps yaw_ref = currentState.yaw_rad
                .catch(err => console.log('Lock heading failed:', err));
            document.getElementById('yawInfo').textContent = 'Heading locked to current';
        }

        window.onload = function () {
            initWebSocket();
            highlightProfile(0);   // show profile 0 active on load
        };
    </script>
</body>
</html>
)rawliteral";

// ── Web server setup ──────────────────────────────────────────────────────────
void setupWebServer() {
    Serial.println("Starting WiFi AP...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Wheelie_Robot", "12345678");
    Serial.print("AP IP: ");
    Serial.println(WiFi.softAPIP());

    server.addHandler(&ws);

    // Main page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", index_html);
    });

    // /set handler
    server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("type") && request->hasParam("val")) {
            String type = request->getParam("type")->value();
            float  val  = request->getParam("val")->value().toFloat();

            // ── Profile switch ─────────────────────────────────────────────
            if      (type == "P")  applyProfile((int)val);    // by index

            // ── Offset trim (only per-session tunable value) ───────────────
            else if (type == "O")  rad_offset = val;

            // ── K1–K4 live gain tuning ─────────────────────────────────────
            else if (type == "K1") K1 = val;
            else if (type == "K2") K2 = val;
            else if (type == "K3") K3 = val;
            else if (type == "K4") K4 = val;

            // ── Yaw control ────────────────────────────────────────────────
            else if (type == "YE") yaw_enabled = (bool)(int)val;
            else if (type == "YR") yaw_ref     = currentState.yaw_rad;  // lock current heading
            else if (type == "TY") yaw_ref     = val;                   // set absolute target yaw (rad)

            // ── Target state ───────────────────────────────────────────────
            else if (type == "TP") target.position    = val;
            else if (type == "TV") target.velocity    = val;
            else if (type == "TA") target.pitch_rad = val;
            else if (type == "TG") target.gyro_rate   = val;
        }
        request->send(200);
    });

    // /reset — return to origin; trajectory ramps smoothly from current pos_setpoint
    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
        target.position = 0.0f;
        target.velocity = 0.0f;
        position_offset = 0.0f;   // clear drift so x1 is clean during return trip
        yaw_ref = currentState.yaw_rad;
        request->send(200);
    });

    server.begin();
}