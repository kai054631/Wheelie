#include "config.h"
#include <WiFi.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

int active_profile = 0;

// ── Apply a profile by index ──────────────────────────────────────────────────
void applyProfile(int idx) {
    if (idx < 0 || idx >= N_PROFILES) return;
    active_profile = idx;
    const Profile &p = profile_list[idx];
    Servo_angle  = p.servo_angle;
    angle_offset = p.angle_offset;
    K1 = p.K1;  K2 = p.K2;
    K3 = p.K3;  K4 = p.K4;
    K5 = p.K5;  K6 = p.K6;
    // Snap position integrator and zero targets so x1 doesn't spike
    position_offset  = get_average_distance_meters();
    target.position  = 0.0f;
    target.velocity  = 0.0f;
    // Lock yaw reference to current heading so yaw control doesn't fight the transition
    yaw_ref = yaw_angle;
    Serial.printf("[profile] switched to %d — servo=%d° K3=%.2f K4=%.2f offset=%.3f\n",
                  idx, p.servo_angle, p.K3, p.K4, p.angle_offset);
}

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

        <!-- ── Yaw control ── -->
        <div class="card">
            <h3>Yaw Control</h3>
            <div class="tune-row" style="gap:10px">
                <button id="yawToggleBtn" onclick="toggleYaw()">Yaw: ON</button>
                <button onclick="lockHeading()">Lock Heading</button>
            </div>
            <div class="profile-info" id="yawInfo">Heading hold active</div>
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
        var yawEnabled = true;
        function toggleYaw() {
            yawEnabled = !yawEnabled;
            fetch('/set?type=YE&val=' + (yawEnabled ? 1 : 0))
                .catch(err => console.log('Yaw toggle failed:', err));
            document.getElementById('yawToggleBtn').textContent = 'Yaw: ' + (yawEnabled ? 'ON' : 'OFF');
            document.getElementById('yawInfo').textContent = yawEnabled ? 'Heading hold active' : 'Heading hold disabled';
        }
        function lockHeading() {
            fetch('/set?type=YR&val=0')   // firmware snaps yaw_ref = yaw_angle
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
            else if (type == "O")  angle_offset = val;

            // ── Yaw control ────────────────────────────────────────────────
            else if (type == "YE") yaw_enabled = (bool)(int)val;
            else if (type == "YR") yaw_ref     = yaw_angle;  // lock current heading

            // ── Target state ───────────────────────────────────────────────
            else if (type == "TP") target.position    = val;
            else if (type == "TV") target.velocity    = val;
            else if (type == "TA") target.pitch_angle = val;
            else if (type == "TG") target.gyro_rate   = val;
        }
        request->send(200);
    });

    // /reset — snap position reference and zero targets
    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
        position_offset = get_average_distance_meters();
        target.position = 0.0f;
        target.velocity = 0.0f;
        yaw_ref = yaw_angle;
        request->send(200);
    });

    server.begin();
}