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
                <button class="profile-btn" id="pbtn0" onclick="switchProfile(0)">25&deg; Squat</button>
                <button class="profile-btn" id="pbtn1" onclick="switchProfile(1)">45&deg; Low</button>
                <button class="profile-btn" id="pbtn2" onclick="switchProfile(2)">65&deg; Mid</button>
                <button class="profile-btn" id="pbtn3" onclick="switchProfile(3)">85&deg; Stand</button>
            </div>
            <div class="profile-info" id="profileInfo">Active: &mdash;</div>
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
                <button onclick="nudgeOffset(-0.01)">&minus; 0.01</button>
                <button onclick="nudgeOffset(+0.01)">+ 0.01</button>
            </div>
            <div class="profile-info" id="offsetDisplay">offset = &mdash;</div>
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
            <h3>LQR Gains (K1&ndash;K4)</h3>
            <div class="tune-row"><label>K1 (pos):   </label><input type="number" id="valK1" step="0.1"><button onclick="setVal('K1')">Set</button></div>
            <div class="tune-row"><label>K2 (vel):   </label><input type="number" id="valK2" step="0.1"><button onclick="setVal('K2')">Set</button></div>
            <div class="tune-row"><label>K3 (pitch): </label><input type="number" id="valK3" step="0.1"><button onclick="setVal('K3')">Set</button></div>
            <div class="tune-row"><label>K4 (gyro):  </label><input type="number" id="valK4" step="0.1"><button onclick="setVal('K4')">Set</button></div>
        </div>

        <!-- ── Yaw control (radians) ── -->
        <div class="card">
            <h3>Yaw Control (rad)</h3>
            <div class="tune-row" style="gap:10px">
                <button id="yawToggleBtn" onclick="toggleYaw()">Yaw: ON</button>
                <button onclick="lockHeading()">Lock Heading</button>
            </div>
            <div class="tune-row" style="margin-top:10px">
                <label>Target Yaw (rad): </label>
                <input type="number" id="valTY" step="0.05" value="0" style="width:90px">
                <button onclick="setYaw()">Set</button>
            </div>
            <div style="display:flex;gap:6px;justify-content:center;margin-top:8px">
                <button onclick="nudgeYaw(-0.26)">&minus;0.26</button>
                <button onclick="nudgeYaw(-0.05)">&minus;0.05</button>
                <button onclick="nudgeYaw(+0.05)">+0.05</button>
                <button onclick="nudgeYaw(+0.26)">+0.26</button>
            </div>
            <!-- yaw gain tuning -->
            <div class="tune-row" style="margin-top:12px"><label>K5 (yaw angle): </label><input type="number" id="valK5" step="0.05"><button onclick="setVal('K5')">Set</button></div>
            <div class="tune-row"><label>K6 (yaw rate):  </label><input type="number" id="valK6" step="0.01"><button onclick="setVal('K6')">Set</button></div>
            <div class="profile-info" id="yawInfo" style="margin-top:8px">Heading hold active</div>
            <div class="profile-info">
                Heading: <span id="yawHeading">&mdash;</span> rad
                &nbsp;|&nbsp; Ref: <span id="yawRef">0.000</span> rad
                &nbsp;|&nbsp; Error: <span id="yawErrRad">&mdash;</span> rad
            </div>
        </div>

    </div>

    <!-- ── State Monitor ── -->
    <div style="max-width:720px;margin:0 auto 20px auto">
        <div class="card">
            <h3>State Monitor</h3>
            <table style="width:100%;border-collapse:collapse;font-size:14px">
                <thead>
                    <tr style="border-bottom:1px solid #555">
                        <th style="padding:6px;text-align:left;color:#aaa">State</th>
                        <th style="padding:6px;color:#4af">Current</th>
                        <th style="padding:6px;color:#fa4">Reference</th>
                        <th style="padding:6px;color:#f66">Error (Ref&minus;Cur)</th>
                    </tr>
                </thead>
                <tbody>
                    <tr><td style="padding:5px;text-align:left">Position (m)</td><td id="sm_cur_pos">&mdash;</td><td id="sm_ref_pos">&mdash;</td><td id="sm_e1">&mdash;</td></tr>
                    <tr style="background:#2a2a2a"><td style="padding:5px;text-align:left">Velocity (m/s)</td><td id="sm_cur_vel">&mdash;</td><td id="sm_ref_vel">&mdash;</td><td id="sm_e2">&mdash;</td></tr>
                    <tr><td style="padding:5px;text-align:left">Pitch (rad)</td><td id="sm_cur_pitch">&mdash;</td><td id="sm_ref_pitch">&mdash;</td><td id="sm_e3">&mdash;</td></tr>
                    <tr style="background:#2a2a2a"><td style="padding:5px;text-align:left">Gyro (rad/s)</td><td id="sm_cur_gyro">&mdash;</td><td id="sm_ref_gyro">&mdash;</td><td id="sm_e4">&mdash;</td></tr>
                    <tr><td style="padding:5px;text-align:left">Yaw (rad)</td><td id="sm_cur_yaw">&mdash;</td><td id="sm_ref_yaw">0.000</td><td id="sm_eyaw">&mdash;</td></tr>
                </tbody>
            </table>
            <div class="profile-info" style="margin-top:8px">
                Final target pos: <span id="sm_tgt_pos">&mdash;</span> m
                &nbsp;|&nbsp; ramp setpoint: <span id="sm_pos_sp">&mdash;</span> m
            </div>
            <div class="profile-info">
                Note: the LQR internally uses x = Current &minus; Reference = &minus;(Error shown).
            </div>
        </div>
    </div>

    <button onclick="resetErrors()">Reset Position / Heading</button>

    <script>
        var gateway = 'ws://' + window.location.hostname + '/ws';
        var websocket;
        var activeProfile = -1;     // force first telemetry to populate inputs
        var profileLabels = ['25\u00B0 Squat', '45\u00B0 Low', '65\u00B0 Mid', '85\u00B0 Stand'];

        // Only overwrite an input if the user is not currently editing it.
        function setIfIdle(id, val, dec) {
            var el = document.getElementById(id);
            if (el && document.activeElement !== el && !isNaN(val))
                el.value = (dec !== undefined) ? val.toFixed(dec) : val;
        }
        function upd(id, val, dec) {
            var el = document.getElementById(id);
            if (el && !isNaN(val)) el.textContent = val.toFixed(dec !== undefined ? dec : 3);
        }

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

                // ── State monitor: Reference is the live tracked setpoint, ──────
                //    Error = Reference - Current  (so it always equals tgt - cur).
                //    Firmware sends x1..x4 = Current - Reference, so Error = -x.
                if (!isNaN(d.cur_pos) && !isNaN(d.x1)) {
                    upd('sm_cur_pos', d.cur_pos, 3);
                    upd('sm_ref_pos', d.cur_pos - d.x1, 3);   // = pos_setpoint + position_offset
                    upd('sm_e1', -d.x1, 3);
                }
                if (!isNaN(d.cur_vel) && !isNaN(d.x2)) {
                    upd('sm_cur_vel', d.cur_vel, 3);
                    upd('sm_ref_vel', d.vel_ff, 3);
                    upd('sm_e2', -d.x2, 3);
                }
                if (!isNaN(d.cur_pitch) && !isNaN(d.x3)) {
                    upd('sm_cur_pitch', d.cur_pitch, 4);
                    upd('sm_ref_pitch', d.tgt_pitch, 4);
                    upd('sm_e3', -d.x3, 4);
                }
                if (!isNaN(d.cur_gyro) && !isNaN(d.x4)) {
                    upd('sm_cur_gyro', d.cur_gyro, 3);
                    upd('sm_ref_gyro', d.tgt_gyro, 3);
                    upd('sm_e4', -d.x4, 3);
                }
                if (!isNaN(d.yaw_rad) && !isNaN(d.yaw_e)) {
                    upd('sm_cur_yaw', d.yaw_rad, 3);
                    upd('sm_ref_yaw', d.yaw_ref, 3);
                    upd('sm_eyaw', -d.yaw_e, 3);
                    upd('yawHeading', d.yaw_rad, 3);
                    upd('yawRef', d.yaw_ref, 3);
                    upd('yawErrRad', -d.yaw_e, 3);
                }
                upd('sm_tgt_pos', d.tgt_pos, 3);
                upd('sm_pos_sp',  d.pos_sp,  3);

                // ── Keep gain/offset inputs in sync with firmware (no hardcoding) ─
                if (!isNaN(d.profile) && d.profile !== activeProfile) {
                    activeProfile = d.profile;
                    highlightProfile(activeProfile);
                    // fill all editable fields from the firmware's live values
                    setIfIdle('valK1', d.K1, 2); setIfIdle('valK2', d.K2, 2);
                    setIfIdle('valK3', d.K3, 2); setIfIdle('valK4', d.K4, 2);
                    setIfIdle('valK5', d.K5, 3); setIfIdle('valK6', d.K6, 3);
                    setIfIdle('valO',  d.offset, 3);
                    upd('offsetDisplay', d.offset, 3);
                }
            };
        }

        // ── Profile switching ──────────────────────────────────────────────────
        function switchProfile(idx) {
            fetch('/set?type=P&val=' + idx).catch(err => console.log('Profile switch failed:', err));
            // input fields refresh automatically from the next telemetry frame
        }
        function highlightProfile(idx) {
            for (var i = 0; i < profileLabels.length; i++)
                document.getElementById('pbtn' + i).classList.toggle('active', i === idx);
            document.getElementById('profileInfo').textContent = 'Active: ' + profileLabels[idx];
        }

        // ── Generic set / offset ───────────────────────────────────────────────
        function setVal(type) {
            var val = document.getElementById('val' + type).value;
            fetch('/set?type=' + type + '&val=' + val).catch(err => console.log('Set failed:', err));
            if (type === 'O')
                document.getElementById('offsetDisplay').textContent = 'offset = ' + parseFloat(val).toFixed(3) + ' rad';
        }
        function nudgeOffset(delta) {
            var cur = parseFloat(document.getElementById('valO').value) || 0;
            cur = Math.round((cur + delta) * 1000) / 1000;
            document.getElementById('valO').value = cur.toFixed(3);
            document.getElementById('offsetDisplay').textContent = 'offset = ' + cur.toFixed(3) + ' rad';
            fetch('/set?type=O&val=' + cur).catch(err => console.log('Nudge failed:', err));
        }
        function resetErrors() {
            fetch('/reset').catch(err => console.log('Reset failed:', err));
        }

        // ── Yaw control (all values in radians) ─────────────────────────────────
        var yawEnabled = true;
        function setYaw() {
            var rad = parseFloat(document.getElementById('valTY').value) || 0;
            fetch('/set?type=TY&val=' + rad).catch(err => console.log('Set yaw failed:', err));
        }
        function nudgeYaw(deltaRad) {
            var cur = parseFloat(document.getElementById('valTY').value) || 0;
            cur = Math.round((cur + deltaRad) * 1000) / 1000;
            document.getElementById('valTY').value = cur.toFixed(3);
            fetch('/set?type=TY&val=' + cur).catch(err => console.log('Nudge yaw failed:', err));
        }
        function toggleYaw() {
            yawEnabled = !yawEnabled;
            fetch('/set?type=YE&val=' + (yawEnabled ? 1 : 0)).catch(err => console.log('Yaw toggle failed:', err));
            document.getElementById('yawToggleBtn').textContent = 'Yaw: ' + (yawEnabled ? 'ON' : 'OFF');
            document.getElementById('yawInfo').textContent = yawEnabled ? 'Heading hold active' : 'Heading hold disabled';
        }
        function lockHeading() {
            fetch('/set?type=YR&val=0').catch(err => console.log('Lock heading failed:', err)); // firmware snaps yaw_ref = current
            document.getElementById('yawInfo').textContent = 'Heading locked to current';
        }

        window.onload = initWebSocket;
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

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/html", index_html);
    });

    // /set handler
    server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (request->hasParam("type") && request->hasParam("val")) {
            String type = request->getParam("type")->value();
            float  val  = request->getParam("val")->value().toFloat();

            if      (type == "P")  applyProfile((int)val);    // profile by index
            else if (type == "O")  rad_offset = val;          // balance offset (rad)

            // ── LQR balancing gains ───────────────────────────────────────
            else if (type == "K1") K1 = val;
            else if (type == "K2") K2 = val;
            else if (type == "K3") K3 = val;
            else if (type == "K4") K4 = val;

            // ── Yaw gains (K5 = yaw angle, K6 = yaw rate) ──────────────────
            else if (type == "K5") K5 = val;
            else if (type == "K6") K6 = val;

            // ── Yaw control (radians) ──────────────────────────────────────
            else if (type == "YE") yaw_enabled = (bool)(int)val;
            else if (type == "YR") yaw_ref     = currentState.yaw_rad;  // lock current heading
            else if (type == "TY") yaw_ref     = val;                   // absolute heading target (rad)

            // ── Target state ───────────────────────────────────────────────
            else if (type == "TP") target.position  = val;
            else if (type == "TV") target.velocity  = val;
            else if (type == "TA") target.pitch_rad = val;
            else if (type == "TG") target.gyro_rate = val;
        }
        request->send(200);
    });

    // /reset — return to origin and re-zero heading
    server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request) {
        target.position = 0.0f;
        target.velocity = 0.0f;
        position_offset = 0.0f;
        yaw_ref = currentState.yaw_rad;
        request->send(200);
    });

    server.begin();
}
