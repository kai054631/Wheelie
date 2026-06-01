/**
 * servo_calibration.cpp  —  Wheelie Servo Trim Finder
 * =====================================================
 * Flash this as a STANDALONE SKETCH (replace main.cpp temporarily).
 * It disables all motor / balance code — only servos + WiFi run.
 *
 * HOW TO USE:
 *   1. Flash this sketch.
 *   2. Connect to WiFi AP "Wheelie_Cal" password "12345678".
 *   3. Open http://192.168.4.1 in a browser.
 *   4. Use the sliders to drive Left and Right servos independently.
 *   5. At each target angle (25, 45, 65, 85°), adjust Right until
 *      both leg heights match a ruler measurement.
 *   6. Read off the "Right Trim" value shown on the page — that is
 *      your SERVO_R_TRIM for that angle.
 *   7. If the trim is the same across all angles → one global constant.
 *      If it varies → build a per-angle trim table.
 *
 * After calibration, add to main.cpp:
 *
 *   #define SERVO_R_TRIM  <value>   // degrees to add to right servo
 *
 *   // In TaskServoCode:
 *   LeftServo.write(constrain(Servo_angle, 20, 100));
 *   RightServo.write(constrain(Servo_angle + SERVO_R_TRIM, 20, 100));
 *
 * PIN / HARDWARE: uses the same pins as production firmware (GPIO 4 & 5).
 * No motors, encoders, IMU, or FOC are initialised here.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

// ── Servo pins (match config.h) ──────────────────────────────────────────────
#define SERVO_L_PIN 4
#define SERVO_R_PIN 5

// ── Servo pulse ranges (match MyServo::setup() calls in main.cpp) ─────────────
// Left  : setup(pin, 2000, 1000)  → min_us=2000, max_us=1000
// Right : setup(pin, 1000, 2000)  → min_us=1000, max_us=2000
#define SERVO_L_MIN_US 2000
#define SERVO_L_MAX_US 1000
#define SERVO_R_MIN_US 1000
#define SERVO_R_MAX_US 2000

// ── State ─────────────────────────────────────────────────────────────────────
volatile int  target_angle = 25;   // "both servos go here" command
volatile int  right_trim   = 0;    // degrees added to right servo only
volatile int  left_angle_override  = -1;  // -1 = follow target_angle
volatile int  right_angle_override = -1;  // -1 = follow target_angle + trim

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ── Servo write helper (replicates MyServo logic without the class) ───────────
static void writeServo(int pin, int min_us, int max_us, int angle) {
    angle = constrain(angle, 0, 180);
    int us    = map(angle, 0, 180, min_us, max_us);
    int pulse = (us * 4095) / 20000;
    ledcWrite(pin, pulse);
}

// ── Web page ──────────────────────────────────────────────────────────────────
const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Wheelie Servo Cal</title>
<style>
  :root {
    --bg:     #0f1117;
    --panel:  #1a1d27;
    --border: #2e3245;
    --accent: #5b8dee;
    --warn:   #e8a24b;
    --green:  #4be89a;
    --text:   #d4d8f0;
    --dim:    #6b7196;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: var(--bg);
    color: var(--text);
    font-family: 'Courier New', monospace;
    min-height: 100vh;
    padding: 24px 16px;
  }
  h1 {
    font-size: 1.1rem;
    letter-spacing: 0.15em;
    color: var(--accent);
    text-transform: uppercase;
    border-bottom: 1px solid var(--border);
    padding-bottom: 12px;
    margin-bottom: 24px;
  }
  .grid {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 16px;
    max-width: 640px;
    margin: 0 auto;
  }
  .card {
    background: var(--panel);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 18px;
  }
  .card.full { grid-column: 1 / -1; }
  .card h2 {
    font-size: 0.75rem;
    letter-spacing: 0.12em;
    color: var(--dim);
    text-transform: uppercase;
    margin-bottom: 16px;
  }
  .big-num {
    font-size: 2.8rem;
    font-weight: bold;
    line-height: 1;
    margin-bottom: 4px;
  }
  .big-num.left  { color: var(--accent); }
  .big-num.right { color: var(--warn); }
  .big-num.trim  { color: var(--green); }
  .label { font-size: 0.72rem; color: var(--dim); margin-bottom: 12px; }

  .slider-row { display: flex; align-items: center; gap: 10px; margin-top: 8px; }
  input[type=range] {
    flex: 1;
    -webkit-appearance: none;
    height: 4px;
    border-radius: 2px;
    background: var(--border);
    outline: none;
  }
  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none;
    width: 18px; height: 18px;
    border-radius: 50%;
    cursor: pointer;
  }
  .slider-left  input[type=range]::-webkit-slider-thumb { background: var(--accent); }
  .slider-right input[type=range]::-webkit-slider-thumb { background: var(--warn); }
  .slider-trim  input[type=range]::-webkit-slider-thumb { background: var(--green); }

  .btn-row { display: flex; gap: 8px; flex-wrap: wrap; margin-top: 12px; }
  button {
    flex: 1;
    padding: 9px 8px;
    font-size: 0.78rem;
    font-family: inherit;
    letter-spacing: 0.08em;
    border: 1px solid var(--border);
    background: var(--panel);
    color: var(--text);
    border-radius: 5px;
    cursor: pointer;
    transition: background 0.15s, border-color 0.15s;
  }
  button:hover { background: var(--border); border-color: var(--accent); }
  button.active { background: var(--accent); color: #fff; border-color: var(--accent); }

  .result-box {
    background: #0b0d14;
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 14px;
    font-size: 0.82rem;
    line-height: 1.8;
    margin-top: 8px;
  }
  .result-box .key  { color: var(--dim); }
  .result-box .val  { color: var(--green); }
  .result-box .code { color: var(--warn); }

  #status { font-size: 0.7rem; color: var(--dim); text-align: right; margin-bottom: 16px; }
  #status.ok  { color: var(--green); }
  #status.err { color: #e85b5b; }

  .mode-label {
    font-size: 0.7rem;
    letter-spacing: 0.1em;
    text-transform: uppercase;
    margin-bottom: 2px;
  }
  .mode-label.linked  { color: var(--accent); }
  .mode-label.indep   { color: var(--warn); }
</style>
</head>
<body>
<h1>⚙ Wheelie — Servo Calibration</h1>
<p id="status">Connecting...</p>

<div class="grid" style="max-width:640px;margin:0 auto;">

  <!-- Sync angle -->
  <div class="card full">
    <h2>Target Angle (both servos)</h2>
    <div class="slider-row slider-left">
      <span class="big-num left" id="dispTarget">25</span>°
      <input type="range" id="slTarget" min="20" max="100" value="25"
             oninput="setTarget(this.value)">
    </div>
    <div class="btn-row">
      <button onclick="goAngle(25)">25°</button>
      <button onclick="goAngle(45)">45°</button>
      <button onclick="goAngle(65)">65°</button>
      <button onclick="goAngle(85)">85°</button>
    </div>
  </div>

  <!-- Left servo -->
  <div class="card">
    <h2>Left Servo</h2>
    <div class="big-num left" id="dispL">25</div>
    <div class="label">degrees commanded</div>
    <div class="slider-row slider-left">
      <input type="range" id="slL" min="20" max="100" value="25"
             oninput="setLeft(this.value)">
    </div>
    <p id="modeL" class="mode-label linked">● linked to target</p>
  </div>

  <!-- Right servo -->
  <div class="card">
    <h2>Right Servo</h2>
    <div class="big-num right" id="dispR">25</div>
    <div class="label">degrees commanded</div>
    <div class="slider-row slider-right">
      <input type="range" id="slR" min="20" max="100" value="25"
             oninput="setRight(this.value)">
    </div>
    <p id="modeR" class="mode-label linked">● linked to target + trim</p>
  </div>

  <!-- Right trim -->
  <div class="card full">
    <h2>Right Trim  <span style="color:var(--dim);font-weight:normal">(add this many ° to right servo)</span></h2>
    <div class="slider-row slider-trim">
      <span style="color:var(--dim);font-size:.8rem">−20</span>
      <input type="range" id="slTrim" min="-20" max="20" value="0" step="1"
             oninput="setTrim(this.value)">
      <span style="color:var(--dim);font-size:.8rem">+20</span>
      <span class="big-num trim" id="dispTrim" style="font-size:2rem;min-width:52px;text-align:right">0</span>°
    </div>
    <div class="btn-row" style="margin-top:10px">
      <button onclick="adjTrim(-1)">− 1°</button>
      <button onclick="adjTrim(+1)">+ 1°</button>
      <button onclick="setTrim(0)">Reset trim</button>
    </div>
  </div>

  <!-- Results table -->
  <div class="card full">
    <h2>Recorded Trim Values</h2>
    <div class="result-box" id="recordBox">
      No readings recorded yet.<br>
      Use the <span class="code">📌 Record</span> button at each test angle.
    </div>
    <div class="btn-row" style="margin-top:10px">
      <button onclick="recordTrim()">📌 Record current angle + trim</button>
      <button onclick="clearRecords()">Clear</button>
    </div>
    <div class="result-box" id="codeBox" style="margin-top:10px;display:none">
      <span class="key">// Paste into main.cpp TaskServoCode:</span><br>
      <span id="codeOut"></span>
    </div>
  </div>

</div>

<script>
var ws, records = [];

function connect() {
  ws = new WebSocket('ws://' + location.hostname + '/ws');
  ws.onopen  = () => setStatus('Connected', true);
  ws.onclose = () => { setStatus('Disconnected — retrying…', false); setTimeout(connect, 2000); };
  ws.onerror = () => setStatus('Error', false);
}
function setStatus(msg, ok) {
  var el = document.getElementById('status');
  el.textContent = msg;
  el.className = ok ? 'ok' : 'err';
}
function send(path) { fetch(path).catch(() => {}); }

// ── Target (both) ────────────────────────────────────────────────────────────
function setTarget(v) {
  v = parseInt(v);
  document.getElementById('dispTarget').textContent = v;
  document.getElementById('slTarget').value = v;
  // Update left if linked
  if (document.getElementById('modeL').classList.contains('linked')) {
    document.getElementById('dispL').textContent = v;
    document.getElementById('slL').value = v;
  }
  // Update right if linked (right = target + trim)
  var trim = parseInt(document.getElementById('slTrim').value);
  if (document.getElementById('modeR').classList.contains('linked')) {
    var rv = Math.min(100, Math.max(20, v + trim));
    document.getElementById('dispR').textContent = rv;
    document.getElementById('slR').value = rv;
  }
  send('/set?type=T&val=' + v);
}
function goAngle(a) {
  document.getElementById('slTarget').value = a;
  setTarget(a);
}

// ── Trim ─────────────────────────────────────────────────────────────────────
function setTrim(v) {
  v = parseInt(v);
  v = Math.max(-20, Math.min(20, v));
  document.getElementById('slTrim').value = v;
  document.getElementById('dispTrim').textContent = (v >= 0 ? '+' : '') + v;
  send('/set?type=TRIM&val=' + v);
  // re-apply to right if linked
  var target = parseInt(document.getElementById('slTarget').value);
  if (document.getElementById('modeR').classList.contains('linked')) {
    var rv = Math.min(100, Math.max(20, target + v));
    document.getElementById('dispR').textContent = rv;
    document.getElementById('slR').value = rv;
  }
}
function adjTrim(delta) {
  var cur = parseInt(document.getElementById('slTrim').value);
  setTrim(cur + delta);
}

// ── Independent left override ────────────────────────────────────────────────
function setLeft(v) {
  v = parseInt(v);
  document.getElementById('dispL').textContent = v;
  document.getElementById('modeL').textContent = '● independent';
  document.getElementById('modeL').className = 'mode-label indep';
  send('/set?type=L&val=' + v);
}
function setRight(v) {
  v = parseInt(v);
  document.getElementById('dispR').textContent = v;
  document.getElementById('modeR').textContent = '● independent';
  document.getElementById('modeR').className = 'mode-label indep';
  send('/set?type=R&val=' + v);
}

// ── Record ────────────────────────────────────────────────────────────────────
function recordTrim() {
  var angle = parseInt(document.getElementById('slTarget').value);
  var trim  = parseInt(document.getElementById('slTrim').value);
  // replace if same angle already recorded
  records = records.filter(r => r.a !== angle);
  records.push({ a: angle, t: trim });
  records.sort((a, b) => a.a - b.a);
  renderRecords();
}
function clearRecords() {
  records = [];
  document.getElementById('recordBox').innerHTML =
    'No readings recorded yet.<br>Use the <span class="code">📌 Record</span> button at each test angle.';
  document.getElementById('codeBox').style.display = 'none';
}
function renderRecords() {
  if (records.length === 0) { clearRecords(); return; }
  var box = document.getElementById('recordBox');
  var html = '';
  records.forEach(r => {
    var sign = r.t >= 0 ? '+' : '';
    html += '<span class="key">servo ' + r.a + '°</span>  →  ' +
            '<span class="val">right_trim = ' + sign + r.t + '°</span><br>';
  });
  box.innerHTML = html;

  // Generate code suggestion
  var allSame = records.every(r => r.t === records[0].t);
  var codeEl  = document.getElementById('codeOut');
  var codeBox = document.getElementById('codeBox');
  if (allSame) {
    var t = records[0].t;
    var sign = t >= 0 ? '+' : '';
    codeEl.innerHTML =
      '<span class="code">#define SERVO_R_TRIM  ' + t + '</span><br>' +
      '<span class="key">// In TaskServoCode:</span><br>' +
      '<span class="code">LeftServo.write(constrain(Servo_angle, 20, 100));</span><br>' +
      '<span class="code">RightServo.write(constrain(Servo_angle + SERVO_R_TRIM, 20, 100));</span>';
  } else {
    var tableLines = records.map(r =>
      '  { ' + r.a + ', ' + (r.t >= 0 ? '+' : '') + r.t + ' }'
    ).join(',<br>');
    codeEl.innerHTML =
      '<span class="key">// Trim varies with angle — use a lookup table:</span><br>' +
      '<span class="code">struct TrimRow { int servo_deg; int right_trim; };<br>' +
      'TrimRow TRIM_TABLE[] = {<br>' + tableLines + '<br>};</span><br>' +
      '<span class="key">// In TaskServoCode, interpolate or nearest-match:</span><br>' +
      '<span class="code">int r_trim = lookupTrim(Servo_angle, TRIM_TABLE, ' + records.length + ');</span>';
  }
  codeBox.style.display = 'block';
}

connect();
</script>
</body>
</html>
)rawliteral";

// ── Globals ────────────────────────────────────────────────────────────────────
volatile int g_target_angle = 25;
volatile int g_right_trim   = 0;
volatile int g_left_override  = -1;   // -1 = follow target
volatile int g_right_override = -1;   // -1 = follow target + trim

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    // Attach both servo PWM channels (50 Hz, 12-bit)
    ledcAttach(SERVO_L_PIN, 50, 12);
    ledcAttach(SERVO_R_PIN, 50, 12);

    // Park at 25° on startup
    writeServo(SERVO_L_PIN, SERVO_L_MIN_US, SERVO_L_MAX_US, 25);
    writeServo(SERVO_R_PIN, SERVO_R_MIN_US, SERVO_R_MAX_US, 25);
    Serial.println("Servos initialised at 25°");

    // WiFi AP
    WiFi.mode(WIFI_AP);
    WiFi.softAP("Wheelie_Cal", "12345678");
    Serial.print("AP up — connect and open http://");
    Serial.println(WiFi.softAPIP());

    server.addHandler(&ws);

    // Serve the calibration page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
        req->send_P(200, "text/html", PAGE);
    });

    // /set?type=T&val=45     — move both to 45° (right gets +trim)
    // /set?type=TRIM&val=2   — set right trim offset
    // /set?type=L&val=47     — independent left override
    // /set?type=R&val=43     — independent right override
    server.on("/set", HTTP_GET, [](AsyncWebServerRequest *req) {
        if (req->hasParam("type") && req->hasParam("val")) {
            String type = req->getParam("type")->value();
            int    val  = req->getParam("val")->value().toInt();

            if (type == "T") {
                g_target_angle  = constrain(val, 20, 120);
                g_left_override  = -1;
                g_right_override = -1;
                Serial.printf("Target → %d°  (right effective: %d°)\n",
                              g_target_angle, g_target_angle + g_right_trim);
            }
            else if (type == "TRIM") {
                g_right_trim    = constrain(val, -20, 20);
                g_right_override = -1;  // re-link right to target
                Serial.printf("Right trim → %+d° (right effective: %d°)\n",
                              g_right_trim, g_target_angle + g_right_trim);
            }
            else if (type == "L") {
                g_left_override = constrain(val, 20, 120);
                Serial.printf("Left override → %d°\n", g_left_override);
            }
            else if (type == "R") {
                g_right_override = constrain(val, 20, 120);
                Serial.printf("Right override → %d°\n", g_right_override);
            }
        }
        req->send(200);
    });

    server.begin();
    Serial.println("Web server started.");
}

// ── Loop — write servos at 20 Hz ──────────────────────────────────────────────
void loop() {
    int left_cmd  = (g_left_override  >= 0) ? g_left_override
                                             : g_target_angle;
    int right_cmd = (g_right_override >= 0) ? g_right_override
                                             : constrain(g_target_angle + g_right_trim, 20, 100);

    writeServo(SERVO_L_PIN, SERVO_L_MIN_US, SERVO_L_MAX_US, left_cmd);
    writeServo(SERVO_R_PIN, SERVO_R_MIN_US, SERVO_R_MAX_US, right_cmd);

    ws.cleanupClients();
    delay(50);
}