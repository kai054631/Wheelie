#include "config.h"
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta charset="utf-8">
  <title>Wheelie Remote</title>
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <style>
    body { font-family: Arial; text-align: center; background: #1a1a1a; color: white; padding-top: 10px;}
    .card { background: #333; padding: 20px; border-radius: 15px; display: inline-block; width: 300px; box-shadow: 0 4px 15px rgba(0,0,0,0.5); margin-bottom: 10px;}
    .btn { width: 75px; height: 75px; margin: 5px; background: #007bff; color: white; border: none; border-radius: 50%; font-size: 18px; touch-action: manipulation; }
    .btn-alt { width: 120px; height: 50px; background: #28a745; border-radius: 10px; margin: 10px; font-weight: bold; cursor: pointer;}
    .btn:active, .btn-alt:active { background: #1e7e34; transform: scale(0.95); }
    .slider { width: 100%; margin: 15px 0; }
    span { color: #ffcc00; font-weight: bold; }
  </style>
</head>
<body>
  <div class="card">
    <h3>姿态控制 (舵机)</h3>
    <button class="btn-alt" onclick="setServo(25)">25° (低姿态)</button>
    <button class="btn-alt" onclick="setServo(45)">45° (高姿态)</button>
    <p>当前舵机目标: <span id="servo_val">45</span>°</p>
  </div>

  <div class="card">
    <h3>运动控制</h3>
    <button class="btn" onpointerdown="move('F')" onpointerup="move('S')">前</button><br>
    <button class="btn" onpointerdown="move('L')" onpointerup="move('S')">左</button>
    <button class="btn" onpointerdown="move('B')" onpointerup="move('S')">后</button>
    <button class="btn" onpointerdown="move('R')" onpointerup="move('S')">右</button>
  </div>

  <div class="card">
    <h3>Pitch PID 调参</h3>
    <p>Kp: <span id="kp_val">%KP%</span> <input type="range" class="slider" min="0" max="15" step="0.1" value="%KP%" oninput="update('P',this.value)"></p>
    <p>Ki: <span id="ki_val">%KI%</span> <input type="range" class="slider" min="0" max="0.5" step="0.001" value="%KI%" oninput="update('I',this.value)"></p>
    <p>Kd: <span id="kd_val">%KD%</span> <input type="range" class="slider" min="0" max="1" step="0.01" value="%KD%" oninput="update('D',this.value)"></p>
    <p>Offset: <span id="off_val">%OFF%</span> <input type="range" class="slider" min="-15" max="15" step="0.1" value="%OFF%" oninput="update('O',this.value)"></p>
  </div>

  <script>
    function move(d) { fetch('/move?dir=' + d); }
    function setServo(a) { 
      fetch('/servo?angle=' + a); 
      document.getElementById('servo_val').innerHTML = a;
    }
    function update(t, v) { 
      fetch('/set?type=' + t + '&val=' + v); 
      // Link the JS directly to the Pitch variables
      let id = (t=='P') ? 'kp_val' : (t=='I') ? 'ki_val' : (t=='D') ? 'kd_val' : 'off_val';
      document.getElementById(id).innerHTML = v;
    }
  </script>
</body>
</html>)rawliteral";

// 处理器：确保每个占位符都有返回
String processor(const String &var)
{
  if (var == "KP") return String(Kp, 1);
  if (var == "KI") return String(Ki, 3); // Changed to 3 decimal places for finer Ki tuning
  if (var == "KD") return String(Kd, 2);
  if (var == "OFF") return String(angle_offset, 2);
  return "0";
}

void setupWebServer()
{
  WiFi.mode(WIFI_STA);
  // WiFi.begin("yong-ThinkPad-T470", "00000000");
  WiFi.begin("smarthome_", "11121968");

  Serial.print("Connecting WiFi\n");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print("Connecting WiFi\n");
  }
  Serial.println("\nWiFi Connected! IP: " + WiFi.localIP().toString());

  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(200, "text/html", index_html, processor); });

  server.on("/servo", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    if (request->hasParam("angle")) {
      int target_angle = request->getParam("angle")->value().toInt();
      Servo_angle = target_angle;
      if(target_angle == 25)
      {
        angle_offset = -10.30; 
      }else
      {
        angle_offset = -2.50; 
      }

      Serial.printf("Servo target updated to: %d\n", Servo_angle);
      Serial.printf("Angle offset updated to: %.2f\n", angle_offset);
    }
    request->send(200); });

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    if (request->hasParam("type") && request->hasParam("val")) {
      String type = request->getParam("type")->value();
      float val = request->getParam("val")->value().toFloat();
      
      // Assigns the slider values to your global Pitch PID variables
      if (type == "P") Kp = val;
      else if (type == "I") Ki = val;
      else if (type == "D") Kd = val;
      else if (type == "O") angle_offset = val;
      
      Serial.printf("Updated %s to %.3f\n", type.c_str(), val);
    }
    request->send(200); });

  server.on("/move", HTTP_GET, [](AsyncWebServerRequest *request)
            {
    if (request->hasParam("dir")) {
      String dir = request->getParam("dir")->value();
      if(dir == "F") move_velocity = -1.5; 
      else if(dir == "B") move_velocity = 1.3;
      else if(dir == "L") turn_velocity = 0.8; 
      else if(dir == "R") turn_velocity = -0.8;
      else { move_velocity = 0; turn_velocity = 0; }
    }
    request->send(200); });

  server.begin();
}