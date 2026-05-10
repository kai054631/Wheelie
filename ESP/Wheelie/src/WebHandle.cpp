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
    body { font-family: Arial; text-align: center; background: #1a1a1a; color: white; padding-top: 20px;}
    .card { background: #333; padding: 20px; border-radius: 15px; display: inline-block; width: 300px; box-shadow: 0 4px 15px rgba(0,0,0,0.5); }
    .btn { width: 80px; height: 80px; margin: 10px; background: #007bff; color: white; border: none; border-radius: 50%; font-size: 20px; touch-action: manipulation; }
    .btn:active { background: #0056b3; }
    .slider { width: 100%; margin: 15px 0; }
    span { color: #ffcc00; font-weight: bold; }
  </style>
</head>
<body>
  <div class="card">
    <h3>运动控制</h3>
    <button class="btn" onpointerdown="move('F')" onpointerup="move('S')">前</button><br>
    <button class="btn" onpointerdown="move('L')" onpointerup="move('S')">左</button>
    <button class="btn" onpointerdown="move('B')" onpointerup="move('S')">后</button>
    <button class="btn" onpointerdown="move('R')" onpointerup="move('S')">右</button>
    <hr>
    <h3>PID 调参</h3>
    <p>Kp: <span id="kp_val">%KP%</span></p>
    <input type="range" class="slider" min="0" max="10" step="0.1" value="%KP%" oninput="update('P',this.value)">
    
    <p>Ki: <span id="ki_val">%KI%</span></p>
    <input type="range" class="slider" min="0" max="1" step="0.01" value="%KI%" oninput="update('I',this.value)"> 
    
    <p>Kd: <span id="kd_val">%KD%</span></p>
    <input type="range" class="slider" min="0" max="1" step="0.01" value="%KD%" oninput="update('D',this.value)">
    
    <p>Offset: <span id="off_val">%OFF%</span></p>
    <input type="range" class="slider" min="-15" max="15" step="0.1" value="%OFF%" oninput="update('O',this.value)">
  </div>

  <script>
    function move(d) { fetch('/move?dir=' + d); }
    function update(t, v) { 
      fetch('/set?type=' + t + '&val=' + v); 
      // 修正：这里加入了 ki_val 的映射
      let id = (t=='P')?'kp_val':(t=='I')?'ki_val':(t=='D')?'kd_val':'off_val';
      let element = document.getElementById(id);
      if(element) element.innerHTML = v;
    }
  </script>
</body>
</html>)rawliteral";

// 处理器：确保每个占位符都有返回，哪怕是 0
String processor(const String &var) {
  if (var == "KP") return String(Kp, 1);
  if (var == "KD") return String(Kd, 2);
  if (var == "KI") return String(Ki, 2);
  if (var == "OFF") return String(angle_offset, 1);
  return "0"; 
}

void setupWebServer() {
  WiFi.mode(WIFI_STA);
  WiFi.begin("smarthome_", "11121968");
  
  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected! IP: " + WiFi.localIP().toString());

  // 必须在 server.begin 之前添加处理器
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
     request->send(200, "text/html", index_html, processor); 
  });

  server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("type") && request->hasParam("val")) {
      String type = request->getParam("type")->value();
      float val = request->getParam("val")->value().toFloat();
      if (type == "P") Kp = val;
      else if (type == "D") Kd = val;
      else if (type == "I") Ki = val;
      else if (type == "O") angle_offset = val;
    }
    request->send(200); 
  });

  server.on("/move", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("dir")) {
      String dir = request->getParam("dir")->value();
      if(dir == "F") move_velocity = -1.3; 
      else if(dir == "B") move_velocity = 1.3;
      else if(dir == "L") turn_velocity = 0.8; 
      else if(dir == "R") turn_velocity = -0.8;
      else { move_velocity = 0; turn_velocity = 0; }
    }
    request->send(200); 
  });

  server.begin();
}