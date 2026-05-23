#include "config.h"
#include <WiFi.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// HTML Dashboard showing clean LQR Tuning inputs only
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <title>Wheelie Static LQR Tuning</title>
    <meta name="viewport" content="width=device-width, initial-scale=1" charset="UTF-8">
    <style>
        body { font-family: Arial; text-align: center; background-color: #222; color: #fff; }
        .card { background-color: #333; padding: 15px; margin: 20px auto; border-radius: 10px; max-width: 400px; }
        button { padding: 10px 20px; font-size: 16px; margin: 5px; cursor: pointer; background: #007BFF; color: white; border: none; border-radius: 5px; }
        button:active { background: #0056b3; }
        input { padding: 10px; font-size: 16px; width: 100px; text-align: center; margin-right: 10px; }
        .tune-row { margin: 10px 0; display: flex; justify-content: center; align-items: center; }
        #status { font-weight: bold; color: #FFA500; }
    </style>
</head>
<body>
    <h2>Wheelie Static LQR Tuning</h2>
    <p>Connection Status: <span id="status">Connecting...</span></p>
    
    <div class="card">
        <h3>Live Matrix Adjustments</h3>
        <div class="tune-row"><label>K1 (Pos): </label><input type="number" id="valK1" step="0.1"><button onclick="updateK('K1')">Set</button></div>
        <div class="tune-row"><label>K2 (Vel): </label><input type="number" id="valK2" step="0.1"><button onclick="updateK('K2')">Set</button></div>
        <div class="tune-row"><label>K3 (Ang): </label><input type="number" id="valK3" step="0.1"><button onclick="updateK('K3')">Set</button></div>
        <div class="tune-row"><label>K4 (Gyro):</label><input type="number" id="valK4" step="0.1"><button onclick="updateK('K4')">Set</button></div>
        <div class="tune-row"><label>Offset: </label><input type="number" id="valO" step="0.1"><button onclick="updateK('O')">Set</button></div>
    </div>

    <script>
        var gateway = `ws://${window.location.hostname}/ws`;
        var websocket;

        function initWebSocket() {
            console.log('Attempting to open WebSocket connection to: ' + gateway);
            websocket = new WebSocket(gateway);

            websocket.onopen = function(event) {
                console.log('SUCCESS: WebSocket Connection Established!');
                document.getElementById('status').innerHTML = "CONNECTED";
                document.getElementById('status').style.color = "#00FF00";
            };

            websocket.onclose = function(event) {
                console.log('WARNING: WebSocket Connection Closed. Retrying in 2 seconds...');
                document.getElementById('status').innerHTML = "DISCONNECTED";
                document.getElementById('status').style.color = "#FF0000";
                setTimeout(initWebSocket, 2000); // Auto-reconnect safety loop
            };

            websocket.onerror = function(error) {
                console.log('ERROR: WebSocket encountered an issue: ', error);
            };

            websocket.onmessage = function(event) {
                // This will force the raw CSV lines straight into your browser console logs
                console.log("DATA RECEIVED: " + event.data); 
            };
        }

        function updateK(type) {
            var val = document.getElementById('val' + type).value;
            fetch(`/set?type=${type}&val=${val}`)
            .then(response => console.log('Param sync status: OK'))
            .catch(err => console.log('Param sync failed: ', err));
            alert(type + " updated to " + val);
        }

        // Initialize script execution after document finishes loading
        window.onload = function() {
            initWebSocket();
        };
    </script>
</body>
</html>
)rawliteral";

void setupWebServer()
{
     Serial.println("Starting WiFi AP...");
     WiFi.mode(WIFI_AP);
     WiFi.softAP("Wheelie_Robot", "12345678");

     IPAddress IP = WiFi.softAPIP();
     Serial.print("AP Started! Connect to 'Wheelie_Robot' and open: ");
     Serial.println(IP);

     server.addHandler(&ws);

     server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
               { request->send(200, "text/html", index_html); });

     // Handle updates for values submitted by input text boxes
     server.on("/set", HTTP_GET, [](AsyncWebServerRequest *request)
               {
        if (request->hasParam("type") && request->hasParam("val")) {
            String type = request->getParam("type")->value();
            float val = request->getParam("val")->value().toFloat();
            
            if (type == "K1") K1 = val;
            else if (type == "K2") K2 = val;
            else if (type == "K3") K3 = val;
            else if (type == "K4") K4 = val;
            else if (type == "O") angle_offset = val;
            
            Serial.printf("Updated via Web: %s = %.4f\n", type.c_str(), val);
        }
        request->send(200); });

     server.begin();
}