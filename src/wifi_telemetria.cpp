#include "wifi_telemetria.h"
#include "emergencia.h"
#include "config.h"
#include "sensores.h"

// Variables externas (definidas en otros módulos)
extern float angRoll, angPitch, angYaw;
extern float alturaControl, alturaLiDAR, alturaBaro;
extern float ax_g, ay_g, az_g;
extern float salidaPID;
extern ModoVuelo modoVuelo;
extern float alturaObjetivo;
extern bool setpointManual;                     // definida en control_vuelo.cpp
extern uint16_t pwmMotor1, pwmMotor2, pwmMotor3, pwmMotor4;   // definidas en motores.cpp

WebSocketsServer webSocket = WebSocketsServer(WEBSOCKET_PORT);
WebServer server(80);

static bool cliente_activo = false;
static unsigned long tUltimoEnvio = 0;

// ===================== DASHBOARD HTML =====================
const char* dashboard_html = R"rawliteral(
<!DOCTYPE html>
<html lang="es">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Dron F450 - Control Inteligente</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: #0d1b2a;
            color: #e0e1dd;
            display: flex;
            flex-direction: column;
            align-items: center;
            padding: 15px;
            min-height: 100vh;
        }
        h1 { color: #e94560; margin-bottom: 15px; }
        .grid-container {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
            gap: 12px;
            width: 100%;
            max-width: 900px;
        }
        .card {
            background: #1b263b;
            border-radius: 12px;
            padding: 15px;
            box-shadow: 0 4px 15px rgba(0,0,0,0.5);
        }
        .card h2 { font-size: 1.1em; color: #a2d2ff; margin-bottom: 10px; border-bottom: 1px solid #415a77; padding-bottom: 5px; }
        .row { display: flex; justify-content: space-between; margin: 5px 0; }
        .value { font-weight: bold; color: #f4a261; }
        .big-value { font-size: 2.2em; text-align: center; margin: 10px 0; color: #e9c46a; }
        .controls { display: flex; gap: 10px; flex-wrap: wrap; justify-content: center; margin: 15px 0; }
        button {
            padding: 12px 20px;
            border: none;
            border-radius: 8px;
            font-size: 16px;
            font-weight: bold;
            cursor: pointer;
            transition: transform 0.2s, box-shadow 0.2s;
        }
        button:active { transform: scale(0.95); }
        #btn-armar { background: #2ecc71; color: white; }
        #btn-desarmar { background: #e67e22; color: white; }
        #btn-emergencia { background: #e74c3c; color: white; font-size: 18px; padding: 15px 30px; width: 100%; }
        .status-box {
            text-align: center;
            padding: 12px;
            border-radius: 8px;
            font-weight: bold;
            margin: 10px 0;
        }
        .desarmado { background: #7f8c8d; }
        .armado { background: #27ae60; }
        .aterrizaje { background: #f39c12; }
        .emergencia { background: #c0392b; }
        .pid-constants { font-family: monospace; }
    </style>
</head>
<body>
    <h1>🚁 DRON F450 - CONTROL INTELIGENTE</h1>

    <!-- Estado y altura principal -->
    <div class="card" style="width:100%; max-width:900px; margin-bottom:15px;">
        <div class="status-box" id="status">ESTADO: DESARMADO</div>
        <div class="big-value" id="altura_control">0.00 m</div>
        <div style="text-align:center; font-size:0.9em;">Altura de control (fusionada)</div>
    </div>

    <!-- Setpoint -->
    <div class="card" style="width:100%; max-width:900px; margin-bottom:15px; text-align:center;">
        <label for="setpoint_input">Altitud deseada (m):</label>
        <input type="number" id="setpoint_input" value="1.0" step="0.1" min="0" max="10" style="padding:8px; font-size:16px; width:100px; border-radius:6px; border:none;">
        <button onclick="enviarSetpoint()" style="background:#a2d2ff; color:#0d1b2a;">Establecer</button>
    </div>

    <div class="grid-container">
        <!-- Actitud -->
        <div class="card">
            <h2>📐 Actitud (º)</h2>
            <div class="row"><span>Roll:</span><span class="value" id="roll">0.0</span></div>
            <div class="row"><span>Pitch:</span><span class="value" id="pitch">0.0</span></div>
            <div class="row"><span>Yaw:</span><span class="value" id="yaw">0.0</span></div>
        </div>

        <!-- Aceleraciones -->
        <div class="card">
            <h2>⚡ Aceleración (m/s²)</h2>
            <div class="row"><span>Eje X:</span><span class="value" id="ax_ms2">0.00</span></div>
            <div class="row"><span>Eje Y:</span><span class="value" id="ay_ms2">0.00</span></div>
            <div class="row"><span>Eje Z:</span><span class="value" id="az_ms2">0.00</span></div>
        </div>

        <!-- Sensores de altura -->
        <div class="card">
            <h2>📏 Alturas (m)</h2>
            <div class="row"><span>LiDAR (VL53L0X):</span><span class="value" id="altura_lidar">0.00</span></div>
            <div class="row"><span>Barómetro (MS5611):</span><span class="value" id="altura_baro">0.00</span></div>
            <div class="row"><span>Objetivo:</span><span class="value" id="altura_obj">1.00</span></div>
        </div>

        <!-- Constantes PID -->
        <div class="card">
            <h2>🔧 PID Altitud</h2>
            <div class="pid-constants">
                <div class="row"><span>Kp:</span><span id="kp_val">15.0</span></div>
                <div class="row"><span>Ki:</span><span id="ki_val">0.5</span></div>
                <div class="row"><span>Kd:</span><span id="kd_val">8.0</span></div>
                <div class="row"><span>Salida PID:</span><span class="value" id="pid">0.000</span></div>
            </div>
        </div>

        <!-- Motores -->
        <div class="card">
            <h2>🛵 Motores (PWM)</h2>
            <div class="row"><span>Motor 1:</span><span class="value" id="motor1">0</span></div>
            <div class="row"><span>Motor 2:</span><span class="value" id="motor2">0</span></div>
            <div class="row"><span>Motor 3:</span><span class="value" id="motor3">0</span></div>
            <div class="row"><span>Motor 4:</span><span class="value" id="motor4">0</span></div>
        </div>
    </div>

    <!-- Controles -->
    <div class="controls">
        <button id="btn-armar" onclick="enviarComando('armar')">🔓 ARMAR</button>
        <button id="btn-desarmar" onclick="enviarComando('desarmar')">🔒 DESARMAR</button>
    </div>
    <button id="btn-emergencia" onclick="ws.send('emergencia')">🚨 PARADA DE EMERGENCIA</button>

    <script>
        const ws = new WebSocket('ws://' + location.hostname + ':81');
        ws.onmessage = function(event) {
            try {
                const data = JSON.parse(event.data);
                // Actitud
                document.getElementById('roll').textContent = data.roll.toFixed(1);
                document.getElementById('pitch').textContent = data.pitch.toFixed(1);
                document.getElementById('yaw').textContent = data.yaw.toFixed(1);
                // Aceleraciones
                document.getElementById('ax_ms2').textContent = data.ax_ms2;
                document.getElementById('ay_ms2').textContent = data.ay_ms2;
                document.getElementById('az_ms2').textContent = data.az_ms2;
                // Alturas
                document.getElementById('altura_control').textContent = data.altura_ctrl.toFixed(2) + ' m';
                document.getElementById('altura_lidar').textContent = data.altura_lidar.toFixed(2);
                document.getElementById('altura_baro').textContent = data.altura_baro.toFixed(2);
                document.getElementById('altura_obj').textContent = data.altura_obj.toFixed(2);
                // PID
                document.getElementById('kp_val').textContent = data.Kp.toFixed(1);
                document.getElementById('ki_val').textContent = data.Ki.toFixed(2);
                document.getElementById('kd_val').textContent = data.Kd.toFixed(1);
                document.getElementById('pid').textContent = data.pid.toFixed(3);
                // Estado
                const statusEl = document.getElementById('status');
                statusEl.textContent = 'ESTADO: ' + data.modo;
                statusEl.className = 'status-box ' + data.modo.toLowerCase();
                // Motores
                document.getElementById('motor1').textContent = data.motor1;
                document.getElementById('motor2').textContent = data.motor2;
                document.getElementById('motor3').textContent = data.motor3;
                document.getElementById('motor4').textContent = data.motor4;
            } catch (e) {
                console.error('Error JSON:', e);
            }
        };
        ws.onopen = function() { console.log('WebSocket conectado'); };
        ws.onclose = function() { console.log('WebSocket desconectado'); };

        function enviarComando(cmd) {
            if (ws.readyState === WebSocket.OPEN) {
                ws.send(JSON.stringify({comando: cmd}));
                console.log('Comando enviado:', cmd);
            } else {
                alert('No hay conexión con el dron');
            }
        }

        function enviarSetpoint() {
            const sp = document.getElementById('setpoint_input').value;
            enviarComando('setpoint ' + sp);
        }
    </script>
</body>
</html>
)rawliteral";

// ===================== FUNCIONES =====================
void handleRoot() {
    server.send(200, "text/html", dashboard_html);
}

void iniciarWiFi() {
    WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    Serial.printf("[WiFi] Punto de acceso creado. IP: %s\n", WiFi.softAPIP().toString().c_str());

    webSocket.begin();
    webSocket.onEvent(webSocketEvent);
    Serial.println("[WebSocket] Servidor iniciado en puerto " + String(WEBSOCKET_PORT));

    server.on("/", handleRoot);
    server.begin();
    Serial.println("[HTTP] Servidor iniciado en puerto 80");
}

void enviarTelemetria() {
    if (!cliente_activo) return;
    if (millis() - tUltimoEnvio < (1000 / TELEMETRIA_HZ)) return;
    tUltimoEnvio = millis();

    // Aceleraciones convertidas a m/s²
    float ax_ms2 = ax_g * 9.81;
    float ay_ms2 = ay_g * 9.81;
    float az_ms2 = az_g * 9.81;

    String json = "{";
    json += "\"roll\":" + String(angRoll, 1) + ",";
    json += "\"pitch\":" + String(angPitch, 1) + ",";
    json += "\"yaw\":" + String(angYaw, 1) + ",";
    json += "\"ax_ms2\":" + String(ax_ms2, 2) + ",";
    json += "\"ay_ms2\":" + String(ay_ms2, 2) + ",";
    json += "\"az_ms2\":" + String(az_ms2, 2) + ",";
    json += "\"altura_ctrl\":" + String(alturaControl, 2) + ",";
    json += "\"altura_lidar\":" + String(alturaLiDAR, 2) + ",";
    json += "\"altura_baro\":" + String(alturaBaro, 2) + ",";
    json += "\"altura_obj\":" + String(alturaObjetivo, 2) + ",";
    json += "\"pid\":" + String(salidaPID, 3) + ",";
    json += "\"Kp\":" + String(PID_ALT_KP, 1) + ",";
    json += "\"Ki\":" + String(PID_ALT_KI, 2) + ",";
    json += "\"Kd\":" + String(PID_ALT_KD, 1) + ",";
    json += "\"motor1\":" + String(pwmMotor1) + ",";
    json += "\"motor2\":" + String(pwmMotor2) + ",";
    json += "\"motor3\":" + String(pwmMotor3) + ",";
    json += "\"motor4\":" + String(pwmMotor4) + ",";
    json += "\"modo\":\"" + String(modoVuelo == MODO_DESARMADO ? "DESARMADO" :
                                  modoVuelo == MODO_ARMADO ? "ARMADO" :
                                  modoVuelo == MODO_ATERRIZAJE_EMERGENCIA ? "ATERRIZAJE" : "EMERGENCIA") + "\",";
    json += "\"timestamp\":" + String(millis());
    json += "}";

    webSocket.broadcastTXT(json);
}

bool clienteConectado() {
    return cliente_activo;
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_DISCONNECTED:
            cliente_activo = false;
            Serial.printf("[WebSocket] Cliente %u desconectado\n", num);
            break;

        case WStype_CONNECTED:
            cliente_activo = true;
            Serial.printf("[WebSocket] Cliente %u conectado desde %s\n",
                          num, webSocket.remoteIP(num).toString().c_str());
            break;

        case WStype_TEXT: {
            String payloadStr = String((char*)payload);
            payloadStr.trim();
            String comando = payloadStr;
            
            // Si viene como JSON {"comando":"..."}, extraer el valor
            if (payloadStr.startsWith("{")) {
                int start = payloadStr.indexOf("\"comando\":\"");
                if (start >= 0) {
                    start += 11; // longitud de "comando":"
                    int end = payloadStr.indexOf("\"", start);
                    if (end > start) {
                        comando = payloadStr.substring(start, end);
                    }
                }
            }
            Serial.printf("[WebSocket] Comando recibido: %s\n", comando.c_str());

            // Comandos de emergencia (texto plano "emergencia" o "EMERGENCY_STOP")
            if (comando.equalsIgnoreCase("emergencia") || comando.equalsIgnoreCase("EMERGENCY_STOP")) {
                Serial.println(">> ¡PARADA DE EMERGENCIA POR WiFi!");
                if (modoVuelo == MODO_ARMADO || modoVuelo == MODO_ATERRIZAJE_EMERGENCIA) {
                    modoVuelo = MODO_ATERRIZAJE_EMERGENCIA;
                }
            }
            else if (comando.equalsIgnoreCase("armar")) {
                if (!setpointManual) {
                    Serial.println(">> No se puede armar: primero establezca una altura objetivo (setpoint)");
                } else if (modoVuelo == MODO_DESARMADO) {
                    modoVuelo = MODO_ARMADO;
                    Serial.println(">> ARMADO");
                }
            }
            else if (comando.equalsIgnoreCase("desarmar")) {
                modoVuelo = MODO_DESARMADO;
                setpointManual = false;
                Serial.println(">> DESARMADO (setpoint reset)");
            }
            else if (comando.startsWith("setpoint ")) {
                float nueva = comando.substring(9).toFloat();
                if (nueva > 0 && nueva < 10) {
                    alturaObjetivo = nueva;
                    setpointManual = true;
                    Serial.printf(">> Altura objetivo: %.2f m (manual)\n", alturaObjetivo);
                }
            }
            else if (comando.startsWith("altura ")) {
                float nueva = comando.substring(7).toFloat();
                if (nueva > 0 && nueva < 10) {
                    alturaObjetivo = nueva;
                    setpointManual = true;
                    Serial.printf(">> Altura objetivo: %.2f m (manual)\n", alturaObjetivo);
                }
            }
            else {
                Serial.println(">> Comando desconocido");
            }
            break;
        }
    }
}