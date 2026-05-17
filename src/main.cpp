#include "config.h"
#include "wifi_telemetria.h"
#include "sensores.h"
#include "control_vuelo.h"
#include "motores.h"
#include "emergencia.h"
#include "control_actitud.h"

// ===================== VARIABLES GLOBALES =====================
extern ModoVuelo modoVuelo;      // Definido en emergencia.cpp
extern float alturaObjetivo;     // Definido en control_vuelo.cpp
unsigned long tUltimoLoop = 0;

// ===================== SETUP =====================
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n========================================");
    Serial.println("   DRON F450 - SISTEMA INICIANDO");
    Serial.println("========================================\n");

    // Inicializar I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(400000);

    // Inicializar sensores
    if (!iniciarSensores()) {
        Serial.println("[FATAL] Fallo en sensores. Reinicie el sistema.");
        while (1) delay(100);
    }

    // Inicializar motores
    initControlActitud();
    iniciarMotores();
    detenerMotores();   // esto ya manda PWM_MIN
    delay(3000);        // espera a que los ESCs se inicialicen y piten confirmación

    // Inicializar WiFi y WebSocket
    iniciarWiFi();

    // Configurar botón de emergencia
    pinMode(PIN_BOTON_EMERGENCIA, INPUT_PULLUP);

    Serial.println("\n========================================");
    Serial.println("   SISTEMA LISTO PARA VOLAR");
    Serial.println("========================================");
    Serial.printf("Conéctate a la red WiFi: %s\n", WIFI_SSID);
    Serial.printf("Abre: http://%s en tu navegador\n", WiFi.softAPIP().toString().c_str());
    Serial.println("Comandos: armar | desarmar | altura X.X");
    Serial.println("========================================\n");

    tUltimoLoop = millis();
}

// ===================== LOOP =====================
void loop() {
    webSocket.loop();
    server.handleClient();

    actualizarActitud();
    actualizarAltitud();

    verificarBotonEmergencia();
    verificarTimeoutSignal();

    switch (modoVuelo) {
        case MODO_DESARMADO:
            detenerMotores();
            break;

        case MODO_ARMADO: {
            float dt = (millis() - tUltimoLoop) / 1000.0;   // dt del lazo
            controlAltitud();
            mezclarMotores();
            break;
        }

        case MODO_ATERRIZAJE_EMERGENCIA:
            ejecutarAterrizajeEmergencia();
            mezclarMotores();
            break;

        case MODO_EMERGENCIA_DETENIDO:
            detenerMotores();
            break;
    }

    aplicarPWM();
    enviarTelemetria();

    while (millis() - tUltimoLoop < 10) delay(1);
    tUltimoLoop = millis();
}