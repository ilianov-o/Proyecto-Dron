#include "emergencia.h"
#include "wifi_telemetria.h"
#include "sensores.h"
#include "control_vuelo.h"

ModoVuelo modoVuelo = MODO_DESARMADO;

static unsigned long tiempoPerdidaSignal = 0;
static bool botonPresionadoAntes = false;

// ==========================================
void verificarBotonEmergencia() {
    bool presionado = (digitalRead(PIN_BOTON_EMERGENCIA) == LOW);

    if (presionado && !botonPresionadoAntes) {
        if (modoVuelo == MODO_ARMADO) {
            Serial.println("[EMERGENCIA] Botón físico activado. Iniciando aterrizaje...");
            modoVuelo = MODO_ATERRIZAJE_EMERGENCIA;
        }
    }
    botonPresionadoAntes = presionado;
}

// ==========================================
void verificarTimeoutSignal() {
    if (modoVuelo != MODO_ARMADO) return;

    if (clienteConectado()) {
        tiempoPerdidaSignal = millis();
    } else if (millis() - tiempoPerdidaSignal > TIMEOUT_SIGNAL_MS) {
        Serial.println("[EMERGENCIA] Pérdida de señal WiFi. Iniciando aterrizaje...");
        modoVuelo = MODO_ATERRIZAJE_EMERGENCIA;
    }
}

// ==========================================
void ejecutarAterrizajeEmergencia() {
    static float alturaObjetivoDescenso = 0;
    static bool inicializado = false;
    static unsigned long tUltimoDescenso = 0;

    if (!inicializado) {
        alturaObjetivoDescenso = alturaControl;
        inicializado = true;
        tUltimoDescenso = millis();
    }

    // Descenso controlado cada 50 ms
    if (millis() - tUltimoDescenso > 50) {
        tUltimoDescenso = millis();

        // Si el LiDAR detecta menos de 10 cm, cortar motores
        if (usarLiDAR && alturaLiDAR < ALTURA_CORTE_MOTORES) {
            Serial.printf("[ATERRIZAJE] Suelo detectado a %.2f m. Cortando motores.\n", alturaLiDAR);
            modoVuelo = MODO_EMERGENCIA_DETENIDO;
            inicializado = false;
            return;
        }

        // Decrementar altura objetivo
        alturaObjetivoDescenso -= VELOCIDAD_DESCENSO * 0.05f; // 0.5 m/s * 0.05 s
        if (alturaObjetivoDescenso < 0) alturaObjetivoDescenso = 0;
        alturaObjetivo = alturaObjetivoDescenso;
    }

    // Si la señal WiFi vuelve, recuperar control
    if (clienteConectado()) {
        Serial.println("[INFO] Señal recuperada. Retomando control.");
        modoVuelo = MODO_ARMADO;
        inicializado = false;
    }
}