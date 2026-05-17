#include "motores.h"
#include "control_vuelo.h"
#include "emergencia.h"      // para acceder a modoVuelo
#include "config.h"

// Correcciones de actitud (calculadas por el control de actitud)
extern float roll_corr;
extern float pitch_corr;
extern float yaw_corr;

uint16_t pwmMotor1 = 0, pwmMotor2 = 0, pwmMotor3 = 0, pwmMotor4 = 0;

// Variables para la rampa de arranque
static bool rampaActiva = false;
static unsigned long tiempoInicioRampa = 0;

// ==========================================
void iniciarMotores() {
    ledcSetup(0, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(1, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(2, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(3, PWM_FREQ, PWM_RESOLUTION);

    ledcAttachPin(MOTOR1_PIN, 0);
    ledcAttachPin(MOTOR2_PIN, 1);
    ledcAttachPin(MOTOR3_PIN, 2);
    ledcAttachPin(MOTOR4_PIN, 3);

    Serial.println("[Motores] PWM inicializado");
}

// ==========================================
void mezclarMotores() {
    float throttle;

    // Rampa de arranque al armar
    static bool estabaArmado = false;
    if (modoVuelo == MODO_ARMADO && !estabaArmado) {
        rampaActiva = true;
        tiempoInicioRampa = millis();
    }
    estabaArmado = (modoVuelo == MODO_ARMADO);

    if (rampaActiva) {
        unsigned long transcurrido = millis() - tiempoInicioRampa;
        if (transcurrido >= RAMP_TIME_MS) {
            rampaActiva = false;
            throttle = THROTTLE_HOVER;
        } else {
            float factor = (float)transcurrido / RAMP_TIME_MS;
            throttle = THROTTLE_HOVER * factor;
        }
    } else {
        throttle = THROTTLE_HOVER;
    }

    // Corrección de altitud (PID o fuzzy) se suma al throttle base
    throttle += salidaPID;
    throttle = constrain(throttle, 0.0f, 1.0f);

    // Mezcla para cada motor (configuración X)
    // Motor 1 (frente izquierdo): +roll -pitch -yaw
    // Motor 2 (frente derecho):   -roll -pitch +yaw
    // Motor 3 (trasero derecho):  -roll +pitch -yaw
    // Motor 4 (trasero izquierdo):+roll +pitch +yaw
    float throttle1 = throttle + roll_corr - pitch_corr - yaw_corr;
    float throttle2 = throttle - roll_corr - pitch_corr + yaw_corr;
    float throttle3 = throttle - roll_corr + pitch_corr - yaw_corr;
    float throttle4 = throttle + roll_corr + pitch_corr + yaw_corr;

    // Limitar entre 0 y 1
    throttle1 = constrain(throttle1, 0.0f, 1.0f);
    throttle2 = constrain(throttle2, 0.0f, 1.0f);
    throttle3 = constrain(throttle3, 0.0f, 1.0f);
    throttle4 = constrain(throttle4, 0.0f, 1.0f);

    // Convertir a PWM
    pwmMotor1 = map(throttle1 * 1000, 0, 1000, PWM_MIN, PWM_MAX);
    pwmMotor2 = map(throttle2 * 1000, 0, 1000, PWM_MIN, PWM_MAX);
    pwmMotor3 = map(throttle3 * 1000, 0, 1000, PWM_MIN, PWM_MAX);
    pwmMotor4 = map(throttle4 * 1000, 0, 1000, PWM_MIN, PWM_MAX);
}

// ==========================================
void aplicarPWM() {
    ledcWrite(0, pwmMotor1);
    ledcWrite(1, pwmMotor2);
    ledcWrite(2, pwmMotor3);
    ledcWrite(3, pwmMotor4);
}

// ==========================================
void detenerMotores() {
    // Cuando se desarma, apagamos motores (PWM_MIN)
    pwmMotor1 = PWM_MIN;
    pwmMotor2 = PWM_MIN;
    pwmMotor3 = PWM_MIN;
    pwmMotor4 = PWM_MIN;
    aplicarPWM();
}