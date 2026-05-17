#include "control_vuelo.h"
#include "control_fuzzy.h"
#include "sensores.h"

float alturaObjetivo = ALTURA_OBJETIVO_DEF;
bool setpointManual = false;
float salidaPID = 0.0f;   // Mantenemos el nombre por compatibilidad con telemetría

void controlAltitud() {
    static unsigned long tPrevAlt = 0;
    float dt = (millis() - tPrevAlt) / 1000.0f;
    tPrevAlt = millis();
    if (dt <= 0.0f) dt = 0.01f;

    float error = alturaObjetivo - alturaControl;
    salidaPID = controlFuzzyAltitud(error, dt);
}

/*
// Esto era para un PID (Pero no hay planta modelada)
void controlAltitud() {
    float dt = (millis() - tPrevAlt) / 1000.0f;
    tPrevAlt = millis();
    if (dt <= 0) dt = 0.01f;

    float errorAlt = alturaObjetivo - alturaControl;
    integralAlt += errorAlt * dt;
    integralAlt = constrain(integralAlt, -PID_ALT_IMAX, PID_ALT_IMAX);
    float derivadaAlt = (errorAlt - errorAltPrev) / dt;
    errorAltPrev = errorAlt;

    salidaPID = PID_ALT_KP * errorAlt + PID_ALT_KI * integralAlt + PID_ALT_KD * derivadaAlt;
    salidaPID = constrain(salidaPID, -0.3f, 0.3f);
}
*/