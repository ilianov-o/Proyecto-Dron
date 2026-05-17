#ifndef CONTROL_ACTITUD_H
#define CONTROL_ACTITUD_H

#include <Arduino.h>

// Inicializa los PID de actitud (resetea integrales, etc.)
void initControlActitud();

// Calcula las correcciones de roll, pitch, yaw.
// Debe llamarse en cada iteración del lazo de control.
void updateControlActitud(float dt);

// Variables de salida (correcciones en el rango -1..1)
extern float roll_corr;
extern float pitch_corr;
extern float yaw_corr;

// Setpoints (ángulos deseados en grados)
extern float setpointRoll;
extern float setpointPitch;
extern float setpointYaw;

#endif