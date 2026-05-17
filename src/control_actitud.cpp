#include "control_actitud.h"
#include "sensores.h"         // Para angRoll, angPitch, angYaw
#include "config.h"           // Por si quieres constantes PID definidas allí

// Ganancias PID (ajustables experimentalmente)
#define PID_ROLL_KP  0.8
#define PID_ROLL_KI  0.02
#define PID_ROLL_KD  0.3

#define PID_PITCH_KP 0.8
#define PID_PITCH_KI 0.02
#define PID_PITCH_KD 0.3

#define PID_YAW_KP   1.5
#define PID_YAW_KI   0.01
#define PID_YAW_KD   0.2

// Setpoints por defecto (nivelado, sin giro)
float setpointRoll = 0.0;
float setpointPitch = 0.0;
float setpointYaw = 0.0;

// Correcciones de salida
float roll_corr = 0.0;
float pitch_corr = 0.0;
float yaw_corr = 0.0;

// Variables internas del PID
static float integralRoll = 0, prevErrorRoll = 0;
static float integralPitch = 0, prevErrorPitch = 0;
static float integralYaw = 0, prevErrorYaw = 0;

void initControlActitud() {
    integralRoll = integralPitch = integralYaw = 0;
    prevErrorRoll = prevErrorPitch = prevErrorYaw = 0;
    roll_corr = pitch_corr = yaw_corr = 0;
}

void updateControlActitud(float dt) {
    if (dt <= 0) dt = 0.01;

    // Roll
    float errorRoll = setpointRoll - angRoll;
    integralRoll += errorRoll * dt;
    integralRoll = constrain(integralRoll, -2.0, 2.0); // anti‑windup
    float derivativeRoll = (errorRoll - prevErrorRoll) / dt;
    roll_corr = PID_ROLL_KP * errorRoll + PID_ROLL_KI * integralRoll + PID_ROLL_KD * derivativeRoll;
    roll_corr = constrain(roll_corr, -0.3, 0.3); // no más del 30% de corrección
    prevErrorRoll = errorRoll;

    // Pitch
    float errorPitch = setpointPitch - angPitch;
    integralPitch += errorPitch * dt;
    integralPitch = constrain(integralPitch, -2.0, 2.0);
    float derivativePitch = (errorPitch - prevErrorPitch) / dt;
    pitch_corr = PID_PITCH_KP * errorPitch + PID_PITCH_KI * integralPitch + PID_PITCH_KD * derivativePitch;
    pitch_corr = constrain(pitch_corr, -0.3, 0.3);
    prevErrorPitch = errorPitch;

    // Yaw
    float errorYaw = setpointYaw - angYaw;
    // Ajuste de error circular para evitar saltos bruscos (opcional)
    if (errorYaw > 180) errorYaw -= 360;
    if (errorYaw < -180) errorYaw += 360;
    integralYaw += errorYaw * dt;
    integralYaw = constrain(integralYaw, -5.0, 5.0);
    float derivativeYaw = (errorYaw - prevErrorYaw) / dt;
    yaw_corr = PID_YAW_KP * errorYaw + PID_YAW_KI * integralYaw + PID_YAW_KD * derivativeYaw;
    yaw_corr = constrain(yaw_corr, -0.3, 0.3);
    prevErrorYaw = errorYaw;
}