#ifndef SENSORES_H
#define SENSORES_H

#include <Wire.h>
#include <MPU6500_WE.h>
#include <MS5611.h>
#include <Adafruit_VL53L0X.h>
#include "config.h"

// Variables globales de sensores
extern float angRoll, angPitch, angYaw;
extern float alturaControl, alturaLiDAR, alturaBaro;
extern bool usarLiDAR;
extern float angRoll, angPitch, angYaw;
extern float alturaControl, alturaLiDAR, alturaBaro;
extern float ax_g, ay_g, az_g;   // Aceleración en g
extern bool usarLiDAR;
bool iniciarSensores();
void actualizarActitud();
void actualizarAltitud();

#endif