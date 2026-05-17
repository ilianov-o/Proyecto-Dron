#ifndef CONTROL_FUZZY_H
#define CONTROL_FUZZY_H

#include <Arduino.h>

// Inicializa el controlador (no necesita parámetros externos)
void initFuzzy();

// Calcula la corrección de throttle (entre -0.3 y +0.3) a partir del error actual y dt
float controlFuzzyAltitud(float error, float dt);

#endif