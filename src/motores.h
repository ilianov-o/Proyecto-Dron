#ifndef MOTORES_H
#define MOTORES_H

#include <Arduino.h>
#include "config.h"

extern uint16_t pwmMotor1, pwmMotor2, pwmMotor3, pwmMotor4;

void iniciarMotores();
void mezclarMotores();
void aplicarPWM();
void detenerMotores();

#endif