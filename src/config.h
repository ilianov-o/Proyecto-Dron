#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ===================== PINES =====================
#define I2C_SDA             8
#define I2C_SCL             9

#define MOTOR1_PIN          4
#define MOTOR2_PIN          5
#define MOTOR3_PIN          6
#define MOTOR4_PIN          7

#define PIN_BOTON_EMERGENCIA 10  // Pulsador externo (activo bajo)

// ===================== DIRECCIONES I2C =====================
#define MPU_ADDR            0x68
#define MS5611_ADDR         0x77
#define VL53L0X_ADDR        0x29

// ===================== WIFI =====================
#define WIFI_SSID           "Dron_F450"
#define WIFI_PASSWORD       "dron123456789"
#define WEBSOCKET_PORT      81

// ===================== CONTROL DE VUELO =====================
#define ALTURA_MAX_US           1.5f    // metros: hasta aquí usa LiDAR
#define ALTURA_MIN_BARO         1.0f    // metros: a partir de aquí mezcla con baro
#define ALTURA_OBJETIVO_DEF     1.0f    // altura hover por defecto

#define PID_ALT_KP          15.0f
#define PID_ALT_KI          0.5f
#define PID_ALT_KD          8.0f
#define PID_ALT_IMAX        10.0f

#define FILTRO_COMP_ALPHA   0.96f

// ===================== MOTORES =====================
#define PWM_FREQ            50      // Hz
#define PWM_RESOLUTION      12      // bits (0-4095)
#define PWM_MIN             205     // ~1 ms
#define PWM_MAX             410     // ~2 ms
#define THROTTLE_HOVER      0.5f    // valor base (ajustar experimentalmente)
#define RAMP_TIME_MS  800   // Duración de la rampa de arranque (ms)

// ===================== EMERGENCIA =====================
#define TIMEOUT_SIGNAL_MS   1000    // ms sin telemetría -> aterrizaje
#define VELOCIDAD_DESCENSO  0.5f    // m/s
#define ALTURA_CORTE_MOTORES 0.10f  // metros (10 cm del suelo)

// ===================== TELEMETRÍA =====================
#define TELEMETRIA_HZ       20      // Frecuencia de envío

#endif