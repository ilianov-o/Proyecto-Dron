#include "control_fuzzy.h"

// ===================== PARÁMETROS AJUSTABLES =====================
#define ERROR_MAX_M      2.0f
#define DERROR_MAX_MS    2.0f
#define OUTPUT_MIN      -0.15f
#define OUTPUT_MAX       0.15f

// ===================== CONJUNTOS DIFUSOS =====================
enum FuzzySet { MN = 0, SN = 1, ZE = 2, SP = 3, MP = 4 };

static float trimf(float x, float a, float b, float c) {
    if (x <= a || x >= c) return 0.0f;
    if (x < b) return (x - a) / (b - a);
    return (c - x) / (c - b);
}

// Puntos para el error (metros)
static const float errA[] = { -2.0f, -1.5f, -0.5f, 0.0f,  0.5f };
static const float errB[] = { -1.5f, -0.5f,  0.0f, 0.5f,  1.5f };
static const float errC[] = { -0.5f,  0.0f,  0.5f, 1.5f,  2.0f };

// Puntos para la derivada del error (m/s)
static const float derA[] = { -2.0f, -1.5f, -0.5f, 0.0f,  0.5f };
static const float derB[] = { -1.5f, -0.5f,  0.0f, 0.5f,  1.5f };
static const float derC[] = { -0.5f,  0.0f,  0.5f, 1.5f,  2.0f };

// Conjuntos de salida (singletons)
static const float outCenters[5] = { -0.25f, -0.12f, 0.0f, 0.12f, 0.25f };

// ===================== TABLA DE REGLAS CORREGIDA =====================
//            der: MN     SN     ZE     SP     MP
// err MN      MN     MN     SN     ZE     SP   (dron muy alto → reducir throttle)
// err SN      MN     SN     ZE     SP     MP
// err ZE      SN     ZE     ZE     ZE     SP
// err SP      SN     ZE     SP     MP     MP
// err MP      ZE     SP     MP     MP     MP   (dron muy bajo → aumentar throttle)
static const FuzzySet reglas[5][5] = {
    { MN, MN, SN, ZE, SP },
    { MN, SN, ZE, SP, MP },
    { SN, ZE, ZE, ZE, SP },
    { SN, ZE, SP, MP, MP },
    { ZE, SP, MP, MP, MP }
};

// ===================== VARIABLES PERSISTENTES =====================
static float lastError = 0.0f;

// ===================== FUNCIONES =====================
void initFuzzy() {
    lastError = 0.0f;
}

static float pertenencia(float x, int set) {
    return trimf(x, errA[set], errB[set], errC[set]);  // misma forma para error y derivada
}

float controlFuzzyAltitud(float error, float dt) {
    error = constrain(error, -ERROR_MAX_M, ERROR_MAX_M);

    if (dt <= 0.0f) dt = 0.01f;
    float derror = (error - lastError) / dt;
    derror = constrain(derror, -DERROR_MAX_MS, DERROR_MAX_MS);
    lastError = error;

    float mu_err[5], mu_der[5];
    for (int i = 0; i < 5; i++) {
        mu_err[i] = pertenencia(error, i);
        mu_der[i] = pertenencia(derror, i);
    }

    float numerador = 0.0f;
    float denominador = 0.0f;

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 5; j++) {
            float fuerza = min(mu_err[i], mu_der[j]);
            if (fuerza > 0.0f) {
                int regla = reglas[i][j];
                float centro = outCenters[regla];
                numerador += fuerza * centro;
                denominador += fuerza;
            }
        }
    }

    float salida = 0.0f;
    if (denominador > 0.0f) {
        salida = numerador / denominador;
    }
    salida = constrain(salida, OUTPUT_MIN, OUTPUT_MAX);

    // === DEPURACIÓN (puedes comentar estas líneas una vez verificado) ===
    Serial.printf("Fuzzy: err=%.2f derr=%.2f sal=%.3f\n", error, derror, salida);
    // ===============================================================

    return salida;
}