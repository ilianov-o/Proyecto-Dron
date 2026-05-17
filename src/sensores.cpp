#include "sensores.h"

MPU6500_WE mpu(MPU_ADDR);
MS5611 ms5611(MS5611_ADDR);
Adafruit_VL53L0X vl53 = Adafruit_VL53L0X();   // Objeto VL53L0X

// Variables globales
bool usarLiDAR = false;
float angRoll = 0, angPitch = 0, angYaw = 0;
float alturaControl = 0, alturaLiDAR = 0, alturaBaro = 0;
float ax_g = 0, ay_g = 0, az_g = 0;           // <-- añadir
// bool usarLiDAR = false;                    // <-- NOTA: Estaba duplicada, la he comentado

static float baroOffset = 0;
static unsigned long tPrevIMU = 0;

// ==========================================

bool iniciarSensores() {
    // MPU-6500
    if (!mpu.init()) {
        Serial.println("[ERROR] MPU-6500 no detectado");
        return false;
    }
    mpu.autoOffsets();  
    mpu.setAccRange(MPU6500_ACC_RANGE_8G);
    mpu.setGyrRange(MPU6500_GYRO_RANGE_500);
    mpu.setGyrDLPF(MPU6500_DLPF_3);
    Serial.println("[OK] MPU-6500 inicializado");

    // MS5611
    if (!ms5611.begin()) {
        Serial.println("[ERROR] MS5611 no detectado");
        return false;
    }
    
    // Calibración robusta del barómetro (promedio de 50 muestras)
    const int nCalib = 50;
    float sumBaro = 0;
    for (int i = 0; i < nCalib; i++) {
        ms5611.read();
        sumBaro += ms5611.getAltitude();
        delay(10);
    }
    baroOffset = sumBaro / nCalib;
    Serial.printf("[OK] MS5611 calibrado. Offset promedio: %.2f m\n", baroOffset);

    // VL53L0X
    if (!vl53.begin(VL53L0X_ADDR, false, &Wire)) {
        Serial.println("[ERROR] VL53L0X no detectado");
        return false;
    }
    Serial.println("[OK] VL53L0X inicializado");

    tPrevIMU = millis();
    return true;
}

// ==========================================

void actualizarActitud() {
    xyzFloat acc = mpu.getGValues();
    xyzFloat gyr = mpu.getGyrValues();

    // Guardar aceleraciones (en g) para telemetría
    ax_g = acc.x;
    ay_g = acc.y;
    az_g = acc.z;

    float dt = (millis() - tPrevIMU) / 1000.0f;
    tPrevIMU = millis();
    if (dt <= 0) dt = 0.01f;

    float accRoll = atan2(acc.y, acc.z) * 180.0f / PI;
    float accPitch = atan2(-acc.x, sqrt(acc.y * acc.y + acc.z * acc.z)) * 180.0f / PI;

    angRoll = FILTRO_COMP_ALPHA * (angRoll + gyr.x * dt) + (1 - FILTRO_COMP_ALPHA) * accRoll;
    angPitch = FILTRO_COMP_ALPHA * (angPitch + gyr.y * dt) + (1 - FILTRO_COMP_ALPHA) * accPitch;
    angYaw += gyr.z * dt;
}

// ==========================================

void actualizarAltitud() {
    // VL53L0X
    VL53L0X_RangingMeasurementData_t measure;
    vl53.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {   // 4 = fuera de rango
        alturaLiDAR = measure.RangeMilliMeter / 1000.0f;
        usarLiDAR = (alturaLiDAR > 0.02f && alturaLiDAR < 3.5f);
    } else {
        usarLiDAR = false;
    }

    // MS5611
    // Lectura del barómetro con filtro exponencial
    ms5611.read();
    float baroCrudo = ms5611.getAltitude() - baroOffset;
    static float alturaBaroFiltrada = baroCrudo; // inicializar la primera vez
    
    alturaBaroFiltrada = 0.9 * alturaBaroFiltrada + 0.1 * baroCrudo;
    alturaBaro = alturaBaroFiltrada;

    // Fusión de sensores
    if (usarLiDAR && alturaLiDAR < ALTURA_MAX_US) {
        if (alturaLiDAR > ALTURA_MIN_BARO) {
            float alpha = (alturaLiDAR - ALTURA_MIN_BARO) / (ALTURA_MAX_US - ALTURA_MIN_BARO);
            alturaControl = (1 - alpha) * alturaLiDAR + alpha * alturaBaro;
        } else {
            alturaControl = alturaLiDAR;
        }
    } else {
        alturaControl = alturaBaro;
        usarLiDAR = false;
    }
}