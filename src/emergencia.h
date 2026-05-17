#ifndef EMERGENCIA_H
#define EMERGENCIA_H

#include "config.h"

// Estados de vuelo
enum ModoVuelo {
    MODO_DESARMADO,
    MODO_ARMADO,
    MODO_ATERRIZAJE_EMERGENCIA,
    MODO_EMERGENCIA_DETENIDO
};

extern ModoVuelo modoVuelo;

void verificarBotonEmergencia();
void verificarTimeoutSignal();
void ejecutarAterrizajeEmergencia();

#endif