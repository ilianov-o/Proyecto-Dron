#ifndef WIFI_TELEMETRIA_H
#define WIFI_TELEMETRIA_H

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <WebServer.h>

extern WebSocketsServer webSocket;
extern WebServer server;

void iniciarWiFi();
void enviarTelemetria();
bool clienteConectado();
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length);
void handleRoot();

#endif