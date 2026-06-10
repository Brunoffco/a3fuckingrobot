#ifndef TELEMETRIA_WIFI_H
#define TELEMETRIA_WIFI_H

#include <Arduino.h>

// Inicializa WiFi AP e endpoints de telemetria
void iniciarTelemetriaWiFi();

// Processa requisições do servidor
void processarTelemetriaWiFi();

// Envia ponto de dados JSON para clientes (com dados do acelerômetro e coordenadas XY)
void enviarDadosTelemetria(int erro, float correcao, int pwmE, int pwmD, 
                           float kp, float ki, float kd, uint16_t posicaoSensor,
                           float ax, float ay, float az, float gx, float gy, float gz,
                           float x, float y);

// Informa se há clientes conectados
bool temClientesConectados();

#endif
