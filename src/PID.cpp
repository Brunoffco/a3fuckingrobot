#include <Arduino.h>
#include "PID.h"
#include "Motores.h"
#include "QRE1113.h"
#include "Configuracoes.h"
#include "TelemetriaWiFi.h"
#include "LSM6DS3.h"
#include "Odometria.h"

int16_t erroAnterior = 0;
int32_t integral = 0;

int32_t calcularPID() {
  uint16_t posicaoAtual = posicaoFaixaBranca();

  uint16_t centroDoSensor = 3500;

  int16_t erro = posicaoAtual - centroDoSensor;

  int16_t P = erro * KP;

  integral += erro;

  int32_t I = KI * integral;

  int16_t D = KD * (erro - erroAnterior);

  erroAnterior = erro;

  int32_t saidaPID = P + I + D;
  saidaPID = constrain(saidaPID, -255, 255);
  return saidaPID;
}

void seguirLinha() {
  uint16_t posicaoAtual = posicaoFaixaBranca();
  uint16_t centroDoSensor = 3500;
  int16_t erro = posicaoAtual - centroDoSensor;
  
  int32_t correcao = calcularPID();

  correcao = constrain(correcao, -150, 150);

  int16_t velocidadeDireita = DUTY_CYCLE_BASE - correcao;
  int16_t velocidadeEsquerda = DUTY_CYCLE_BASE + correcao;

  velocidadeDireita = constrain(velocidadeDireita, DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);
  velocidadeEsquerda = constrain(velocidadeEsquerda, DUTY_CYCLE_MIN, DUTY_CYCLE_MAX);

  controleMotores(1, velocidadeDireita, velocidadeEsquerda);

  // Atualizar odometria
  atualizarOdometria(velocidadeEsquerda, velocidadeDireita, giroZ);
  
  // Obter posição atual
  float posX, posY;
  obterPosicao(posX, posY);

  if (DEBUG_PID_ATIVADO) {
    Serial.print("Correção: ");
    Serial.print(correcao);
    Serial.print(" | PWM da Direita: ");
    Serial.print(velocidadeDireita);
    Serial.print(" | PWM da Esquerda: ");
    Serial.print(velocidadeEsquerda);
    Serial.print(" | P: ");
    Serial.print(KP, 3);
    Serial.print(" | I: ");
    Serial.print(KI, 3);
    Serial.print(" | D: ");
    Serial.println(KD, 3);
  }

  // Envia telemetria via WiFi com dados do acelerômetro e posição XY
  #ifdef INCLUDE_WIFI
  enviarDadosTelemetria(erro, correcao, velocidadeEsquerda, velocidadeDireita,
                        KP, KI, KD, posicaoAtual, acelX, acelY, acelZ, giroX, giroY, giroZ, posX, posY);
  #endif
}