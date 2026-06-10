#ifndef PINAGEM_H
#define PINAGEM_H

#include <Arduino.h>
#include "Configuracoes.h"
#include "Motores.h"

const uint8_t PIN_CONFIG_MODE = 41;
const uint8_t PIN_MENU_DIREITA = 42;
const uint8_t PIN_MENU_ESQUERDA = 47;
const uint8_t PIN_MENU_INCREMENTO = 38;
const uint8_t PIN_MENU_DECREMENTO = 21;

const uint8_t PIN_LED_ESP32 = 44;

// Motor da esquerda
const uint8_t PIN_DRV_PWMA = 11;
const uint8_t PIN_DRV_AIN1 = 13;
const uint8_t PIN_DRV_AIN2 = 14;
const uint8_t PIN_ENCODER_C1_E = 39;
const uint8_t PIN_ENCODER_C2_E = 40;

// Motor da direita
const uint8_t PIN_DRV_PWMB = 12;
const uint8_t PIN_DRV_BIN1 = 15;
const uint8_t PIN_DRV_BIN2 = 16;
const uint8_t PIN_ENCODER_C1_D = 17;
const uint8_t PIN_ENCODER_C2_D = 18;

const uint8_t PIN_D1 = 1;
const uint8_t PIN_D2 = 2;
const uint8_t PIN_D3 = 10;
const uint8_t PIN_D4 = 9;
const uint8_t PIN_D5 = 3;
const uint8_t PIN_D6 = 8;
const uint8_t PIN_D7 = 7;
const uint8_t PIN_D8 = 6;

const uint8_t SDA_PIN = 4;
const uint8_t SCL_PIN = 5;

inline void configurarPinosModoNormal() {
  pinMode(PIN_LED_ESP32, OUTPUT);

  pinMode(PIN_DRV_PWMA, OUTPUT);
  pinMode(PIN_DRV_PWMB, OUTPUT);
  pinMode(PIN_DRV_AIN1, OUTPUT);
  pinMode(PIN_DRV_AIN2, OUTPUT);
  pinMode(PIN_DRV_BIN1, OUTPUT);
  pinMode(PIN_DRV_BIN2, OUTPUT);

  ledcAttach(PIN_DRV_PWMA, FREQUENCIA_HZ, RESOLUCAO);
  ledcAttach(PIN_DRV_PWMB, FREQUENCIA_HZ, RESOLUCAO);

  pinMode(PIN_ENCODER_C1_D, INPUT);
  pinMode(PIN_ENCODER_C2_D, INPUT);
  pinMode(PIN_ENCODER_C1_E, INPUT);
  pinMode(PIN_ENCODER_C2_E, INPUT);

  lerVariacaoEncoder();
}

inline void configurarPinosModoConfig() {
  pinMode(PIN_MENU_DIREITA, INPUT_PULLUP);
  pinMode(PIN_MENU_ESQUERDA, INPUT_PULLUP);
  pinMode(PIN_MENU_INCREMENTO, INPUT_PULLUP);
  pinMode(PIN_MENU_DECREMENTO, INPUT_PULLUP);
}

#endif
