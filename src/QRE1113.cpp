// Dependência: QTRSensors by Pololu v4.0.0
// Repositório oficial: https://github.com/pololu/qtr-sensors-arduino

#include <QTRSensors.h>
#include <Arduino.h>
#include "QRE1113.h"
#include "Pinagem.h"
#include "Configuracoes.h"

QTRSensors moduloSensores;
const uint8_t NUM_SENSORES = 8;
const uint8_t TEMPO_CALIBRAGEM_S = 10;
uint16_t valorDosSensores[NUM_SENSORES];

uint16_t iteracoesCalibragem() {
  const double TEMPO_ADC_S = 40e-6;
  const uint8_t AMOSTRAS = 4;
  const uint8_t LEITURAS = 10;
  const double TEMPO_POR_ITERACAO_S = TEMPO_ADC_S * AMOSTRAS * NUM_SENSORES * LEITURAS;

  if (TEMPO_CALIBRAGEM_S == 0) {
    return 1;
  } else {
    return static_cast<uint16_t>(TEMPO_CALIBRAGEM_S / TEMPO_POR_ITERACAO_S);
  }
}

void configurarModuloQRE() {
  moduloSensores.setTypeAnalog();
  moduloSensores.setSensorPins((const uint8_t[]){ PIN_D8, PIN_D7, PIN_D6, PIN_D5, PIN_D4, PIN_D3, PIN_D2, PIN_D1 }, NUM_SENSORES);

  delay(500);

  //digitalWrite(PIN_LED_ESP32, HIGH);

  for (uint16_t i = 0; i < iteracoesCalibragem(); i++) {
    moduloSensores.calibrate();
  }

  //digitalWrite(PIN_LED_ESP32, LOW);

  Serial.println("Valores mínimos e máximos medidos durante a calibragem:");

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    Serial.print(moduloSensores.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  for (uint8_t i = 0; i < NUM_SENSORES; i++) {
    Serial.print(moduloSensores.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
}

uint16_t posicaoFaixaBranca() {
  return moduloSensores.readLineWhite(valorDosSensores);
}

void valorSensoresQRE() {
  if (DEBUG_QRE_ATIVADO) {
    uint16_t posicao = posicaoFaixaBranca();
    uint16_t contador = 0;

    for (uint8_t i = 0; i < NUM_SENSORES; i++) {
      if (valorDosSensores[i] < 900) {
        if (i == 0) {
          contador = 0;
        }
        contador++;
      }
      Serial.print(valorDosSensores[i]);
      Serial.print('\t');

      if (contador == 8) {
        pararCarro = true;
      }
    }
    Serial.println(posicao);
  }
}

void delayAntesDoStart() {

  long inicio = millis();
  long agora = 0;
  uint16_t tempoAntesDoStart = 5000;

  while ((agora - inicio) < tempoAntesDoStart) {
    agora = millis();
    digitalWrite(PIN_LED_ESP32, LOW);
    delay(250);
    digitalWrite(PIN_LED_ESP32, HIGH);
    delay(250);
  }

  digitalWrite(PIN_LED_ESP32, LOW);
}
