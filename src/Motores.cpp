#include <Arduino.h>
#include "Motores.h"
#include "Pinagem.h"

long valorEncoderMotorDireito = 0;
long valorEncoderMotorEsquerdo = 0;
double rpmMotorDireito = 0.0;
double rpmMotorEsquerdo = 0.0;
unsigned long ultimoTempo = 0;
const double PULSOS_POR_VOLTA = 200.0;

void atualizarEncoderMotorDireito() {
  if (digitalRead(PIN_ENCODER_C1_D) > digitalRead(PIN_ENCODER_C2_D)) {
    valorEncoderMotorDireito++;
  } else {
    valorEncoderMotorDireito--;
  }
}

void atualizarEncoderMotorEsquerdo() {
  if (digitalRead(PIN_ENCODER_C1_E) > digitalRead(PIN_ENCODER_C2_E)) {
    valorEncoderMotorEsquerdo++;
  } else {
    valorEncoderMotorEsquerdo--;
  }
}

void lerVariacaoEncoder() {
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_C1_D), atualizarEncoderMotorDireito, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODER_C1_E), atualizarEncoderMotorEsquerdo, RISING);
}

void calcularRPM() {
  unsigned long tempoAtual = millis();
  unsigned long deltaTempo = tempoAtual - ultimoTempo;
  double minutos = deltaTempo / 60000.0;

  if (deltaTempo == 0) {
    rpmMotorDireito = 0.0;
    rpmMotorEsquerdo = 0.0;
  }

  double rotacoesDireita = abs(valorEncoderMotorDireito) / PULSOS_POR_VOLTA;
  double rotacoesEsquerda = abs(valorEncoderMotorEsquerdo) / PULSOS_POR_VOLTA;

  double rpmDireita = rotacoesDireita / minutos;
  double rpmEsquerda = rotacoesEsquerda / minutos;

  valorEncoderMotorDireito = 0;
  valorEncoderMotorEsquerdo = 0;

  ultimoTempo = tempoAtual;

  rpmMotorDireito = rpmDireita;
  rpmMotorEsquerdo = rpmEsquerda;
}

void printarRPM() {

  short INTERVALO_DE_EXIBICAO_MS = 0;

  static unsigned long ultimoPrint = 0;

  if (millis() - ultimoPrint >= INTERVALO_DE_EXIBICAO_MS) {
    calcularRPM();
    Serial.print("RPM Motor Direito: ");
    Serial.print(rpmMotorDireito);
    Serial.print(" | RPM Motor Esuerdo: ");
    Serial.println(rpmMotorEsquerdo);
    ultimoPrint = millis();
  }
}

void controleMotores(uint8_t codigoDeDirecao, uint8_t velocidadeDireita, uint8_t velocidadeEsquerda) {

  //digitalWrite(PIN_DRV_STBY, HIGH);

  if (codigoDeDirecao == 1) {
    // Para frente
    digitalWrite(PIN_DRV_AIN1, HIGH);
    digitalWrite(PIN_DRV_AIN2, LOW);
    digitalWrite(PIN_DRV_BIN1, HIGH);
    digitalWrite(PIN_DRV_BIN2, LOW);
  } else if (codigoDeDirecao == 2) {
    // Para trás
    digitalWrite(PIN_DRV_AIN1, LOW);
    digitalWrite(PIN_DRV_AIN2, HIGH);
    digitalWrite(PIN_DRV_BIN1, LOW);
    digitalWrite(PIN_DRV_BIN2, HIGH);
  } else if (codigoDeDirecao == 3) {
    // Para a direita
    digitalWrite(PIN_DRV_AIN1, HIGH);
    digitalWrite(PIN_DRV_AIN2, LOW);
    digitalWrite(PIN_DRV_BIN1, LOW);
    digitalWrite(PIN_DRV_BIN2, HIGH);
  } else if (codigoDeDirecao == 4) {
    // Para a esquerda
    digitalWrite(PIN_DRV_AIN1, LOW);
    digitalWrite(PIN_DRV_AIN2, HIGH);
    digitalWrite(PIN_DRV_BIN1, HIGH);
    digitalWrite(PIN_DRV_BIN2, LOW);
  } else {
    // Parado
    digitalWrite(PIN_DRV_AIN1, HIGH);
    digitalWrite(PIN_DRV_BIN1, HIGH);
    digitalWrite(PIN_DRV_AIN2, HIGH);
    digitalWrite(PIN_DRV_BIN2, HIGH);
  }

  ledcWrite(PIN_DRV_PWMA, velocidadeEsquerda);
  ledcWrite(PIN_DRV_PWMB, velocidadeDireita);
}
