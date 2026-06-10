#include <Wire.h>
#include "Pinagem.h"
#include "QRE1113.h"
#include "Motores.h"
#include "Configuracoes.h"
#include "PID.h"
#include "Display.h"
#include "Menus.h"
#include "LSM6DS3.h"
#include "TelemetriaWiFi.h"
// #include "ServidorWeb.h"

// Não esqueça de upar os arquivos do servidor web
// 1 - Abra o menu de opções do PlatformIO
// 2 - Platform > Build Filesystem Image
// 3 - Platform > Upload Filesystem Image

void setup() {

  Wire.setPins(SDA_PIN, SCL_PIN);
  Wire.begin();
  pinMode(PIN_CONFIG_MODE, INPUT_PULLUP);
  modoDeConfiguracao = digitalRead(PIN_CONFIG_MODE) == LOW;
  configuracoesSalvas();

  if (modoDeConfiguracao) {
    configurarPinosModoConfig();
    iniciarDisplay();
  } else {
    Serial.begin(115200);
    configurarPinosModoNormal();
    iniciarTelemetriaWiFi();
    configurarModuloQRE();
    inicializarLSM6DS3();
    delayAntesDoStart();
  }
}

void loop() {

  if (modoDeConfiguracao) {
    menuConfigurarCarro();
  } else {
    valorSensoresQRE();
    seguirLinha();
    // menuTempoDePercurso();

    // Direção, direita, esquerda
    // controleMotores(1, 0, 150);

    // printarRPM();
    processarTelemetriaWiFi();
  }
  lerSensorLSM6DS3();
  delay(DELAY_LOOP_MS);
}