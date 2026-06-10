#include <Arduino.h>
#include "Odometria.h"
#include "Motores.h"

// Parâmetros do robô (ajuste conforme necessário)
const float DIAMETRO_RODA_CM = 2.2;  // Diâmetro da roda em cm
const float CIRCUNFERENCIA_CM = PI * DIAMETRO_RODA_CM;
const float PULSOS_POR_VOLTA = 200.0;
const float DISTANCIA_ENTRE_RODAS_CM = 115.0;  // Distância entre os eixos das rodas (mm)
const float CM_POR_PULSO = CIRCUNFERENCIA_CM / PULSOS_POR_VOLTA;

// Variáveis globais de posição
static float posX = 0.0;
static float posY = 0.0;
static float heading = 0.0;  // em graus (0-360)

// Controle de tempo para integração
static unsigned long ultimoTempo = 0;

// Estado anterior dos motores para detectar movimento
static int pwmEAnterior = 0;
static int pwmDAnterior = 0;

void inicializarOdometria() {
  posX = 0.0;
  posY = 0.0;
  heading = 0.0;
  ultimoTempo = millis();
  pwmEAnterior = 0;
  pwmDAnterior = 0;
  
  Serial.println("[ODOMETRIA] Sistema inicializado");
}

void atualizarOdometria(int pwmEsquerda, int pwmDireita, float giroZ) {
  unsigned long tempoAtual = millis();
  float deltaTempo = (tempoAtual - ultimoTempo) / 1000.0;  // em segundos
  ultimoTempo = tempoAtual;
  
  if (deltaTempo <= 0 || deltaTempo > 1.0) {
    return;  // Ignorar valores inválidos ou grandes gaps
  }
  
  // Calcular velocidades em cm/s baseado no PWM (aproximação linear)
  // PWM 255 = velocidade máxima. Ajuste este valor se a posição ficar muito errada
  // Dica: Comece com 20 cm/s, depois aumentar se necessário para ~30 cm/s
  const float MAX_VELOCITY_CM_S = 20.0;
  float velEsqCMperS = (pwmEsquerda / 255.0) * MAX_VELOCITY_CM_S;
  float velDirCMperS = (pwmDireita / 255.0) * MAX_VELOCITY_CM_S;
  
  // Distâncias percorridas neste intervalo
  float distE = velEsqCMperS * deltaTempo;
  float distD = velDirCMperS * deltaTempo;
  
  // Distância média percorrida (movimento para frente)
  float distMedia = (distE + distD) / 2.0;
  
  // Diferença de distância entre rodas (para calcular rotação)
  float deltaDist = distD - distE;
  
  // Atualizar heading usando a diferença de distâncias
  // Fórmula: delta_theta = delta_distance / distance_between_wheels
  if (DISTANCIA_ENTRE_RODAS_CM > 0) {
    float deltaHeading = (deltaDist / DISTANCIA_ENTRE_RODAS_CM) * (180.0 / PI);  // Convert rad to deg
    heading += deltaHeading;
    
    // Normalizar heading para 0-360
    while (heading >= 360.0) heading -= 360.0;
    while (heading < 0.0) heading += 360.0;
  }
  
  // Integrar movimento em X,Y usando heading atual
  float headingRad = heading * (PI / 180.0);
  posX += distMedia * cos(headingRad);
  posY += distMedia * sin(headingRad);
  
  // Debug output a cada 500ms
  static unsigned long ultimoDebug = 0;
  if (millis() - ultimoDebug >= 500) {
    Serial.printf("[ODO] X:%.2f cm Y:%.2f cm Heading:%.1f° PWM-E:%d PWM-D:%d\n",
                  posX, posY, heading, pwmEsquerda, pwmDireita);
    ultimoDebug = millis();
  }
}

void obterPosicao(float &x, float &y) {
  x = posX;
  y = posY;
}

float obterHeading() {
  return heading;
}

void resetarOdometria() {
  posX = 0.0;
  posY = 0.0;
  heading = 0.0;
  ultimoTempo = millis();
  
  Serial.println("[ODOMETRIA] Posição resetada");
}
