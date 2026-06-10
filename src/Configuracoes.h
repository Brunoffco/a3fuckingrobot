#ifndef CONFIGURACOES_H
#define CONFIGURACOES_H

#include <Arduino.h>

// Motores
const uint16_t FREQUENCIA_HZ = 20000;
const uint8_t RESOLUCAO = 8;

// Valores padrão para o PWM
extern uint8_t DUTY_CYCLE_MIN;
extern uint8_t DUTY_CYCLE_MAX;
extern uint8_t DUTY_CYCLE_BASE;

// Variáveis do PID
extern float KP;
extern float KI;
extern float KD;

// Usado no Menu.cpp
const unsigned long INTERVALO_DEBOUNCE_MS = 200;
const float PASSO_INCREMENTO_KP = 0.005f;
const float PASSO_INCREMENTO_KI = 0.001f;
const float PASSO_DECREMENTO_KD = 0.05f;

// Para contador de tempo de percurso
extern long inicioDoPercurso;
extern long tempoTotalDePercurso;
extern bool tempoDePercursoContabilizado;

// Outros
extern uint16_t DELAY_LOOP_MS;
extern bool DEBUG_PID_ATIVADO;
extern bool DEBUG_QRE_ATIVADO;
extern bool pararCarro;
extern bool modoDeConfiguracao;
extern uint8_t menuAtual;

void configuracoesSalvas();
void salvarConfiguracoes();

#endif
