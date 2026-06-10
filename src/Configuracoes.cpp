#include <Arduino.h>
#include <Preferences.h>
#include "Configuracoes.h"
#include "Pinagem.h"

Preferences preferences;

bool pararCarro = false;
bool modoDeConfiguracao = false;

long inicioDoPercurso = 0L;
long tempoTotalDePercurso = 0L;
bool tempoDePercursoContabilizado = false;

uint8_t DUTY_CYCLE_MIN;
uint8_t DUTY_CYCLE_MAX;
uint8_t DUTY_CYCLE_BASE;

float KP;
float KI;
float KD;

uint16_t DELAY_LOOP_MS;
bool DEBUG_PID_ATIVADO;
bool DEBUG_QRE_ATIVADO;

uint8_t menuAtual = 1;

void configuracoesSalvas() {
  preferences.begin("configuracao", false);
  KP = preferences.getFloat("kp", 0.02);
  KI = preferences.getFloat("ki", 0.000);
  KD = preferences.getFloat("kd", 0.1);
  DUTY_CYCLE_MIN = preferences.getInt("dc_min", 190);
  DUTY_CYCLE_MAX = preferences.getInt("dc_max", 255);
  DUTY_CYCLE_BASE = preferences.getInt("dc_base", 220);
  DELAY_LOOP_MS = preferences.getInt("delay_loop", 15);
  DEBUG_PID_ATIVADO = preferences.getBool("debug_pid", 1);
  DEBUG_QRE_ATIVADO = preferences.getBool("debug_qre", 1);
}

void salvarConfiguracoes() {
  preferences.putFloat("kp", KP);
  preferences.putFloat("ki", KI);
  preferences.putFloat("kd", KD);
  preferences.putInt("dc_min", DUTY_CYCLE_MIN);
  preferences.putInt("dc_max", DUTY_CYCLE_MAX);
  preferences.putInt("dc_base", DUTY_CYCLE_BASE);
  preferences.putInt("delay_loop", DELAY_LOOP_MS);
  preferences.putBool("debug_pid", DEBUG_PID_ATIVADO);
  preferences.putBool("debug_qre", DEBUG_QRE_ATIVADO);
}
