#include <Arduino.h>
#include "Menus.h"
#include "Display.h"
#include "Configuracoes.h"
#include "Pinagem.h"

void menuTempoDePercurso() {
  if (inicioDoPercurso == 0L) {
    inicioDoPercurso = millis();
  }

  if (pararCarro) {
    if (!tempoDePercursoContabilizado) {
      tempoTotalDePercurso = millis() - inicioDoPercurso;
      tempoDePercursoContabilizado = true;
    }
    double tempoTotalDePercursoEmSegundos = tempoTotalDePercurso / 1000.0;
    char tempo[20];
    snprintf(tempo, sizeof(tempo), "%.3fs", tempoTotalDePercursoEmSegundos);
    display("Tempo", tempo, false, false);
  }
}

void atualizarMenuAtual(bool direita, bool esquerda) {
  static unsigned long ultimaTroca = 0;

  if (millis() - ultimaTroca < INTERVALO_DEBOUNCE_MS) {
    return;
  }

  if (direita && menuAtual < 9) {
    menuAtual++;
    ultimaTroca = millis();
  } else if (esquerda && menuAtual > 1) {
    menuAtual--;
    ultimaTroca = millis();
  }
}

void exibirValorAtual() {
  char buffer[6];
  switch (menuAtual) {
    case 1:
      snprintf(buffer, sizeof(buffer), "%.3f", KP);
      display("P", buffer, false, true);
      break;
    case 2:
      snprintf(buffer, sizeof(buffer), "%.3f", KI);
      display("I", buffer, true, true);
      break;
    case 3:
      snprintf(buffer, sizeof(buffer), "%.3f", KD);
      display("D", buffer, true, true);
      break;
    case 4:
      snprintf(buffer, sizeof(buffer), "%d", DUTY_CYCLE_MIN);
      display("DC MIN", buffer, true, true);
      break;
    case 5:
      snprintf(buffer, sizeof(buffer), "%d", DUTY_CYCLE_MAX);
      display("DC MAX", buffer, true, true);
      break;
    case 6:
      snprintf(buffer, sizeof(buffer), "%d", DUTY_CYCLE_BASE);
      display("DC BASE", buffer, true, true);
      break;
    case 7:
      snprintf(buffer, sizeof(buffer), "%d", DELAY_LOOP_MS);
      display("DELAY", buffer, true, true);
      break;
    case 8:
      snprintf(buffer, sizeof(buffer), "%d", DEBUG_PID_ATIVADO);
      display("DBG PID", buffer, true, true);
      break;
    case 9:
      snprintf(buffer, sizeof(buffer), "%d", DEBUG_QRE_ATIVADO);
      display("DBG QRE", buffer, true, false);
      break;
  }
}

void ajustarParametro(bool incremento, bool decremento) {
  static unsigned long ultimaTroca = 0;

  if (millis() - ultimaTroca < INTERVALO_DEBOUNCE_MS) {
    return;
  }

  if (incremento) {
    switch (menuAtual) {
      case 1:
        KP += PASSO_INCREMENTO_KP;
        break;
      case 2:
        KI += PASSO_INCREMENTO_KI;
        break;
      case 3:
        KD += PASSO_DECREMENTO_KD;
        break;
      case 4:
        if (DUTY_CYCLE_MIN < 255) DUTY_CYCLE_MIN += 1;
        break;
      case 5:
        if (DUTY_CYCLE_MAX < 255) DUTY_CYCLE_MAX += 1;
        break;
      case 6:
        if (DUTY_CYCLE_BASE < 255) DUTY_CYCLE_BASE += 1;
        break;
      case 7:
        if (DELAY_LOOP_MS < 1000) DELAY_LOOP_MS += 1;
        break;
      case 8:
        if (DEBUG_PID_ATIVADO == false) DEBUG_PID_ATIVADO = true;
        break;
      case 9:
        if (DEBUG_QRE_ATIVADO == false) DEBUG_QRE_ATIVADO = true;
        break;
    }
    ultimaTroca = millis();
  } else if (decremento) {
    switch (menuAtual) {
      case 1:
        KP = max(KP - PASSO_INCREMENTO_KP, 0.0f);
        break;
      case 2:
        KI = max(KI - PASSO_INCREMENTO_KI, 0.0f);
        break;
      case 3:
        KD = max(KD - PASSO_DECREMENTO_KD, 0.0f);
        break;
      case 4:
        if (DUTY_CYCLE_MIN > 0) DUTY_CYCLE_MIN -= 1;
        break;
      case 5:
        if (DUTY_CYCLE_MAX > 0) DUTY_CYCLE_MAX -= 1;
        break;
      case 6:
        if (DUTY_CYCLE_BASE > 0) DUTY_CYCLE_BASE -= 1;
        break;
      case 7:
        if (DELAY_LOOP_MS > 0) DELAY_LOOP_MS -= 1;
        break;
      case 8:
        if (DEBUG_PID_ATIVADO == true) DEBUG_PID_ATIVADO = false;
        break;
      case 9:
        if (DEBUG_QRE_ATIVADO == true) DEBUG_QRE_ATIVADO = false;
        break;
    }
    ultimaTroca = millis();
  }
}

void menuConfigurarCarro() {
  bool botaoEsquerda = (digitalRead(PIN_MENU_ESQUERDA) == LOW);
  bool botaoDireita = (digitalRead(PIN_MENU_DIREITA) == LOW);
  bool botaoIncremento = (digitalRead(PIN_MENU_INCREMENTO) == LOW);
  bool botaoDecremento = (digitalRead(PIN_MENU_DECREMENTO) == LOW);

  atualizarMenuAtual(botaoDireita, botaoEsquerda);
  exibirValorAtual();
  ajustarParametro(botaoIncremento, botaoDecremento);

  salvarConfiguracoes();
}
