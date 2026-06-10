#ifndef ODOMETRIA_H
#define ODOMETRIA_H

#include <Arduino.h>

// Inicializa sistema de odometria
void inicializarOdometria();

// Atualiza posição (deve ser chamado no loop principal)
void atualizarOdometria(int pwmEsquerda, int pwmDireita, float giroZ);

// Obtém posição atual em cm
void obterPosicao(float &x, float &y);

// Obtém heading atual em graus
float obterHeading();

// Reset da posição
void resetarOdometria();

#endif
