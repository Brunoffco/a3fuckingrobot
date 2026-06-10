#ifndef MOTORES_H
#define MOTORES_H

void controleMotores(uint8_t codigoDeDirecao, uint8_t velocidadeDireita, uint8_t velocidadeEsquerda);
void calcularRPM();
void printarRPM();
void lerVariacaoEncoder();

#endif
