#ifndef LSM6DS3_H
#define LSM6DS3_H

// Funções
void lerSensorLSM6DS3();
void inicializarLSM6DS3();

// Dados do sensor (atualizados a cada leitura)
extern float acelX, acelY, acelZ;
extern float giroX, giroY, giroZ;
extern bool sensorInicializado;

#endif