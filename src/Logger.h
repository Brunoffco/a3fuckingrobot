#pragma once
#include <Arduino.h>
#include <vector>
#include "PID.h"
#include "Motores.h"

struct LogEntry {
    unsigned long tempo;
    float rpm;
    float velKmh;
    int erro;
    float correcaoPID;
    int pwmEsq;
    int pwmDir;
    float kp_term;
    float ki_term;
    float kd_term;
    uint16_t posicaoSensor;
};

class Logger {
private:
    std::vector<LogEntry> logs;
    unsigned long lastLog = 0;
    unsigned long startTime = 0;
    float distancia = 0.0f;
    
    // Variáveis para normalização ML
    int minErro = 2147483647;
    int maxErro = -2147483648;
    int minPWM = 2147483647;
    int maxPWM = -2147483648;
    float minPos = 7000.0f;
    float maxPos = 0.0f;

public:
    void begin();
    void log(int erro, float correcao, int pwmE, int pwmD);
    void log(int erro, float correcao, int pwmE, int pwmD, float kp, float ki_term, float kd_term, float rpmVal);
    void logComPosicao(int erro, float correcao, int pwmE, int pwmD, float kp, float ki, float kd, uint16_t posicao);
    void endRun();
    void printStats();
    void saveToSerialCSV();
    void reset();
    
    // Novos métodos para telemetria
    String getCSVData();
    String getMLJSON();
    
    float getVelMedia();
    float getTempoTotal();
};

extern Logger logger;