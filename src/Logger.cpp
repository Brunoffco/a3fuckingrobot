#include "Logger.h"

Logger logger;

void Logger::begin() {
    startTime = millis();
    lastLog = millis();
    logs.reserve(800);
    Serial.println("\n=== LOGGER INICIADO - Seguidor de Linha ===\n");
    Serial.println("tempo_ms,rpm,vel_kmh,erro,correcao_pid,pwm_esq,pwm_dir");
}

void Logger::log(int erro, float correcao, int pwmE, int pwmD, float kp, float ki_term, float kd_term, float rpmVal) {
    if (millis() - lastLog < 80) return;

    float deltaT = (millis() - lastLog) / 1000.0f;
    float velInstant = ((abs(pwmE) + abs(pwmD)) / 2.0f) * 0.0018f;
    distancia += velInstant * deltaT;

    LogEntry entry;
    entry.tempo = millis() - startTime;
    entry.rpm = rpmVal;
    entry.velKmh = rpmVal * 0.018;
    entry.erro = erro;
    entry.correcaoPID = correcao;
    entry.pwmEsq = pwmE;
    entry.pwmDir = pwmD;
    entry.kp_term = kp;
    entry.ki_term = ki_term;
    entry.kd_term = kd_term;
    entry.posicaoSensor = 0;

    logs.push_back(entry);
    Serial.printf("%lu,%.1f,%.2f,%d,%.1f,%d,%d\n",
        entry.tempo, entry.rpm, entry.velKmh, entry.erro,
        entry.correcaoPID, entry.pwmEsq, entry.pwmDir);

    lastLog = millis();
}

void Logger::logComPosicao(int erro, float correcao, int pwmE, int pwmD, float kp, float ki, float kd, uint16_t posicao) {
    if (millis() - lastLog < 80) return;

    float deltaT = (millis() - lastLog) / 1000.0f;
    float velInstant = ((abs(pwmE) + abs(pwmD)) / 2.0f) * 0.0018f;
    distancia += velInstant * deltaT;

    // Atualiza limites para normalização ML
    if (erro < minErro) minErro = erro;
    if (erro > maxErro) maxErro = erro;
    if (pwmE < minPWM) minPWM = pwmE;
    if (pwmE > maxPWM) maxPWM = pwmE;
    if (pwmD < minPWM) minPWM = pwmD;
    if (pwmD > maxPWM) maxPWM = pwmD;
    if (posicao < minPos) minPos = posicao;
    if (posicao > maxPos) maxPos = posicao;

    LogEntry entry;
    entry.tempo = millis() - startTime;
    entry.erro = erro;
    entry.correcaoPID = correcao;
    entry.pwmEsq = pwmE;
    entry.pwmDir = pwmD;
    entry.kp_term = kp;
    entry.ki_term = ki;
    entry.kd_term = kd;
    entry.posicaoSensor = posicao;
    entry.rpm = 0;
    entry.velKmh = 0;

    logs.push_back(entry);
    
    lastLog = millis();
}

void Logger::endRun() {
    Serial.println("\n=== FIM DA CORRIDA - DADOS COLETADOS ===\n");
    printStats();
    saveToSerialCSV();
}

void Logger::printStats() {
    Serial.printf("Tempo total      : %.1f segundos\n", getTempoTotal());
    Serial.printf("Distância        : %.2f metros\n", distancia);
    Serial.printf("Velocidade média : %.2f km/h\n", getVelMedia());
    Serial.printf("Amostras         : %d\n", logs.size());
}

void Logger::saveToSerialCSV() {
    Serial.println("\n--- COLE TODO ESTE CONTEÚDO ABAIXO EM UM ARQUIVO data.csv ---");
    Serial.println("tempo_ms,rpm,vel_kmh,erro,correcao_pid,pwm_esq,pwm_dir");
    for (auto& e : logs) {
        Serial.printf("%lu,%.1f,%.2f,%d,%.1f,%d,%d\n",
            e.tempo, e.rpm, e.velKmh, e.erro, e.correcaoPID, e.pwmEsq, e.pwmDir);
    }
}

float Logger::getVelMedia() {
    float t = getTempoTotal();
    return (t > 0) ? (distancia / t) * 3.6f : 0;
}

float Logger::getTempoTotal() {
    return (millis() - startTime) / 1000.0f;
}

String Logger::getCSVData() {
    String csv = "tempo_ms,erro,correcao,pwm_esq,pwm_dir,posicao,kp,ki,kd\n";
    for (const auto& e : logs) {
        csv += String(e.tempo) + "," + String(e.erro) + "," + String(e.correcaoPID, 2) + "," +
               String(e.pwmEsq) + "," + String(e.pwmDir) + "," + String(e.posicaoSensor) + "," +
               String(e.kp_term, 6) + "," + String(e.ki_term, 6) + "," + String(e.kd_term, 6) + "\n";
    }
    return csv;
}

String Logger::getMLJSON() {
    // Retorna dados normalizados (0-1) para consumo por modelos ML
    String json = "[";
    
    for (size_t i = 0; i < logs.size(); i++) {
        const auto& e = logs[i];
        
        // Normaliza valores
        float erroNorm = (maxErro == minErro) ? 0.5f : (float)(e.erro - minErro) / (maxErro - minErro);
        float pwmNorm = (maxPWM == minPWM) ? 0.5f : (float)(e.pwmEsq - minPWM) / (maxPWM - minPWM);
        float posNorm = (maxPos == minPos) ? 0.5f : (e.posicaoSensor - minPos) / (maxPos - minPos);
        
        json += "{\"t\":" + String(e.tempo) + 
                ",\"e\":" + String(erroNorm, 3) +
                ",\"c\":" + String(e.correcaoPID, 2) +
                ",\"p\":" + String(pwmNorm, 3) +
                ",\"pos\":" + String(posNorm, 3) +
                ",\"kp\":" + String(e.kp_term, 6) +
                ",\"ki\":" + String(e.ki_term, 6) +
                ",\"kd\":" + String(e.kd_term, 6) + "}";
        
        if (i < logs.size() - 1) json += ",";
    }
    
    json += "]";
    return json;
}

void Logger::reset() {
    logs.clear();
    distancia = 0;
    startTime = millis();
    minErro = 2147483647;
    maxErro = -2147483648;
    minPWM = 2147483647;
    maxPWM = -2147483648;
    minPos = 7000.0f;
    maxPos = 0.0f;
}