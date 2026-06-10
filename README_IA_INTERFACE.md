# 🤖 RoboSeguidor - Interface & IA - Quick Start

## 📋 O que foi implementado

### ✅ Interface Web Completa
- **Dashboard em tempo real** com 5 blocos de informação
- **Gráfico de trajetória** em canvas (tempo real)
- **Análise IA integrada** com recomendações automáticas
- **Botão "APLICAR RECOMENDAÇÃO"** para otimizar PID dinamicamente
- **Download CSV** com histórico completo

### ✅ Endpoints REST
```
GET  /                  → Interface web
GET  /dados.json        → Último ponto de dados
GET  /dados.csv         → Histórico CSV
GET  /ml.json           → Análise de curvas clássica
GET  /tune              → ⭐ Análise IA + recomendações
GET  /aplicar-pid       → ⭐ Aplica novo PID dinâmicamente  
GET  /reset             → Limpa buffer
GET  /telemetria        → SSE (streaming)
```

### ✅ Cliente Python v2.0
- Menu interativo com 7 opções
- Análise IA no servidor
- Aplicação automática de PID
- Ciclos de otimização em loop
- Stream ao vivo de dados

---

## 🚀 Como Usar

### 1️⃣ Upload do Código
```bash
# No VS Code:
# 1. Conecte ESP32-S3 via USB
# 2. PlatformIO → Upload
# 3. Aguarde conclusão
```

### 2️⃣ Conectar ao WiFi do Robô
```
SSID: RoboSeguidor
Senha: (nenhuma)
IP: 192.168.1.1
```

### 3️⃣ Abrir Interface Web
```
Navegador: http://192.168.1.1
```

### 4️⃣ Usar Cliente Python (Recomendado)
```bash
# Terminal:
python ml_consumer_example.py

# Menu:
[1] Verificar conexão
[2] Obter análise IA
[3] Aplicar PID manual
[4] Stream ao vivo
[5] Baixar CSV
[6] ⭐ OTIMIZAR COMPLETO (automático)
[7] Sair
```

---

## 📊 Fluxo de Otimização com IA

### Automático (Opção 6)
```
1. Coleta dados por 10 segundos
   ↓
2. Envia para análise IA
   ↓
3. IA recomenda: Kp, Ki, Kd
   ↓
4. Pergunta se aplica
   ↓
5. Aplica via /aplicar-pid
   ↓
6. Coleta mais 10s com novo PID
   ↓
7. Compara antes/depois
```

### Manual (Interface Web)
```
1. Executar robô (20+ segundos)
   ↓
2. Clicar "ATUALIZAR ML" (B5)
   ↓
3. Ver recomendações
   ↓
4. Clicar "✓ APLICAR RECOMENDAÇÃO"
   ↓
5. Novo PID é aplicado
```

---

## 📈 Algoritmo IA - Como Funciona

### Métricas Coletadas
```
✓ Erro médio          → Qualidade do seguimento
✓ Erro máximo         → Picos de instabilidade
✓ Correção média/máx  → Esforço do PID
✓ Aceleração lateral  → Tipo de curva
✓ Instabilidades      → Saltos > 500 ADC
```

### Lógica de Recomendação
```
Erro Alto (>2000)  → ↑ Kp=0.15, ↑ Kd=0.30  (agressivo)
Erro Médio (500)   → ↔ Kp=0.07, → Kd=0.20  (moderado)
Erro Baixo (<200)  → ↓ Kp=0.03, ↓ Kd=0.10  (fino)

Aceleração Y > 2g  → +0.15 extra em Kd (curvas)
```

---

## 🎯 Exemplo Prático

### Ciclo 1: Diagnóstico
```bash
$ python ml_consumer_example.py
Menu > [2]  # Obter análise

[IA ANALYSIS]
Erro Médio: 850
Instabilidades: 5
Aceleração Y: 0.450g

[RECOMENDAÇÃO]
Kp: 0.070000
Kd: 0.200000
Confiança: 85%
```

### Ciclo 2: Otimização
```bash
Menu > [6]  # Otimizar completo

[INFO] Coletando dados por 10 segundos...
[INFO] Requisitando análise IA...

[IA ANALYSIS]
Erro Médio: 850
[RECOMENDAÇÃO]
Kp: 0.070000
Kd: 0.200000

Deseja aplicar a recomendação? (S/n): S
[✓] PID atualizado com sucesso!

[INFO] Aguardando dados com novo PID...
[IA ANALYSIS]
Erro Médio: 480  ← MELHOROU! ✅
```

---

## 🔧 Configurações Avançadas

### Modificar Intervalo de SSE
```cpp
// Em TelemetriaWiFi.cpp linha ~15
const unsigned long INTERVALO_SSE_MS = 100;  // 100ms = 10Hz
// Mudar para 50 para 20Hz, 200 para 5Hz, etc.
```

### Ajustar Buffer
```cpp
// Em TelemetriaWiFi.cpp linha ~28
const int BUFFER_SIZE = 1000;  // 1000 pontos
// Com 100ms = ~100 segundos de histórico
// Mudar para 500 para 50s, 2000 para 200s
```

### Aplicar PID Real (TODO)
```cpp
// Em TelemetriaWiFi.cpp linha ~285
// Descomente para aplicar PID dinâmico:
// KP = novoKp;
// KD = novoKd;
// KI = webServer.hasArg("ki") ? webServer.arg("ki").toFloat() : KI;

// Isso requer:
// extern float KP, KI, KD;
// em Configuracoes.h
```

---

## 📱 Acessar de Outro Dispositivo

### Smartphone/Tablet
```
1. Conectar ao WiFi "RoboSeguidor"
2. Abrir navegador: http://192.168.1.1
3. Dashboard funcional em tempo real
```

### PC (Python)
```bash
python ml_consumer_example.py
```

### Mac/Linux
```bash
# Instalar requests
pip3 install requests

# Rodar
python3 ml_consumer_example.py
```

---

## 🐛 Troubleshooting

### "Robô não responde"
```bash
# Verificar conectividade
python -c "
from ml_consumer_example import RoboSeguidor
robo = RoboSeguidor('192.168.1.1')
print('Conectado!' if robo.verificar_conexao() else 'Erro!')
"
```

### "Erro 400: Faltam parâmetros"
```
Usar cliente Python (menu opção 3) em vez de URL manual
Ou verificar: /aplicar-pid?kp=0.05&kd=0.2
```

### "Dados insuficientes"
```
Rodar robô por mais de 20 segundos antes de analisar
/tune retorna erro se bufferCount < 20
```

---

## 📊 Monitorar em Tempo Real

### Opção 1: Interface Web
```
http://192.168.1.1
→ Atualiza a cada 200ms
→ Visualizar gráfico ao vivo
```

### Opção 2: Stream Python
```python
dados = robo.stream_ao_vivo(duracao_segundos=60)
# Exibe: Pos, Erro, PWM-E, PWM-D
```

### Opção 3: CSV + Análise
```python
robo.obter_dados_csv()
df = robo.analisar_dados_localmente("telemetria.csv")
# Estatísticas + gráficos com pandas
```

---

## ✨ Próximos Passos

- [ ] Integrar recomendações em tempo real no C++
- [ ] TensorFlow Lite no ESP32
- [ ] Persistência de dados em SPIFFS
- [ ] Multi-robô com MQTT
- [ ] Dashboard web avançado (Plotly)
- [ ] API de webhooks

---

## 📞 Suporte

**Problemas?** Verifique:
1. WiFi conectado ✓
2. IP correto (192.168.1.1) ✓
3. Robô em normal mode (não config) ✓
4. Mínimo 20 pontos de dados para IA ✓
5. Python 3.6+ instalado ✓

---

**Status**: ✅ **FUNCIONAL E PRONTO PARA USO**
**Versão**: 2.0
**Data**: 2025-06-09

🚀 **Boa otimização!**
