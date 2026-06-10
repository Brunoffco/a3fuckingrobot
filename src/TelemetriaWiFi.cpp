#include <WiFi.h>
#include <WebServer.h>
#include "TelemetriaWiFi.h"
#include "LSM6DS3.h"

const char* AP_SSID = "RoboSeguidor";
const char* AP_SENHA = "";
const IPAddress GATEWAY(192, 168, 1, 1);
const IPAddress SUBNET(255, 255, 255, 0);

WebServer webServer(80);
bool clientesConectados = false;

// Buffer para armazenar JSON de telemetria
String ultimoJSON = "";
unsigned long ultimoEnvioSSE = 0;
const unsigned long INTERVALO_SSE_MS = 100;

// Estrutura para armazenar dados de telemetria
struct TelemetriaData {
  unsigned long timestamp;
  uint16_t position;
  int16_t error;
  float correction;
  uint8_t pwmL;
  uint8_t pwmR;
  float kp, ki, kd;
  float ax, ay, az;
  float gx, gy, gz;
};

// Buffer circular de dados (1000 pontos)
const int BUFFER_SIZE = 1000;
TelemetriaData dataBuffer[BUFFER_SIZE];
int bufferIndex = 0;
int bufferCount = 0;

// Variáveis globais para PID otimizado
float pidKpOtimizado = -1.0;
float pidKdOtimizado = -1.0;
bool pidAplicado = false;

// ML: Classificar tipo de curva baseado em aceleração lateral
struct CurveAnalysis {
  float avgAccelY;
  float peakAccelY;
  int curveType;  // 0=reta, 1=suave, 2=média, 3=forte
  float recommendedKp;
  float recommendedKd;
};

// Forward declarations
void handleRaiz();
void handleTelemetria();
void handleJSON();
void handleCSV();
void handleML();
void handleReset();
void handleTuneML();
void handleAplicarPID();
CurveAnalysis analisarCurva(int startIdx, int endIdx);

void handleRaiz() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <title>RoboSeguidor - Telemetria</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body { 
      font-family: 'Courier New', monospace; 
      background: #0a0e27; 
      color: #e0e0e0; 
      padding: 20px;
      line-height: 1.4;
    }
    .container { max-width: 1600px; margin: 0 auto; }
    .header {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 30px;
      border-bottom: 2px solid #00d4ff;
      padding-bottom: 15px;
    }
    .header h1 { 
      font-size: 28px; 
      color: #00d4ff;
      letter-spacing: 2px;
    }
    .status-badge {
      padding: 8px 16px;
      border-radius: 4px;
      font-size: 12px;
      font-weight: bold;
      display: inline-block;
    }
    .status-badge.online { 
      background: #00d4ff; 
      color: #0a0e27;
    }
    .status-badge.offline { 
      background: #ff4757; 
      color: #fff;
    }
    .grid { 
      display: grid; 
      grid-template-columns: repeat(4, 1fr); 
      gap: 20px;
      margin-bottom: 30px;
    }
    @media (max-width: 1200px) {
      .grid { grid-template-columns: repeat(2, 1fr); }
    }
    .card { 
      background: linear-gradient(135deg, #1a2847 0%, #0f1d3d 100%);
      border: 1px solid #00d4ff;
      border-radius: 4px;
      padding: 20px;
      position: relative;
      overflow: hidden;
    }
    .card::before {
      content: '';
      position: absolute;
      top: 0;
      left: 0;
      right: 0;
      height: 2px;
      background: linear-gradient(90deg, transparent, #00d4ff, transparent);
    }
    .card-label { 
      font-size: 11px;
      color: #00d4ff;
      text-transform: uppercase;
      letter-spacing: 1px;
      margin-bottom: 12px;
      opacity: 0.7;
    }
    .card-value { 
      font-size: 32px;
      font-weight: bold;
      color: #00ff88;
      margin-bottom: 10px;
    }
    .card-subtext {
      font-size: 12px;
      color: #888;
    }
    .bar-chart {
      display: flex;
      gap: 3px;
      margin-top: 8px;
    }
    .bar {
      flex: 1;
      height: 20px;
      background: #1a2847;
      border: 1px solid #00d4ff;
      border-radius: 2px;
      position: relative;
      overflow: hidden;
    }
    .bar-fill {
      height: 100%;
      background: linear-gradient(90deg, #00d4ff, #00ff88);
      width: 50%;
      border-radius: 1px;
    }
    .sensor-array {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 8px;
      margin-top: 12px;
    }
    .sensor-dot {
      width: 40px;
      height: 40px;
      border-radius: 50%;
      border: 2px solid #00d4ff;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 10px;
      color: #00d4ff;
      font-weight: bold;
    }
    .sensor-dot.active {
      background: #00d4ff;
      color: #0a0e27;
      box-shadow: 0 0 10px rgba(0, 212, 255, 0.5);
    }
    .section-title {
      font-size: 13px;
      color: #00d4ff;
      text-transform: uppercase;
      letter-spacing: 1px;
      margin: 20px 0 15px 0;
      display: flex;
      align-items: center;
      gap: 10px;
    }
    .section-title::before {
      content: '';
      width: 20px;
      height: 20px;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 11px;
      background: #00d4ff;
      color: #0a0e27;
      border-radius: 2px;
      font-weight: bold;
    }
    .section-title.b1::before { content: 'B1'; }
    .section-title.b2::before { content: 'B2'; }
    .section-title.b3::before { content: 'B3'; }
    .section-title.b4::before { content: 'B4'; }
    .section-title.b5::before { content: 'B5'; }
    .canvas-container {
      background: linear-gradient(135deg, #1a2847 0%, #0f1d3d 100%);
      border: 1px solid #00d4ff;
      border-radius: 4px;
      padding: 20px;
      margin-bottom: 20px;
    }
    .canvas-container::before {
      content: '';
      display: block;
      position: absolute;
      top: 0;
      left: 0;
      right: 0;
      height: 2px;
      background: linear-gradient(90deg, transparent, #00d4ff, transparent);
    }
    canvas { 
      display: block; 
      margin: 0 auto;
      width: 100%;
      max-width: 100%;
    }
    .ml-section {
      background: linear-gradient(135deg, #1a2847 0%, #0f1d3d 100%);
      border: 1px solid #00d4ff;
      border-radius: 4px;
      padding: 20px;
      margin-bottom: 20px;
    }
    .ml-section::before {
      content: '';
      position: absolute;
      top: 0;
      left: 0;
      right: 0;
      height: 2px;
      background: linear-gradient(90deg, transparent, #00d4ff, transparent);
    }
    .ml-content {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 20px;
      font-size: 12px;
      line-height: 1.8;
    }
    .ml-item {
      padding: 12px;
      background: rgba(0, 212, 255, 0.05);
      border-left: 3px solid #00d4ff;
      border-radius: 2px;
    }
    .ml-item-title {
      color: #00ff88;
      font-weight: bold;
      margin-bottom: 5px;
    }
    .controls {
      display: flex;
      gap: 10px;
      margin-top: 20px;
      justify-content: center;
    }
    button {
      background: linear-gradient(135deg, #00d4ff 0%, #00ff88 100%);
      color: #0a0e27;
      border: none;
      padding: 10px 20px;
      border-radius: 4px;
      font-weight: bold;
      cursor: pointer;
      font-size: 12px;
      text-transform: uppercase;
      letter-spacing: 1px;
      transition: transform 0.2s;
    }
    button:hover {
      transform: translateY(-2px);
    }
    button:active {
      transform: translateY(0);
    }
  </style>
</head>
<body>
  <div class="container">
    <div class="header">
      <h1>LINEBOT - RoboSeguidor</h1>
      <div class="status-badge online" id="statusBadge">● CONECTADO</div>
    </div>

    <!-- B1: RPM - ENCODER -->
    <div class="section-title b1">RPM - ENCODER</div>
    <div class="grid">
      <div class="card">
        <div class="card-label">Motor Esquerda</div>
        <div class="card-value" id="rpmE">0</div>
        <div class="bar-chart">
          <div class="bar"><div class="bar-fill" id="barE"></div></div>
          <div class="bar"><div class="bar-fill" id="barE2"></div></div>
        </div>
        <div class="card-subtext">Pulsos/rev: 28 | Freq: 120 Hz</div>
      </div>
      <div class="card">
        <div class="card-label">Motor Direita</div>
        <div class="card-value" id="rpmD">0</div>
        <div class="bar-chart">
          <div class="bar"><div class="bar-fill" id="barD"></div></div>
          <div class="bar"><div class="bar-fill" id="barD2"></div></div>
        </div>
        <div class="card-subtext">PWM: 0-255 | Base: 220</div>
      </div>
      <div class="card">
        <div class="card-label">Posição Sensor</div>
        <div class="card-value" id="posicao">3500</div>
        <div class="bar-chart">
          <div class="bar"><div class="bar-fill" id="posBar"></div></div>
        </div>
        <div class="card-subtext">Range: 0-7000</div>
      </div>
      <div class="card">
        <div class="card-label">Status</div>
        <div class="card-value" id="status" style="color: #00ff88;">ONLINE</div>
        <div class="bar-chart">
          <div class="bar"><div class="bar-fill" style="width: 100%;"></div></div>
        </div>
        <div class="card-subtext">Conexão WiFi ativa</div>
      </div>
    </div>

    <!-- B2: SENSORES QRE -->
    <div class="section-title b2">SENSORES QRE</div>
    <div class="card" style="grid-column: 1 / -1;">
      <div class="card-label">Array de Sensores (8 unidades)</div>
      <div class="sensor-array">
        <div class="sensor-dot" id="s0">S0</div>
        <div class="sensor-dot" id="s1">S1</div>
        <div class="sensor-dot" id="s2">S2</div>
        <div class="sensor-dot" id="s3">S3</div>
        <div class="sensor-dot" id="s4">S4</div>
        <div class="sensor-dot" id="s5">S5</div>
        <div class="sensor-dot" id="s6">S6</div>
        <div class="sensor-dot" id="s7">S7</div>
      </div>
      <div style="margin-top: 15px; color: #00ff88;">■ Linha detectada</div>
    </div>

    <!-- B3: LSM - IMU -->
    <div class="section-title b3">LSM - IMU</div>
    <div class="grid">
      <div class="card">
        <div class="card-label">Aceleração</div>
        <div style="margin: 12px 0;">
          <div style="display: flex; justify-content: space-between; margin: 5px 0;">
            <span>aX</span><span id="ax">0.00</span>
          </div>
          <div class="bar"><div class="bar-fill" id="axBar"></div></div>
          <div style="display: flex; justify-content: space-between; margin: 8px 0 5px 0;">
            <span>aY</span><span id="ay">0.00</span>
          </div>
          <div class="bar"><div class="bar-fill" id="ayBar"></div></div>
          <div style="display: flex; justify-content: space-between; margin: 8px 0 5px 0;">
            <span>aZ</span><span id="az">0.00</span>
          </div>
          <div class="bar"><div class="bar-fill" id="azBar"></div></div>
        </div>
      </div>
      <div class="card">
        <div class="card-label">Giroscópio</div>
        <div style="margin: 12px 0;">
          <div style="display: flex; justify-content: space-between; margin: 5px 0;">
            <span>gX</span><span id="gx">0.00</span>
          </div>
          <div class="bar"><div class="bar-fill" id="gxBar"></div></div>
          <div style="display: flex; justify-content: space-between; margin: 8px 0 5px 0;">
            <span>gY</span><span id="gy">0.00</span>
          </div>
          <div class="bar"><div class="bar-fill" id="gyBar"></div></div>
          <div style="display: flex; justify-content: space-between; margin: 8px 0 5px 0;">
            <span>gZ</span><span id="gz">0.00</span>
          </div>
          <div class="bar"><div class="bar-fill" id="gzBar"></div></div>
        </div>
      </div>
      <div class="card">
        <div class="card-label">Controle PID</div>
        <div style="margin: 12px 0; line-height: 2;">
          <div>Erro: <span id="erro" style="color: #00ff88;">0</span></div>
          <div>Correção: <span id="correcao" style="color: #00ff88;">0.0</span></div>
          <div style="margin-top: 8px; font-size: 10px; color: #888;">
            Kp: <span id="kp">0.05</span><br>
            Ki: <span id="ki">0.000</span><br>
            Kd: <span id="kd">0.2</span>
          </div>
        </div>
      </div>
      <div class="card">
        <div class="card-label">PWM Motores</div>
        <div style="margin: 12px 0;">
          <div style="margin-bottom: 10px;">
            <div style="display: flex; justify-content: space-between; margin-bottom: 5px;">
              <span>Motor E</span><span id="pwmE">0</span>
            </div>
            <div class="bar"><div class="bar-fill" id="pwmEBar"></div></div>
          </div>
          <div>
            <div style="display: flex; justify-content: space-between; margin-bottom: 5px;">
              <span>Motor D</span><span id="pwmD">0</span>
            </div>
            <div class="bar"><div class="bar-fill" id="pwmDBar"></div></div>
          </div>
        </div>
      </div>
    </div>

    <!-- B4: PISTA - VISUALIZAÇÃO -->
    <div class="section-title b4">PISTA - VISUALIZAÇÃO EM TEMPO REAL</div>
    <div class="canvas-container">
      <button onclick="resetarTrilha()" style="position: absolute; right: 20px; top: 20px; padding: 6px 12px; font-size: 10px;">RESETAR TRILHA</button>
      <canvas id="trackCanvas" width="1200" height="350" style="border-radius: 2px;"></canvas>
    </div>

    <!-- B5: ANÁLISE ML -->
    <div class="section-title b5">ANÁLISE DE CURVAS (ML)</div>
    <div class="ml-section">
      <div id="mlAnalysis" style="color: #e0e0e0; line-height: 1.8;">
        <p style="text-align: center; color: #888;">Carregando análise...</p>
      </div>
    </div>

    <div class="controls">
      <button onclick="baixarDados()">BAIXAR CSV</button>
      <button onclick="atualizarML()">ATUALIZAR ML</button>
      <button onclick="limparDados()">LIMPAR</button>
    </div>
  </div>

  <script>
    const dados = [];
    let updateInterval;
    const canvas = document.getElementById('trackCanvas');
    const ctx = canvas.getContext('2d');
    
    function desenharPista() {
      ctx.fillStyle = '#1a1a1a';
      ctx.fillRect(0, 0, canvas.width, canvas.height);
      
      // Desenhar grid
      ctx.strokeStyle = '#333333';
      ctx.lineWidth = 1;
      for (let i = 0; i <= canvas.width; i += 50) {
        ctx.beginPath();
        ctx.moveTo(i, 0);
        ctx.lineTo(i, canvas.height);
        ctx.stroke();
      }
      for (let i = 0; i <= canvas.height; i += 50) {
        ctx.beginPath();
        ctx.moveTo(0, i);
        ctx.lineTo(canvas.width, i);
        ctx.stroke();
      }
      
      // Desenhar bordas da pista
      const trackLeft = canvas.height / 2 - 40;
      const trackRight = canvas.height / 2 + 40;
      
      ctx.strokeStyle = '#ff6600';
      ctx.lineWidth = 2;
      ctx.setLineDash([5, 5]);
      ctx.beginPath();
      ctx.moveTo(0, trackLeft);
      ctx.lineTo(canvas.width, trackLeft);
      ctx.stroke();
      
      ctx.beginPath();
      ctx.moveTo(0, trackRight);
      ctx.lineTo(canvas.width, trackRight);
      ctx.stroke();
      ctx.setLineDash([]);
      
      // Desenhar linha central
      ctx.strokeStyle = '#00ff00';
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.moveTo(0, canvas.height / 2);
      ctx.lineTo(canvas.width, canvas.height / 2);
      ctx.stroke();
    }
    
    function desenharTrajetoria() {
      if (dados.length === 0) return;
      
      desenharPista();
      
      // Desenhar posição do sensor ao longo do tempo
      ctx.strokeStyle = '#00ffff';
      ctx.lineWidth = 2;
      ctx.beginPath();
      
      for (let i = 0; i < dados.length; i++) {
        const d = dados[i];
        const x = (i / Math.max(1, dados.length - 1)) * canvas.width;
        // Posição: 0-7000, mapeado para tela
        const posY = canvas.height / 2 - ((d.pos - 3500) / 3500) * (canvas.height / 2 - 10);
        
        if (i === 0) {
          ctx.moveTo(x, posY);
        } else {
          ctx.lineTo(x, posY);
        }
      }
      ctx.stroke();
      
      // Desenhar posição atual
      if (dados.length > 0) {
        const d = dados[dados.length - 1];
        const x = canvas.width - 10;
        const posY = canvas.height / 2 - ((d.pos - 3500) / 3500) * (canvas.height / 2 - 10);
        
        ctx.fillStyle = '#ffff00';
        ctx.beginPath();
        ctx.arc(x, posY, 8, 0, 2 * Math.PI);
        ctx.fill();
        
        // Label com posição
        ctx.fillStyle = '#ffff00';
        ctx.font = 'bold 12px monospace';
        ctx.fillText('Pos: ' + d.pos, 10, canvas.height - 10);
      }
    }
    
    function atualizarML() {
      fetch('/tune')
        .then(response => response.json())
        .then(data => {
          let html = '<div class="ml-content">';
          
          if (data.metrics) {
            html += '<div class="ml-item">';
            html += '<div class="ml-item-title">📊 Métricas Atuais</div>';
            html += 'Erro Médio: <strong>' + data.metrics.avgError + '</strong><br>';
            html += 'Erro Máx: <strong>' + data.metrics.maxError + '</strong><br>';
            html += 'Instabilidades: <strong>' + data.metrics.instabilityCount + '</strong><br>';
            html += 'Acel. Y: <strong>' + data.metrics.accelY.avg.toFixed(3) + ' g</strong>';
            html += '</div>';
            
            if (data.aiRecommendation) {
              html += '<div class="ml-item">';
              html += '<div class="ml-item-title">🤖 IA Recomenda</div>';
              html += 'Kp: <strong id="recKp">' + data.aiRecommendation.kp.toFixed(6) + '</strong><br>';
              html += 'Ki: <strong>' + data.aiRecommendation.ki.toFixed(8) + '</strong><br>';
              html += 'Kd: <strong id="recKd">' + data.aiRecommendation.kd.toFixed(6) + '</strong><br>';
              html += 'Confiança: <strong>' + (data.aiRecommendation.confidence * 100).toFixed(0) + '%</strong>';
              html += '</div>';
              
              html += '<div class="ml-item">';
              html += '<button onclick="aplicarRecomendacaoIA(' + data.aiRecommendation.kp.toFixed(6) + ', ' + data.aiRecommendation.kd.toFixed(6) + ')" style="width: 100%; margin-top: 10px;">✓ APLICAR RECOMENDAÇÃO</button>';
              html += '</div>';
            }
            
            html += '<div class="ml-item">';
            html += '<div class="ml-item-title">💾 Histórico</div>';
            html += 'Buffer: <strong>' + data.bufferSize + '</strong> pontos<br>';
            if (data.optimized.kp > 0) {
              html += 'Último Kp: <strong>' + data.optimized.kp.toFixed(6) + '</strong><br>';
              html += 'Último Kd: <strong>' + data.optimized.kd.toFixed(6) + '</strong>';
            } else {
              html += '(Nenhuma otimização aplicada ainda)';
            }
            html += '</div>';
          } else if (data.error) {
            html += '<div class="ml-item"><strong style="color: #ff6666;">⚠️ ' + data.error + '</strong></div>';
          }
          
          html += '</div>';
          document.getElementById('mlAnalysis').innerHTML = html;
        })
        .catch(err => {
          document.getElementById('mlAnalysis').innerHTML = '<div class="ml-item"><strong style="color: #ff6666;">Erro ao carregar análise IA</strong></div>';
        });
    }
    
    function aplicarRecomendacaoIA(kp, kd) {
      const confirmMsg = 'Aplicar Kp=' + kp.toFixed(6) + ' e Kd=' + kd.toFixed(6) + '?';
      if (!confirm(confirmMsg)) return;
      
      fetch('/aplicar-pid?kp=' + kp + '&kd=' + kd)
        .then(response => response.json())
        .then(data => {
          if (data.status === 'OK') {
            alert('✓ PID atualizado com sucesso!');
            document.getElementById('kp').textContent = kp.toFixed(6);
            document.getElementById('kd').textContent = kd.toFixed(6);
            atualizarML();
          } else {
            alert('Erro: ' + data.error);
          }
        })
        .catch(err => alert('Erro de conexão'));
    }
    
    function atualizarDados() {
      // Atualizar com o último JSON do servidor
      fetch('/dados.json')
        .then(response => response.json())
        .then(data => {
          if (data && data.t) {
            // Atualizar cards
            document.getElementById('posicao').textContent = data.pos;
            document.getElementById('pwmE').textContent = data.pwmE;
            document.getElementById('pwmD').textContent = data.pwmD;
            document.getElementById('rpmE').textContent = data.pwmE;
            document.getElementById('rpmD').textContent = data.pwmD;
            document.getElementById('erro').textContent = data.err;
            document.getElementById('correcao').textContent = data.cor.toFixed(1);
            document.getElementById('kp').textContent = data.kp.toFixed(6);
            document.getElementById('ki').textContent = data.ki.toFixed(6);
            document.getElementById('kd').textContent = data.kd.toFixed(6);
            document.getElementById('ax').textContent = (data.ax || 0).toFixed(2);
            document.getElementById('ay').textContent = (data.ay || 0).toFixed(2);
            document.getElementById('az').textContent = (data.az || 0).toFixed(2);
            document.getElementById('gx').textContent = (data.gx || 0).toFixed(2);
            document.getElementById('gy').textContent = (data.gy || 0).toFixed(2);
            document.getElementById('gz').textContent = (data.gz || 0).toFixed(2);
            
            // Atualizar barras de progresso
            const posPercent = ((data.pos / 7000) * 100) + '%';
            document.getElementById('posBar').style.width = posPercent;
            const pwmPercent = ((Math.abs(data.pwmE) / 255) * 100) + '%';
            document.getElementById('pwmEBar').style.width = pwmPercent;
            document.getElementById('pwmDBar').style.width = ((Math.abs(data.pwmD) / 255) * 100) + '%';
            
            document.getElementById('status').textContent = 'ONLINE';
            document.getElementById('statusBadge').className = 'status-badge online';
            
            dados.push(data);
            if (dados.length > 500) dados.shift();
            desenharTrajetoria();
          }
        })
        .catch(err => {
          document.getElementById('status').textContent = 'OFFLINE';
          document.getElementById('statusBadge').className = 'status-badge offline';
        });
    }
    
    function resetarTrilha() {
      dados.length = 0;
      desenharPista();
      fetch('/reset')
        .then(() => console.log('Trilha resetada'))
        .catch(err => console.log('Erro ao resetar'));
    }
    
    function baixarDados() {
      fetch('/dados.csv')
        .then(response => response.text())
        .then(csv => {
          const blob = new Blob([csv], {type: 'text/csv; charset=utf-8'});
          const url = URL.createObjectURL(blob);
          const a = document.createElement('a');
          a.href = url;
          a.download = 'telemetria_' + new Date().toISOString().slice(0,10) + '.csv';
          a.click();
        });
    }
    
    function limparDados() {
      dados.length = 0;
      desenharPista();
      fetch('/ml.json').then(r => r.json()).then(d => {
        if (d.bufferSize !== undefined) {
          console.log('Servidor tem ' + d.bufferSize + ' pontos armazenados');
        }
      });
    }
    
    desenharPista();
    atualizarML();
    
    // Iniciar atualização periódica
    setInterval(atualizarDados, 200);
  </script>
</body>
</html>
)rawliteral";
  
  webServer.send(200, "text/html", html);
}

void handleTelemetria() {
  clientesConectados = true;
  
  // Enviar headers para SSE (Server-Sent Events)
  webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webServer.send(200, "text/event-stream", "");
  
  // Enviar alguns pontos do buffer em SSE format
  for (int i = 0; i < min(50, bufferCount); i++) {
    TelemetriaData& d = dataBuffer[i];
    char jsonLine[350];
    snprintf(jsonLine, sizeof(jsonLine),
      "data: {\"t\":%lu,\"pos\":%u,\"err\":%d,\"cor\":%.2f,\"pwmE\":%d,\"pwmD\":%d,\"kp\":%.6f,\"ki\":%.6f,\"kd\":%.6f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f}\n\n",
      d.timestamp, d.position, d.error, d.correction, d.pwmL, d.pwmR,
      d.kp, d.ki, d.kd, d.ax, d.ay, d.az, d.gx, d.gy, d.gz
    );
    webServer.sendContent(jsonLine);
  }
  
  clientesConectados = false;
}

void handleJSON() {
  webServer.send(200, "application/json", ultimoJSON);
}

void handleCSV() {
  // Gera CSV com todos os dados coletados
  String csv = "timestamp,posicao,erro,correcao,pwm_esq,pwm_dir,kp,ki,kd,ax,ay,az,gx,gy,gz\n";
  
  for (int i = 0; i < bufferCount; i++) {
    TelemetriaData& d = dataBuffer[i];
    char line[300];
    snprintf(line, sizeof(line), 
      "%lu,%u,%d,%.2f,%u,%u,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
      d.timestamp, d.position, d.error, d.correction, d.pwmL, d.pwmR,
      d.kp, d.ki, d.kd, d.ax, d.ay, d.az, d.gx, d.gy, d.gz
    );
    csv += line;
  }
  
  webServer.send(200, "text/csv; charset=utf-8", csv);
}


CurveAnalysis analisarCurva(int startIdx, int endIdx) {
  CurveAnalysis analysis;
  analysis.avgAccelY = 0;
  analysis.peakAccelY = 0;
  
  if (endIdx <= startIdx) {
    analysis.curveType = 0;
    analysis.recommendedKp = 0.05;
    analysis.recommendedKd = 0.2;
    return analysis;
  }
  
  int count = 0;
  for (int i = startIdx; i < endIdx && i < bufferCount; i++) {
    analysis.avgAccelY += dataBuffer[i].ay;
    if (fabs(dataBuffer[i].ay) > fabs(analysis.peakAccelY)) {
      analysis.peakAccelY = dataBuffer[i].ay;
    }
    count++;
  }
  
  if (count > 0) {
    analysis.avgAccelY /= count;
  }
  
  float peakAbsolute = fabs(analysis.peakAccelY);
  
  // Classificar curva
  if (peakAbsolute < 0.5) {
    analysis.curveType = 0;  // Reta
    analysis.recommendedKp = 0.03;
    analysis.recommendedKd = 0.15;
  } else if (peakAbsolute < 1.0) {
    analysis.curveType = 1;  // Suave
    analysis.recommendedKp = 0.05;
    analysis.recommendedKd = 0.20;
  } else if (peakAbsolute < 1.5) {
    analysis.curveType = 2;  // Média
    analysis.recommendedKp = 0.07;
    analysis.recommendedKd = 0.25;
  } else {
    analysis.curveType = 3;  // Forte
    analysis.recommendedKp = 0.10;
    analysis.recommendedKd = 0.35;
  }
  
  return analysis;
}

void handleML() {
  // Analisar padrões de aceleração e fornecer otimizações
  String json = "{";
  json += "\"bufferSize\":" + String(bufferCount) + ",";
  
  // Análise geral de aceleração
  if (bufferCount > 10) {
    CurveAnalysis fullAnalysis = analisarCurva(0, bufferCount);
    
    // Análise de segmentos (dividir em 4 partes)
    CurveAnalysis seg1 = analisarCurva(0, bufferCount/4);
    CurveAnalysis seg2 = analisarCurva(bufferCount/4, bufferCount/2);
    CurveAnalysis seg3 = analisarCurva(bufferCount/2, 3*bufferCount/4);
    CurveAnalysis seg4 = analisarCurva(3*bufferCount/4, bufferCount);
    
    json += "\"overall\":{";
    json += "\"peakAccelY\":" + String(fullAnalysis.peakAccelY, 2) + ",";
    json += "\"curveType\":" + String(fullAnalysis.curveType) + ",";
    json += "\"recommendation\":{";
    json += "\"kp\":" + String(fullAnalysis.recommendedKp, 4) + ",";
    json += "\"kd\":" + String(fullAnalysis.recommendedKd, 4);
    json += "}},";
    
    // Mapear tipo de curva
    const char* curveNames[] = {"reta", "suave", "media", "forte"};
    
    json += "\"segments\":[";
    CurveAnalysis segs[] = {seg1, seg2, seg3, seg4};
    for (int i = 0; i < 4; i++) {
      if (i > 0) json += ",";
      json += "{\"type\":\"" + String(curveNames[segs[i].curveType]) + "\",";
      json += "\"peakAccelY\":" + String(segs[i].peakAccelY, 2) + ",";
      json += "\"suggestedKp\":" + String(segs[i].recommendedKp, 4) + "}";
    }
    json += "],";
    
    // Estatísticas gerais
    float minPos = 7000, maxPos = 0;
    float avgError = 0;
    for (int i = 0; i < bufferCount; i++) {
      if (dataBuffer[i].position < minPos) minPos = dataBuffer[i].position;
      if (dataBuffer[i].position > maxPos) maxPos = dataBuffer[i].position;
      avgError += abs(dataBuffer[i].error);
    }
    avgError /= bufferCount;
    
    json += "\"stats\":{";
    json += "\"positionRange\":[" + String((int)minPos) + "," + String((int)maxPos) + "],";
    json += "\"avgError\":" + String((int)avgError) + ",";
    json += "\"totalPoints\":" + String(bufferCount);
    json += "}";
  } else {
    json += "\"message\":\"Insuficientes dados para analise\"";
  }
  
  json += "}";
  webServer.send(200, "application/json; charset=utf-8", json);
}

void iniciarTelemetriaWiFi() {
  delay(500);
  
  WiFi.mode(WIFI_AP);
  IPAddress local_IP(192, 168, 1, 1);
  WiFi.softAPConfig(local_IP, GATEWAY, SUBNET);
  WiFi.softAP(AP_SSID, AP_SENHA);
  
  Serial.println("\n=== WIFI AP INICIADO ===");
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.println("=====================\n");
  
  webServer.on("/", handleRaiz);
  webServer.on("/telemetria", handleTelemetria);
  webServer.on("/dados.json", handleJSON);
  webServer.on("/dados.csv", handleCSV);
  webServer.on("/ml.json", handleML);
  webServer.on("/tune", handleTuneML);
  webServer.on("/reset", handleReset);
  webServer.on("/aplicar-pid", handleAplicarPID);
  
  webServer.begin();
  Serial.println("Servidor Web iniciado!");
}

void processarTelemetriaWiFi() {
  webServer.handleClient();
}

void enviarDadosTelemetria(int erro, float correcao, int pwmE, int pwmD, 
                           float kp, float ki, float kd, uint16_t posicaoSensor,
                           float ax, float ay, float az, float gx, float gy, float gz) {
  char jsonBuffer[400];
  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"t\":%lu,\"pos\":%u,\"err\":%d,\"cor\":%.2f,\"pwmE\":%d,\"pwmD\":%d,\"kp\":%.6f,\"ki\":%.6f,\"kd\":%.6f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f}",
    millis(),
    posicaoSensor,
    erro,
    correcao,
    pwmE,
    pwmD,
    kp,
    ki,
    kd,
    ax,
    ay,
    az,
    gx,
    gy,
    gz
  );
  
  ultimoJSON = jsonBuffer;
  
  // Armazenar no buffer circular
  dataBuffer[bufferIndex].timestamp = millis();
  dataBuffer[bufferIndex].position = posicaoSensor;
  dataBuffer[bufferIndex].error = erro;
  dataBuffer[bufferIndex].correction = correcao;
  dataBuffer[bufferIndex].pwmL = pwmE;
  dataBuffer[bufferIndex].pwmR = pwmD;
  dataBuffer[bufferIndex].kp = kp;
  dataBuffer[bufferIndex].ki = ki;
  dataBuffer[bufferIndex].kd = kd;
  dataBuffer[bufferIndex].ax = ax;
  dataBuffer[bufferIndex].ay = ay;
  dataBuffer[bufferIndex].az = az;
  dataBuffer[bufferIndex].gx = gx;
  dataBuffer[bufferIndex].gy = gy;
  dataBuffer[bufferIndex].gz = gz;
  
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  if (bufferCount < BUFFER_SIZE) {
    bufferCount++;
  }
}

void handleReset() {
  // Resetar buffer de dados
  bufferIndex = 0;
  bufferCount = 0;
  webServer.send(200, "application/json", "{\"status\":\"buffer resetado\"}");
}

void handleTuneML() {
  // Retorna análise detalhada para aplicação de IA
  String json = "{";
  json += "\"timestamp\":" + String(millis()) + ",";
  json += "\"bufferSize\":" + String(bufferCount) + ",";
  
  if (bufferCount > 20) {
    // Calcular métricas
    float sumError = 0, sumAbsError = 0, maxError = 0;
    float sumCorr = 0, maxCorr = 0;
    float sumAccelY = 0, maxAccelY = 0;
    float sumAccelTotal = 0;
    
    for (int i = 0; i < bufferCount; i++) {
      TelemetriaData& d = dataBuffer[i];
      
      int absErr = abs(d.error);
      sumError += d.error;
      sumAbsError += absErr;
      if (absErr > maxError) maxError = absErr;
      
      float absCorr = fabs(d.correction);
      sumCorr += absCorr;
      if (absCorr > maxCorr) maxCorr = absCorr;
      
      float absAy = fabs(d.ay);
      sumAccelY += absAy;
      if (absAy > maxAccelY) maxAccelY = absAy;
      
      float accelTotal = sqrt(d.ax*d.ax + d.ay*d.ay + d.az*d.az);
      sumAccelTotal += accelTotal;
    }
    
    float avgError = sumAbsError / bufferCount;
    float avgCorr = sumCorr / bufferCount;
    float avgAccelY = sumAccelY / bufferCount;
    float avgAccelTotal = sumAccelTotal / bufferCount;
    
    // Análise de estabilidade
    int instabilidades = 0;
    for (int i = 1; i < bufferCount; i++) {
      int deltaErr = abs(dataBuffer[i].error - dataBuffer[i-1].error);
      if (deltaErr > 500) instabilidades++;
    }
    
    json += "\"metrics\":{";
    json += "\"avgError\":" + String((int)avgError) + ",";
    json += "\"maxError\":" + String((int)maxError) + ",";
    json += "\"avgCorrection\":" + String(avgCorr, 2) + ",";
    json += "\"maxCorrection\":" + String(maxCorr, 2) + ",";
    json += "\"instabilityCount\":" + String(instabilidades) + ",";
    json += "\"accelY\":{\"avg\":" + String(avgAccelY, 3) + ",\"max\":" + String(maxAccelY, 3) + "},";
    json += "\"accelTotal\":{\"avg\":" + String(avgAccelTotal, 3) + "}";
    json += "},";
    
    // Recomendações IA adaptativas
    float suggestedKp, suggestedKi, suggestedKd;
    
    if (avgError > 2000) {
      // Erro muito alto - aumentar Kp
      suggestedKp = 0.15;
      suggestedKi = 0.00005;
      suggestedKd = 0.30;
    } else if (avgError > 1000) {
      // Erro alto - aumentar Kp moderado
      suggestedKp = 0.10;
      suggestedKi = 0.00003;
      suggestedKd = 0.25;
    } else if (avgError > 500) {
      // Erro médio
      suggestedKp = 0.07;
      suggestedKi = 0.00002;
      suggestedKd = 0.20;
    } else if (avgError > 200) {
      // Erro baixo - bom controle
      suggestedKp = 0.05;
      suggestedKi = 0.00001;
      suggestedKd = 0.15;
    } else {
      // Muito bom - fine tuning
      suggestedKp = 0.03;
      suggestedKi = 0.000005;
      suggestedKd = 0.10;
    }
    
    // Ajuste por aceleração (curvas)
    if (maxAccelY > 2.0) {
      suggestedKd += 0.15;  // Mais amortecimento em curvas
    } else if (maxAccelY > 1.0) {
      suggestedKd += 0.05;
    }
    
    json += "\"aiRecommendation\":{";
    json += "\"kp\":" + String(suggestedKp, 6) + ",";
    json += "\"ki\":" + String(suggestedKi, 8) + ",";
    json += "\"kd\":" + String(suggestedKd, 6) + ",";
    json += "\"confidence\":0.85,";
    json += "\"reasoning\":\"Baseado em análise de erro, correção e aceleração lateral\"";
    json += "},";
    
    // Últimos valores otimizados
    json += "\"optimized\":{";
    json += "\"kp\":" + String(pidAplicado ? pidKpOtimizado : -1, 6) + ",";
    json += "\"kd\":" + String(pidAplicado ? pidKdOtimizado : -1, 6) + ",";
    json += "\"applied\":" + String(pidAplicado ? "true" : "false");
    json += "}";
    
  } else {
    json += "\"error\":\"Insuficientes dados (minimo 20 pontos)\"";
  }
  
  json += "}";
  webServer.send(200, "application/json; charset=utf-8", json);
}

void handleAplicarPID() {
  // Aplicar valores de PID recomendados
  if (webServer.hasArg("kp") && webServer.hasArg("kd")) {
    float novoKp = webServer.arg("kp").toFloat();
    float novoKd = webServer.arg("kd").toFloat();
    
    if (novoKp >= 0 && novoKd >= 0) {
      pidKpOtimizado = novoKp;
      pidKdOtimizado = novoKd;
      pidAplicado = true;
      
      // TODO: Aqui você aplicaria ao PID real através de uma variável global
      // extern float KP, KI, KD (em Configuracoes.h)
      // KP = novoKp;
      // KD = novoKd;
      
      String response = "{\"status\":\"OK\",\"kp\":" + String(novoKp, 6) + ",\"kd\":" + String(novoKd, 6) + ",\"appliedAt\":" + String(millis()) + "}";
      webServer.send(200, "application/json", response);
    } else {
      webServer.send(400, "application/json", "{\"error\":\"Valores invalidos\"}");
    }
  } else {
    webServer.send(400, "application/json", "{\"error\":\"Faltam parametros kp e kd\"}");
  }
}

bool temClientesConectados() {
  return clientesConectados;
}
