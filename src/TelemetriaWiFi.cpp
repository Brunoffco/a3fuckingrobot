#include <WiFi.h>
#include <WebServer.h>
#include "TelemetriaWiFi.h"
#include "LSM6DS3.h"
#include "Configuracoes.h"

const char* AP_SSID = "RoboSeguidor";
const char* AP_SENHA = "";
const IPAddress GATEWAY(192, 168, 1, 1);
const IPAddress SUBNET(255, 255, 255, 0);

WebServer webServer(80);
bool clientesConectados = false;

String ultimoJSON = "";
unsigned long ultimoEnvioSSE = 0;
const unsigned long INTERVALO_SSE_MS = 100;

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

const int BUFFER_SIZE = 1000;
TelemetriaData dataBuffer[BUFFER_SIZE];
int bufferIndex = 0;
int bufferCount = 0;

float pidKpOtimizado = -1.0;
float pidKdOtimizado = -1.0;
bool pidAplicado = false;

struct CurveAnalysis {
  float avgAccelY;
  float peakAccelY;
  int curveType;
  float recommendedKp;
  float recommendedKd;
};

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
<html lang="pt-BR">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>RoboSeguidor — Telemetria</title>
  <style>
    :root {
      --navy:   #0d1b2a;
      --navy2:  #142233;
      --navy3:  #1c2f45;
      --blue:   #1a56db;
      --blue2:  #2563eb;
      --blue3:  #3b82f6;
      --white:  #f0f4f8;
      --dim:    #8fa3b8;
      --rule:   #1e3352;
      --black:  #070e17;
    }

    *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

    body {
      font-family: 'Courier New', 'Lucida Console', monospace;
      background: var(--black);
      color: var(--white);
      min-height: 100vh;
      padding: 0;
    }

    /* ── TOP BAR ── */
    .topbar {
      background: var(--navy);
      border-bottom: 1px solid var(--rule);
      padding: 0 32px;
      height: 52px;
      display: flex;
      align-items: center;
      justify-content: space-between;
      position: sticky;
      top: 0;
      z-index: 100;
    }
    .topbar-left {
      display: flex;
      align-items: center;
      gap: 20px;
    }
    .logo {
      font-size: 13px;
      font-weight: bold;
      letter-spacing: 3px;
      color: var(--white);
      text-transform: uppercase;
    }
    .logo span {
      color: var(--blue3);
    }
    .sep {
      width: 1px;
      height: 20px;
      background: var(--rule);
    }
    .version {
      font-size: 10px;
      color: var(--dim);
      letter-spacing: 1px;
    }
    .status-pill {
      display: flex;
      align-items: center;
      gap: 7px;
      font-size: 11px;
      font-weight: bold;
      letter-spacing: 1px;
      padding: 5px 12px;
      border-radius: 2px;
      border: 1px solid var(--rule);
    }
    .status-pill .dot {
      width: 7px;
      height: 7px;
      border-radius: 50%;
      background: var(--dim);
    }
    .status-pill.online .dot { background: #4ade80; box-shadow: 0 0 6px #4ade80; }
    .status-pill.offline .dot { background: #f87171; }
    .status-pill.online { border-color: #1e3f2a; color: #4ade80; }
    .status-pill.offline { border-color: #3f1e1e; color: #f87171; }

    /* ── LAYOUT ── */
    .page {
      max-width: 1480px;
      margin: 0 auto;
      padding: 28px 28px 60px;
    }

    /* ── SECTION HEADER ── */
    .section-head {
      display: flex;
      align-items: center;
      gap: 12px;
      margin: 32px 0 14px;
    }
    .section-tag {
      font-size: 9px;
      font-weight: bold;
      letter-spacing: 2px;
      color: var(--navy);
      background: var(--blue3);
      padding: 3px 7px;
      border-radius: 2px;
    }
    .section-label {
      font-size: 11px;
      letter-spacing: 2px;
      color: var(--dim);
      text-transform: uppercase;
    }
    .section-rule {
      flex: 1;
      height: 1px;
      background: var(--rule);
    }

    /* ── METRIC GRID ── */
    .metric-grid {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 12px;
    }
    @media (max-width: 1100px) { .metric-grid { grid-template-columns: repeat(2, 1fr); } }
    @media (max-width: 600px)  { .metric-grid { grid-template-columns: 1fr; } }

    .card {
      background: var(--navy2);
      border: 1px solid var(--rule);
      border-radius: 3px;
      padding: 18px 20px 16px;
    }
    .card-label {
      font-size: 9px;
      letter-spacing: 2px;
      color: var(--dim);
      text-transform: uppercase;
      margin-bottom: 10px;
    }
    .card-value {
      font-size: 36px;
      font-weight: bold;
      color: var(--white);
      line-height: 1;
      margin-bottom: 12px;
      letter-spacing: -1px;
    }
    .card-sub {
      font-size: 10px;
      color: var(--dim);
      letter-spacing: 0.5px;
    }

    /* ── TRACK BAR ── */
    .track-bar {
      height: 4px;
      background: var(--navy3);
      border-radius: 2px;
      margin-top: 10px;
      overflow: hidden;
    }
    .track-fill {
      height: 100%;
      background: var(--blue2);
      border-radius: 2px;
      width: 50%;
      transition: width 0.15s;
    }

    /* ── SENSOR ARRAY ── */
    .sensor-row {
      display: flex;
      gap: 8px;
      margin-top: 10px;
    }
    .sensor-node {
      flex: 1;
      height: 36px;
      border: 1px solid var(--rule);
      border-radius: 2px;
      display: flex;
      align-items: center;
      justify-content: center;
      font-size: 9px;
      font-weight: bold;
      letter-spacing: 1px;
      color: var(--dim);
      background: var(--navy);
      transition: background 0.1s, color 0.1s;
    }
    .sensor-node.active {
      background: var(--blue2);
      color: var(--white);
      border-color: var(--blue3);
    }

    /* ── IMU ROWS ── */
    .imu-grid {
      display: grid;
      grid-template-columns: repeat(4, 1fr);
      gap: 12px;
    }
    @media (max-width: 1100px) { .imu-grid { grid-template-columns: repeat(2, 1fr); } }

    .imu-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin: 8px 0 4px;
      font-size: 11px;
    }
    .imu-row .axis { color: var(--dim); letter-spacing: 1px; }
    .imu-row .val  { color: var(--white); font-weight: bold; }

    .pid-line {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin: 7px 0;
      font-size: 11px;
    }
    .pid-line .k { color: var(--dim); letter-spacing: 1px; }
    .pid-line .v { color: var(--white); font-weight: bold; }

    .pid-badge {
      display: none;
      margin-top: 10px;
      font-size: 9px;
      font-weight: bold;
      letter-spacing: 2px;
      color: #4ade80;
      border: 1px solid #1e3f2a;
      padding: 4px 10px;
      border-radius: 2px;
      text-align: center;
    }

    /* ── CANVAS ── */
    .canvas-wrap {
      background: var(--navy2);
      border: 1px solid var(--rule);
      border-radius: 3px;
      padding: 16px;
      position: relative;
    }
    .canvas-wrap canvas {
      display: block;
      width: 100%;
      border-radius: 2px;
    }
    .canvas-btn {
      position: absolute;
      top: 14px;
      right: 14px;
    }

    /* ── ML PANEL ── */
    .ml-wrap {
      background: var(--navy2);
      border: 1px solid var(--rule);
      border-radius: 3px;
      padding: 20px;
    }
    .ml-grid {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      gap: 14px;
    }
    @media (max-width: 700px) { .ml-grid { grid-template-columns: 1fr; } }

    .ml-block {
      background: var(--navy);
      border: 1px solid var(--rule);
      border-left: 3px solid var(--blue2);
      border-radius: 2px;
      padding: 14px 16px;
      font-size: 11px;
      line-height: 2;
    }
    .ml-block-title {
      font-size: 9px;
      font-weight: bold;
      letter-spacing: 2px;
      color: var(--blue3);
      text-transform: uppercase;
      margin-bottom: 8px;
    }
    .ml-block strong { color: var(--white); }

    /* ── BUTTONS ── */
    .btn {
      background: transparent;
      color: var(--white);
      border: 1px solid var(--blue2);
      padding: 8px 18px;
      border-radius: 2px;
      font-family: inherit;
      font-size: 10px;
      font-weight: bold;
      letter-spacing: 2px;
      text-transform: uppercase;
      cursor: pointer;
      transition: background 0.15s, color 0.15s;
    }
    .btn:hover {
      background: var(--blue2);
      color: var(--white);
    }
    .btn-primary {
      background: var(--blue2);
      color: var(--white);
      border-color: var(--blue2);
    }
    .btn-primary:hover { background: var(--blue3); border-color: var(--blue3); }
    .btn-apply {
      display: block;
      width: 100%;
      margin-top: 10px;
      background: var(--navy3);
      border-color: var(--blue3);
      color: var(--blue3);
    }
    .btn-apply:hover { background: var(--blue2); color: var(--white); border-color: var(--blue2); }

    .controls-row {
      display: flex;
      gap: 10px;
      margin-top: 24px;
      justify-content: center;
      flex-wrap: wrap;
    }

    .empty-state {
      text-align: center;
      color: var(--dim);
      font-size: 11px;
      padding: 20px 0;
      letter-spacing: 1px;
    }
  </style>
</head>
<body>

  <div class="topbar">
    <div class="topbar-left">
      <div class="logo">ROBO<span>SEGUIDOR</span></div>
      <div class="sep"></div>
      <div class="version">TELEMETRIA v2.0</div>
    </div>
    <div class="status-pill online" id="statusPill">
      <div class="dot"></div>
      <span id="statusText">CONECTADO</span>
    </div>
  </div>

  <div class="page">

    <!-- B1: VELOCIDADE -->
    <div class="section-head">
      <div class="section-tag">B1</div>
      <div class="section-label">Motores &amp; Posição</div>
      <div class="section-rule"></div>
    </div>
    <div class="metric-grid">
      <div class="card">
        <div class="card-label">Motor Esquerda — PWM</div>
        <div class="card-value" id="rpmE">0</div>
        <div class="track-bar"><div class="track-fill" id="barE" style="width:0%"></div></div>
        <div class="card-sub" style="margin-top:8px">0 – 255</div>
      </div>
      <div class="card">
        <div class="card-label">Motor Direita — PWM</div>
        <div class="card-value" id="rpmD">0</div>
        <div class="track-bar"><div class="track-fill" id="barD" style="width:0%"></div></div>
        <div class="card-sub" style="margin-top:8px">Base: 220</div>
      </div>
      <div class="card">
        <div class="card-label">Posição Sensor</div>
        <div class="card-value" id="posicao">3500</div>
        <div class="track-bar"><div class="track-fill" id="posBar" style="width:50%"></div></div>
        <div class="card-sub" style="margin-top:8px">Range: 0 – 7000</div>
      </div>
      <div class="card">
        <div class="card-label">Erro PID</div>
        <div class="card-value" id="erro">0</div>
        <div class="card-sub">Correção: <span id="correcao">0.0</span></div>
      </div>
    </div>

    <!-- B2: SENSORES -->
    <div class="section-head">
      <div class="section-tag">B2</div>
      <div class="section-label">Array QRE — 8 sensores IR</div>
      <div class="section-rule"></div>
    </div>
    <div class="card">
      <div class="card-label">Detecção de linha</div>
      <div class="sensor-row">
        <div class="sensor-node" id="s0">S0</div>
        <div class="sensor-node" id="s1">S1</div>
        <div class="sensor-node" id="s2">S2</div>
        <div class="sensor-node" id="s3">S3</div>
        <div class="sensor-node" id="s4">S4</div>
        <div class="sensor-node" id="s5">S5</div>
        <div class="sensor-node" id="s6">S6</div>
        <div class="sensor-node" id="s7">S7</div>
      </div>
    </div>

    <!-- B3: IMU + PID -->
    <div class="section-head">
      <div class="section-tag">B3</div>
      <div class="section-label">IMU LSM6DS3 &amp; Controle PID</div>
      <div class="section-rule"></div>
    </div>
    <div class="imu-grid">
      <div class="card">
        <div class="card-label">Aceleração (g)</div>
        <div class="imu-row"><span class="axis">aX</span><span class="val" id="ax">0.00</span></div>
        <div class="track-bar"><div class="track-fill" id="axBar" style="width:50%"></div></div>
        <div class="imu-row"><span class="axis">aY</span><span class="val" id="ay">0.00</span></div>
        <div class="track-bar"><div class="track-fill" id="ayBar" style="width:50%"></div></div>
        <div class="imu-row"><span class="axis">aZ</span><span class="val" id="az">0.00</span></div>
        <div class="track-bar"><div class="track-fill" id="azBar" style="width:50%"></div></div>
      </div>
      <div class="card">
        <div class="card-label">Giroscópio (°/s)</div>
        <div class="imu-row"><span class="axis">gX</span><span class="val" id="gx">0.00</span></div>
        <div class="track-bar"><div class="track-fill" id="gxBar" style="width:50%"></div></div>
        <div class="imu-row"><span class="axis">gY</span><span class="val" id="gy">0.00</span></div>
        <div class="track-bar"><div class="track-fill" id="gyBar" style="width:50%"></div></div>
        <div class="imu-row"><span class="axis">gZ</span><span class="val" id="gz">0.00</span></div>
        <div class="track-bar"><div class="track-fill" id="gzBar" style="width:50%"></div></div>
      </div>
      <div class="card">
        <div class="card-label">Ganhos PID ativos</div>
        <div class="pid-line"><span class="k">Kp</span><span class="v" id="kp">0.02</span></div>
        <div class="track-bar"><div class="track-fill" style="width:8%"></div></div>
        <div class="pid-line"><span class="k">Ki</span><span class="v" id="ki">0.000000</span></div>
        <div class="track-bar"><div class="track-fill" style="width:0%"></div></div>
        <div class="pid-line"><span class="k">Kd</span><span class="v" id="kd">0.10</span></div>
        <div class="track-bar"><div class="track-fill" style="width:40%"></div></div>
        <div class="pid-badge" id="pidAppliedBadge">✓ ML APLICADO</div>
      </div>
      <div class="card">
        <div class="card-label">PWM Motores</div>
        <div class="imu-row"><span class="axis">Motor E</span><span class="val" id="pwmE">0</span></div>
        <div class="track-bar"><div class="track-fill" id="pwmEBar" style="width:0%"></div></div>
        <div class="imu-row"><span class="axis">Motor D</span><span class="val" id="pwmD">0</span></div>
        <div class="track-bar"><div class="track-fill" id="pwmDBar" style="width:0%"></div></div>
      </div>
    </div>

    <!-- B4: PISTA -->
    <div class="section-head">
      <div class="section-tag">B4</div>
      <div class="section-label">Pista — posição do sensor em tempo real</div>
      <div class="section-rule"></div>
    </div>
    <div class="canvas-wrap">
      <div class="canvas-btn">
        <button class="btn" onclick="resetarTrilha()">RESETAR</button>
      </div>
      <canvas id="trackCanvas" width="1200" height="320"></canvas>
    </div>

    <!-- B5: ML -->
    <div class="section-head">
      <div class="section-tag">B5</div>
      <div class="section-label">Análise ML — recomendação de PID</div>
      <div class="section-rule"></div>
    </div>
    <div class="ml-wrap">
      <div id="mlAnalysis">
        <div class="empty-state">AGUARDANDO DADOS...</div>
      </div>
    </div>

    <div class="controls-row">
      <button class="btn" onclick="baixarDados()">BAIXAR CSV</button>
      <button class="btn btn-primary" onclick="atualizarML()">ATUALIZAR ML</button>
      <button class="btn" onclick="limparDados()">LIMPAR</button>
    </div>

  </div><!-- /page -->

  <script>
    const dados = [];
    const canvas = document.getElementById('trackCanvas');
    const ctx = canvas.getContext('2d');

    // ── CANVAS ──────────────────────────────────────────────
    function desenharPista() {
      const W = canvas.width, H = canvas.height;
      ctx.fillStyle = '#070e17';
      ctx.fillRect(0, 0, W, H);

      // grid
      ctx.strokeStyle = '#101e2f';
      ctx.lineWidth = 1;
      for (let x = 0; x <= W; x += 60) {
        ctx.beginPath(); ctx.moveTo(x, 0); ctx.lineTo(x, H); ctx.stroke();
      }
      for (let y = 0; y <= H; y += 40) {
        ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke();
      }

      // track bounds
      const mid = H / 2;
      ctx.strokeStyle = '#1e3352';
      ctx.lineWidth = 1;
      ctx.setLineDash([6, 6]);
      ctx.beginPath(); ctx.moveTo(0, mid - 50); ctx.lineTo(W, mid - 50); ctx.stroke();
      ctx.beginPath(); ctx.moveTo(0, mid + 50); ctx.lineTo(W, mid + 50); ctx.stroke();
      ctx.setLineDash([]);

      // centre
      ctx.strokeStyle = '#1a3a5c';
      ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(0, mid); ctx.lineTo(W, mid); ctx.stroke();

      // labels
      ctx.fillStyle = '#1e3352';
      ctx.font = '9px Courier New';
      ctx.fillText('0', 4, H - 6);
      ctx.fillText('7000', W - 34, H - 6);
      ctx.fillText('3500', W / 2 - 12, mid - 6);
    }

    function desenharTrajetoria() {
      if (dados.length === 0) { desenharPista(); return; }
      desenharPista();

      const W = canvas.width, H = canvas.height, mid = H / 2;

      ctx.strokeStyle = '#2563eb';
      ctx.lineWidth = 2;
      ctx.beginPath();
      for (let i = 0; i < dados.length; i++) {
        const x = (i / Math.max(1, dados.length - 1)) * W;
        const posY = mid - ((dados[i].pos - 3500) / 3500) * (mid - 14);
        i === 0 ? ctx.moveTo(x, posY) : ctx.lineTo(x, posY);
      }
      ctx.stroke();

      // current dot
      const last = dados[dados.length - 1];
      const lx = W - 12;
      const ly = mid - ((last.pos - 3500) / 3500) * (mid - 14);
      ctx.fillStyle = '#f0f4f8';
      ctx.beginPath();
      ctx.arc(lx, ly, 5, 0, 2 * Math.PI);
      ctx.fill();

      ctx.fillStyle = '#3b82f6';
      ctx.font = 'bold 10px Courier New';
      ctx.fillText('POS ' + last.pos, 8, H - 8);
    }

    // ── ML PANEL ────────────────────────────────────────────
    function atualizarML() {
      fetch('/tune')
        .then(r => r.json())
        .then(data => {
          const el = document.getElementById('mlAnalysis');
          if (data.error) {
            el.innerHTML = '<div class="empty-state">⚠ ' + data.error + '</div>';
            return;
          }
          if (!data.metrics) {
            el.innerHTML = '<div class="empty-state">SEM DADOS SUFICIENTES</div>';
            return;
          }

          const m = data.metrics;
          const rec = data.aiRecommendation;
          const opt = data.optimized;

          let html = '<div class="ml-grid">';

          // bloco 1: métricas
          html += '<div class="ml-block">';
          html += '<div class="ml-block-title">Métricas actuais</div>';
          html += 'Erro médio &nbsp;&nbsp;: <strong>' + m.avgError + '</strong><br>';
          html += 'Erro máx &nbsp;&nbsp;&nbsp;: <strong>' + m.maxError + '</strong><br>';
          html += 'Instabilid. &nbsp;: <strong>' + m.instabilityCount + '</strong><br>';
          html += 'Acel. Y (avg) : <strong>' + m.accelY.avg.toFixed(3) + ' g</strong><br>';
          html += 'Acel. Y (max) : <strong>' + m.accelY.max.toFixed(3) + ' g</strong>';
          html += '</div>';

          // bloco 2: recomendação
          if (rec) {
            html += '<div class="ml-block">';
            html += '<div class="ml-block-title">Recomendação ML</div>';
            html += 'Kp : <strong id="recKp">' + rec.kp.toFixed(6) + '</strong><br>';
            html += 'Ki : <strong>' + rec.ki.toFixed(8) + '</strong><br>';
            html += 'Kd : <strong id="recKd">' + rec.kd.toFixed(6) + '</strong><br>';
            html += 'Confiança : <strong>' + (rec.confidence * 100).toFixed(0) + '%</strong>';
            html += '</div>';

            // bloco 3: aplicar
            html += '<div class="ml-block">';
            html += '<div class="ml-block-title">Aplicar ao robô</div>';
            html += 'Ao confirmar, Kp/Ki/Kd são escritos nas variáveis globais imediatamente e gravados no NVS.<br>';
            html += '<button class="btn btn-apply" onclick="aplicarRecomendacaoIA('
              + rec.kp.toFixed(6) + ','
              + rec.ki.toFixed(8) + ','
              + rec.kd.toFixed(6) + ')">✓ APLICAR RECOMENDAÇÃO</button>';
            html += '</div>';
          }

          // bloco 4: histórico
          html += '<div class="ml-block">';
          html += '<div class="ml-block-title">Histórico</div>';
          html += 'Buffer : <strong>' + data.bufferSize + '</strong> pontos<br>';
          if (opt && opt.kp > 0) {
            html += 'Último Kp : <strong>' + opt.kp.toFixed(6) + '</strong><br>';
            html += 'Último Kd : <strong>' + opt.kd.toFixed(6) + '</strong><br>';
            html += 'Status &nbsp;&nbsp;&nbsp;: <strong style="color:#4ade80">' + (opt.applied ? '✓ Aplicado' : 'Pendente') + '</strong>';
          } else {
            html += 'Nenhuma otimização aplicada ainda.';
          }
          html += '</div>';

          html += '</div>';
          el.innerHTML = html;
        })
        .catch(() => {
          document.getElementById('mlAnalysis').innerHTML =
            '<div class="empty-state">ERRO AO CARREGAR ANÁLISE</div>';
        });
    }

    function aplicarRecomendacaoIA(kp, ki, kd) {
      if (!confirm(
        'Aplicar ao robô agora?\n\n' +
        'Kp = ' + kp.toFixed(6) + '\n' +
        'Ki = ' + ki.toFixed(8) + '\n' +
        'Kd = ' + kd.toFixed(6) + '\n\n' +
        'Os valores serão gravados no NVS.'
      )) return;

      fetch('/aplicar-pid?kp=' + kp + '&ki=' + ki + '&kd=' + kd)
        .then(r => r.json())
        .then(data => {
          if (data.status === 'OK') {
            document.getElementById('kp').textContent = parseFloat(data.kp).toFixed(6);
            document.getElementById('ki').textContent = parseFloat(data.ki).toFixed(8);
            document.getElementById('kd').textContent = parseFloat(data.kd).toFixed(6);
            document.getElementById('pidAppliedBadge').style.display = 'block';
            atualizarML();
          } else {
            alert('Erro: ' + data.error);
          }
        })
        .catch(() => alert('Erro de conexão'));
    }

    // ── DATA POLL ────────────────────────────────────────────
    function atualizarDados() {
      fetch('/dados.json')
        .then(r => r.json())
        .then(data => {
          if (!data || !data.t) return;

          document.getElementById('posicao').textContent = data.pos;
          document.getElementById('erro').textContent    = data.err;
          document.getElementById('correcao').textContent = data.cor.toFixed(1);
          document.getElementById('rpmE').textContent    = data.pwmE;
          document.getElementById('rpmD').textContent    = data.pwmD;
          document.getElementById('pwmE').textContent    = data.pwmE;
          document.getElementById('pwmD').textContent    = data.pwmD;
          document.getElementById('kp').textContent      = data.kp.toFixed(6);
          document.getElementById('ki').textContent      = data.ki.toFixed(8);
          document.getElementById('kd').textContent      = data.kd.toFixed(6);
          document.getElementById('ax').textContent      = (data.ax||0).toFixed(2);
          document.getElementById('ay').textContent      = (data.ay||0).toFixed(2);
          document.getElementById('az').textContent      = (data.az||0).toFixed(2);
          document.getElementById('gx').textContent      = (data.gx||0).toFixed(2);
          document.getElementById('gy').textContent      = (data.gy||0).toFixed(2);
          document.getElementById('gz').textContent      = (data.gz||0).toFixed(2);

          document.getElementById('posBar').style.width  = ((data.pos/7000)*100)+'%';
          document.getElementById('barE').style.width    = ((Math.abs(data.pwmE)/255)*100)+'%';
          document.getElementById('barD').style.width    = ((Math.abs(data.pwmD)/255)*100)+'%';
          document.getElementById('pwmEBar').style.width = ((Math.abs(data.pwmE)/255)*100)+'%';
          document.getElementById('pwmDBar').style.width = ((Math.abs(data.pwmD)/255)*100)+'%';

          // IMU bars: map ±2g / ±250°s to 0-100%
          const acclPct = v => Math.min(100, Math.max(0, ((parseFloat(v)+2)/4)*100));
          const giroPct = v => Math.min(100, Math.max(0, ((parseFloat(v)+250)/500)*100));
          document.getElementById('axBar').style.width = acclPct(data.ax)+'%';
          document.getElementById('ayBar').style.width = acclPct(data.ay)+'%';
          document.getElementById('azBar').style.width = acclPct(data.az)+'%';
          document.getElementById('gxBar').style.width = giroPct(data.gx)+'%';
          document.getElementById('gyBar').style.width = giroPct(data.gy)+'%';
          document.getElementById('gzBar').style.width = giroPct(data.gz)+'%';

          const pill = document.getElementById('statusPill');
          const txt  = document.getElementById('statusText');
          pill.className = 'status-pill online';
          txt.textContent = 'CONECTADO';

          dados.push(data);
          if (dados.length > 500) dados.shift();
          desenharTrajetoria();
        })
        .catch(() => {
          document.getElementById('statusPill').className = 'status-pill offline';
          document.getElementById('statusText').textContent = 'OFFLINE';
        });
    }

    function resetarTrilha() {
      dados.length = 0;
      desenharPista();
      fetch('/reset').catch(() => {});
    }

    function baixarDados() {
      fetch('/dados.csv')
        .then(r => r.text())
        .then(csv => {
          const a = document.createElement('a');
          a.href = URL.createObjectURL(new Blob([csv], {type:'text/csv'}));
          a.download = 'telemetria_' + new Date().toISOString().slice(0,10) + '.csv';
          a.click();
        });
    }

    function limparDados() {
      dados.length = 0;
      desenharPista();
      document.getElementById('pidAppliedBadge').style.display = 'none';
    }

    desenharPista();
    atualizarML();
    setInterval(atualizarDados, 200);
  </script>
</body>
</html>
)rawliteral";

  webServer.send(200, "text/html", html);
}

void handleTelemetria() {
  clientesConectados = true;
  webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webServer.send(200, "text/event-stream", "");

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
  analysis.avgAccelY   = 0;
  analysis.peakAccelY  = 0;

  if (endIdx <= startIdx) {
    analysis.curveType      = 0;
    analysis.recommendedKp  = 0.05;
    analysis.recommendedKd  = 0.2;
    return analysis;
  }

  int count = 0;
  for (int i = startIdx; i < endIdx && i < bufferCount; i++) {
    analysis.avgAccelY += dataBuffer[i].ay;
    if (fabs(dataBuffer[i].ay) > fabs(analysis.peakAccelY))
      analysis.peakAccelY = dataBuffer[i].ay;
    count++;
  }
  if (count > 0) analysis.avgAccelY /= count;

  float peak = fabs(analysis.peakAccelY);
  if      (peak < 0.5) { analysis.curveType = 0; analysis.recommendedKp = 0.03; analysis.recommendedKd = 0.15; }
  else if (peak < 1.0) { analysis.curveType = 1; analysis.recommendedKp = 0.05; analysis.recommendedKd = 0.20; }
  else if (peak < 1.5) { analysis.curveType = 2; analysis.recommendedKp = 0.07; analysis.recommendedKd = 0.25; }
  else                 { analysis.curveType = 3; analysis.recommendedKp = 0.10; analysis.recommendedKd = 0.35; }

  return analysis;
}

void handleML() {
  String json = "{";
  json += "\"bufferSize\":" + String(bufferCount) + ",";

  if (bufferCount > 10) {
    CurveAnalysis full = analisarCurva(0, bufferCount);
    CurveAnalysis s1   = analisarCurva(0,              bufferCount/4);
    CurveAnalysis s2   = analisarCurva(bufferCount/4,  bufferCount/2);
    CurveAnalysis s3   = analisarCurva(bufferCount/2,  3*bufferCount/4);
    CurveAnalysis s4   = analisarCurva(3*bufferCount/4,bufferCount);

    json += "\"overall\":{";
    json += "\"peakAccelY\":" + String(full.peakAccelY,2) + ",";
    json += "\"curveType\":"  + String(full.curveType)    + ",";
    json += "\"recommendation\":{\"kp\":" + String(full.recommendedKp,4) + ",\"kd\":" + String(full.recommendedKd,4) + "}},";

    const char* names[] = {"reta","suave","media","forte"};
    CurveAnalysis segs[] = {s1,s2,s3,s4};
    json += "\"segments\":[";
    for (int i = 0; i < 4; i++) {
      if (i>0) json += ",";
      json += "{\"type\":\"" + String(names[segs[i].curveType]) + "\","
            + "\"peakAccelY\":" + String(segs[i].peakAccelY,2) + ","
            + "\"suggestedKp\":" + String(segs[i].recommendedKp,4) + "}";
    }
    json += "],";

    float minPos = 7000, maxPos = 0, avgErr = 0;
    for (int i = 0; i < bufferCount; i++) {
      if (dataBuffer[i].position < minPos) minPos = dataBuffer[i].position;
      if (dataBuffer[i].position > maxPos) maxPos = dataBuffer[i].position;
      avgErr += abs(dataBuffer[i].error);
    }
    avgErr /= bufferCount;
    json += "\"stats\":{\"positionRange\":[" + String((int)minPos) + "," + String((int)maxPos) + "],"
          + "\"avgError\":" + String((int)avgErr) + ","
          + "\"totalPoints\":" + String(bufferCount) + "}";
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
  Serial.print("SSID: "); Serial.println(AP_SSID);
  Serial.print("IP: ");   Serial.println(WiFi.softAPIP());
  Serial.println("=====================\n");

  webServer.on("/",           handleRaiz);
  webServer.on("/telemetria", handleTelemetria);
  webServer.on("/dados.json", handleJSON);
  webServer.on("/dados.csv",  handleCSV);
  webServer.on("/ml.json",    handleML);
  webServer.on("/tune",       handleTuneML);
  webServer.on("/reset",      handleReset);
  webServer.on("/aplicar-pid",handleAplicarPID);

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
    "{\"t\":%lu,\"pos\":%u,\"err\":%d,\"cor\":%.2f,\"pwmE\":%d,\"pwmD\":%d,"
    "\"kp\":%.6f,\"ki\":%.6f,\"kd\":%.6f,"
    "\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f}",
    millis(), posicaoSensor, erro, correcao, pwmE, pwmD,
    kp, ki, kd, ax, ay, az, gx, gy, gz
  );
  ultimoJSON = jsonBuffer;

  dataBuffer[bufferIndex] = {
    millis(), posicaoSensor, (int16_t)erro, correcao,
    (uint8_t)pwmE, (uint8_t)pwmD,
    kp, ki, kd,
    ax, ay, az, gx, gy, gz
  };
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  if (bufferCount < BUFFER_SIZE) bufferCount++;
}

void handleReset() {
  bufferIndex = 0;
  bufferCount = 0;
  webServer.send(200, "application/json", "{\"status\":\"buffer resetado\"}");
}

void handleTuneML() {
  String json = "{";
  json += "\"timestamp\":" + String(millis()) + ",";
  json += "\"bufferSize\":" + String(bufferCount) + ",";

  if (bufferCount > 20) {
    float sumAbsErr = 0, maxErr = 0, sumCorr = 0, maxCorr = 0;
    float sumAy = 0, maxAy = 0, sumAccel = 0;

    for (int i = 0; i < bufferCount; i++) {
      TelemetriaData& d = dataBuffer[i];
      int ae = abs(d.error);
      sumAbsErr += ae;
      if (ae > maxErr) maxErr = ae;
      float ac = fabs(d.correction);
      sumCorr += ac; if (ac > maxCorr) maxCorr = ac;
      float ay = fabs(d.ay);
      sumAy += ay; if (ay > maxAy) maxAy = ay;
      sumAccel += sqrt(d.ax*d.ax + d.ay*d.ay + d.az*d.az);
    }

    float avgErr   = sumAbsErr / bufferCount;
    float avgCorr  = sumCorr   / bufferCount;
    float avgAy    = sumAy     / bufferCount;
    float avgAccel = sumAccel  / bufferCount;

    int instab = 0;
    for (int i = 1; i < bufferCount; i++)
      if (abs(dataBuffer[i].error - dataBuffer[i-1].error) > 500) instab++;

    json += "\"metrics\":{"
          "\"avgError\":"        + String((int)avgErr)   + ","
          "\"maxError\":"        + String((int)maxErr)   + ","
          "\"avgCorrection\":"   + String(avgCorr,2)     + ","
          "\"maxCorrection\":"   + String(maxCorr,2)     + ","
          "\"instabilityCount\":" + String(instab)       + ","
          "\"accelY\":{\"avg\":" + String(avgAy,3) + ",\"max\":" + String(maxAy,3) + "},"
          "\"accelTotal\":{\"avg\":" + String(avgAccel,3) + "}"
          "},";

    float sKp, sKi, sKd;
    if      (avgErr > 2000) { sKp = 0.15; sKi = 0.00005; sKd = 0.30; }
    else if (avgErr > 1000) { sKp = 0.10; sKi = 0.00003; sKd = 0.25; }
    else if (avgErr > 500)  { sKp = 0.07; sKi = 0.00002; sKd = 0.20; }
    else if (avgErr > 200)  { sKp = 0.05; sKi = 0.00001; sKd = 0.15; }
    else                    { sKp = 0.03; sKi = 0.000005;sKd = 0.10; }

    if      (maxAy > 2.0) sKd += 0.15;
    else if (maxAy > 1.0) sKd += 0.05;

    json += "\"aiRecommendation\":{"
          "\"kp\":"         + String(sKp,6)  + ","
          "\"ki\":"         + String(sKi,8)  + ","
          "\"kd\":"         + String(sKd,6)  + ","
          "\"confidence\":0.85,"
          "\"reasoning\":\"Baseado em análise de erro médio e aceleração lateral\""
          "},";

    json += "\"optimized\":{"
          "\"kp\":" + String(pidAplicado ? pidKpOtimizado : -1.0f, 6) + ","
          "\"kd\":" + String(pidAplicado ? pidKdOtimizado : -1.0f, 6) + ","
          "\"applied\":" + String(pidAplicado ? "true" : "false")
          + "}";
  } else {
    json += "\"error\":\"Insuficientes dados (minimo 20 pontos)\"";
  }

  json += "}";
  webServer.send(200, "application/json; charset=utf-8", json);
}

void handleAplicarPID() {
  if (!webServer.hasArg("kp") || !webServer.hasArg("kd")) {
    webServer.send(400, "application/json", "{\"error\":\"Faltam parametros kp e kd\"}");
    return;
  }

  float novoKp = webServer.arg("kp").toFloat();
  float novoKd = webServer.arg("kd").toFloat();
  float novoKi = webServer.hasArg("ki") ? webServer.arg("ki").toFloat() : KI;

  if (novoKp < 0 || novoKd < 0 || novoKi < 0) {
    webServer.send(400, "application/json", "{\"error\":\"Valores invalidos (devem ser >= 0)\"}");
    return;
  }

  // Armazena localmente
  pidKpOtimizado = novoKp;
  pidKdOtimizado = novoKd;
  pidAplicado    = true;

  // Aplica nas variáveis globais do PID — efeito imediato no próximo loop()
  KP = novoKp;
  KI = novoKi;
  KD = novoKd;

  // Persiste no NVS para sobreviver a reboot
  salvarConfiguracoes();

  Serial.println("[ML] PID atualizado via interface web:");
  Serial.print("  KP = "); Serial.println(KP, 6);
  Serial.print("  KI = "); Serial.println(KI, 8);
  Serial.print("  KD = "); Serial.println(KD, 6);

  String resp = "{\"status\":\"OK\","
                "\"kp\":" + String(KP, 6) + ","
                "\"ki\":" + String(KI, 8) + ","
                "\"kd\":" + String(KD, 6) + ","
                "\"appliedAt\":" + String(millis()) + "}";
  webServer.send(200, "application/json", resp);
}

bool temClientesConectados() {
  return clientesConectados;
}
