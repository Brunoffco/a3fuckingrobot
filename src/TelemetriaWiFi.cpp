#include <WiFi.h>
#include <WebServer.h>
#include "TelemetriaWiFi.h"
#include "LSM6DS3.h"
#include "Configuracoes.h"

const char *AP_SSID = "RoboSeguidor";
const char *AP_SENHA = "";
const IPAddress GATEWAY(192, 168, 1, 1);
const IPAddress SUBNET(255, 255, 255, 0);

WebServer webServer(80);
bool clientesConectados = false;

String ultimoJSON = "";
unsigned long ultimoEnvioSSE = 0;
const unsigned long INTERVALO_SSE_MS = 100;

// Variáveis globais de odometria (atualize externamente)
float odometriaX = 0.0;
float odometriaY = 0.0;

struct TelemetriaData
{
  unsigned long timestamp;
  uint16_t position;
  int16_t error;
  float correction;
  uint8_t pwmL;
  uint8_t pwmR;
  float kp, ki, kd;
  float ax, ay, az;
  float gx, gy, gz;
  float x, y;               // ← coordenadas cartesianas
};

const int BUFFER_SIZE = 1000;
TelemetriaData dataBuffer[BUFFER_SIZE];
int bufferIndex = 0;
int bufferCount = 0;

float pidKpOtimizado = -1.0;
float pidKdOtimizado = -1.0;
bool pidAplicado = false;

struct CurveAnalysis
{
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

// ==================== PÁGINA HTML COM GRÁFICO XY ====================
void handleRaiz()
{
  String html = R"rawliteral(
<!DOCTYPE html>
<html lang="pt-BR">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>RoboSeguidor — Telemetria XY</title>
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
    .logo span { color: var(--blue3); }
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
    .page {
      max-width: 1480px;
      margin: 0 auto;
      padding: 28px 28px 60px;
    }
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
      background: #0a111f;
      border-radius: 2px;
    }
    .canvas-btn {
      position: absolute;
      top: 14px;
      right: 14px;
    }
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
    .coord-info {
      text-align: center;
      margin-top: 12px;
      font-size: 10px;
      color: var(--dim);
    }
  </style>
</head>
<body>
<div class="topbar">
  <div class="topbar-left">
    <div class="logo">ROBO<span>SEGUIDOR</span></div>
    <div class="sep"></div>
    <div class="version">TELEMETRIA XY</div>
  </div>
  <div class="status-pill online" id="statusPill">
    <div class="dot"></div>
    <span id="statusText">CONECTADO</span>
  </div>
</div>
<div class="page">
  <div class="section-head"><div class="section-tag">B1</div><div class="section-label">Motores &amp; Posição</div><div class="section-rule"></div></div>
  <div class="metric-grid">
    <div class="card"><div class="card-label">Motor Esquerda — PWM</div><div class="card-value" id="rpmE">0</div><div class="track-bar"><div class="track-fill" id="barE" style="width:0%"></div></div><div class="card-sub">0 – 255</div></div>
    <div class="card"><div class="card-label">Motor Direita — PWM</div><div class="card-value" id="rpmD">0</div><div class="track-bar"><div class="track-fill" id="barD" style="width:0%"></div></div><div class="card-sub">Base: 220</div></div>
    <div class="card"><div class="card-label">Posição Sensor</div><div class="card-value" id="posicao">3500</div><div class="track-bar"><div class="track-fill" id="posBar" style="width:50%"></div></div><div class="card-sub">Range: 0 – 7000</div></div>
    <div class="card"><div class="card-label">Erro PID</div><div class="card-value" id="erro">0</div><div class="card-sub">Correção: <span id="correcao">0.0</span></div></div>
  </div>
  <div class="section-head"><div class="section-tag">B2</div><div class="section-label">Array QRE — 8 sensores IR</div><div class="section-rule"></div></div>
  <div class="card"><div class="card-label">Detecção de linha</div><div class="sensor-row"><div class="sensor-node" id="s0">S0</div><div class="sensor-node" id="s1">S1</div><div class="sensor-node" id="s2">S2</div><div class="sensor-node" id="s3">S3</div><div class="sensor-node" id="s4">S4</div><div class="sensor-node" id="s5">S5</div><div class="sensor-node" id="s6">S6</div><div class="sensor-node" id="s7">S7</div></div></div>
  <div class="section-head"><div class="section-tag">B3</div><div class="section-label">IMU LSM6DS3 &amp; Controle PID</div><div class="section-rule"></div></div>
  <div class="imu-grid">
    <div class="card"><div class="card-label">Aceleração (g)</div><div class="imu-row"><span class="axis">aX</span><span class="val" id="ax">0.00</span></div><div class="track-bar"><div class="track-fill" id="axBar" style="width:50%"></div></div><div class="imu-row"><span class="axis">aY</span><span class="val" id="ay">0.00</span></div><div class="track-bar"><div class="track-fill" id="ayBar" style="width:50%"></div></div><div class="imu-row"><span class="axis">aZ</span><span class="val" id="az">0.00</span></div><div class="track-bar"><div class="track-fill" id="azBar" style="width:50%"></div></div></div>
    <div class="card"><div class="card-label">Giroscópio (°/s)</div><div class="imu-row"><span class="axis">gX</span><span class="val" id="gx">0.00</span></div><div class="track-bar"><div class="track-fill" id="gxBar" style="width:50%"></div></div><div class="imu-row"><span class="axis">gY</span><span class="val" id="gy">0.00</span></div><div class="track-bar"><div class="track-fill" id="gyBar" style="width:50%"></div></div><div class="imu-row"><span class="axis">gZ</span><span class="val" id="gz">0.00</span></div><div class="track-bar"><div class="track-fill" id="gzBar" style="width:50%"></div></div></div>
    <div class="card"><div class="card-label">Ganhos PID ativos</div><div class="pid-line"><span class="k">Kp</span><span class="v" id="kp">0.02</span></div><div class="track-bar"><div class="track-fill" style="width:8%"></div></div><div class="pid-line"><span class="k">Ki</span><span class="v" id="ki">0.000000</span></div><div class="track-bar"><div class="track-fill" style="width:0%"></div></div><div class="pid-line"><span class="k">Kd</span><span class="v" id="kd">0.10</span></div><div class="track-bar"><div class="track-fill" style="width:40%"></div></div><div class="pid-badge" id="pidAppliedBadge">✓ ML APLICADO</div></div>
    <div class="card"><div class="card-label">PWM Motores</div><div class="imu-row"><span class="axis">Motor E</span><span class="val" id="pwmE">0</span></div><div class="track-bar"><div class="track-fill" id="pwmEBar" style="width:0%"></div></div><div class="imu-row"><span class="axis">Motor D</span><span class="val" id="pwmD">0</span></div><div class="track-bar"><div class="track-fill" id="pwmDBar" style="width:0%"></div></div></div>
  </div>
  <div class="section-head"><div class="section-tag">B4</div><div class="section-label">Plano Cartesiano (X,Y)</div><div class="section-rule"></div></div>
  <div class="canvas-wrap">
    <div class="canvas-btn"><button class="btn" onclick="resetarTrilha()">RESETAR</button></div>
    <canvas id="trackCanvas" width="800" height="600"></canvas>
    <div class="coord-info" id="coordInfo">x: -- | y: --</div>
  </div>
  <div class="section-head"><div class="section-tag">B5</div><div class="section-label">Análise ML — recomendação de PID</div><div class="section-rule"></div></div>
  <div class="ml-wrap"><div id="mlAnalysis"><div class="empty-state">AGUARDANDO DADOS...</div></div></div>
  <div class="controls-row">
    <button class="btn" onclick="baixarDados()">BAIXAR CSV</button>
    <button class="btn btn-primary" onclick="atualizarML()">ATUALIZAR ML</button>
    <button class="btn" onclick="limparDados()">LIMPAR</button>
  </div>
</div>
<script>
  const trajetoria = [];
  const canvas = document.getElementById('trackCanvas');
  const ctx = canvas.getContext('2d');
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;

  function atualizarLimites(x, y) {
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
  }
  function resetarLimites() {
    minX = Infinity; maxX = -Infinity; minY = Infinity; maxY = -Infinity;
    trajetoria.length = 0;
    desenharPlano();
  }
  function worldToPixelX(x, w) { if (minX === maxX) return w/2; return ((x - minX) / (maxX - minX)) * w; }
  function worldToPixelY(y, h) { if (minY === maxY) return h/2; return h - ((y - minY) / (maxY - minY)) * h; }
  function desenharPlano() {
    const w = canvas.width, h = canvas.height;
    ctx.fillStyle = '#0a111f'; ctx.fillRect(0, 0, w, h);
    if (trajetoria.length === 0) {
      ctx.strokeStyle = '#2a3f5e'; ctx.lineWidth = 1;
      ctx.beginPath(); ctx.moveTo(0, h/2); ctx.lineTo(w, h/2); ctx.moveTo(w/2, 0); ctx.lineTo(w/2, h); ctx.stroke();
      ctx.fillStyle = '#8fa3b8'; ctx.font = '10px Courier New'; ctx.fillText('X →', w-20, h/2-5); ctx.fillText('Y ↑', w/2+5, 15);
      return;
    }
    let lx = minX, ly = minY, ux = maxX, uy = maxY;
    if (lx === Infinity) { for (let p of trajetoria) { if (p.x < lx) lx = p.x; if (p.x > ux) ux = p.x; if (p.y < ly) ly = p.y; if (p.y > uy) uy = p.y; } }
    const dx = (ux - lx) * 0.1 || 1, dy = (uy - ly) * 0.1 || 1;
    const xMin = lx - dx, xMax = ux + dx, yMin = ly - dy, yMax = uy + dy;
    function xPx(x) { return ((x - xMin) / (xMax - xMin)) * w; }
    function yPx(y) { return h - ((y - yMin) / (yMax - yMin)) * h; }
    ctx.strokeStyle = '#1c2f45'; ctx.lineWidth = 0.5;
    for (let i = 0; i <= 8; i++) { let x = xMin + (i/8)*(xMax - xMin); let px = xPx(x); ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, h); ctx.stroke(); }
    for (let i = 0; i <= 8; i++) { let y = yMin + (i/8)*(yMax - yMin); let py = yPx(y); ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(w, py); ctx.stroke(); }
    ctx.strokeStyle = '#3b82f6'; ctx.lineWidth = 1;
    let zeroXpx = xPx(0); if (zeroXpx > 0 && zeroXpx < w) { ctx.beginPath(); ctx.moveTo(zeroXpx, 0); ctx.lineTo(zeroXpx, h); ctx.stroke(); }
    let zeroYpx = yPx(0); if (zeroYpx > 0 && zeroYpx < h) { ctx.beginPath(); ctx.moveTo(0, zeroYpx); ctx.lineTo(w, zeroYpx); ctx.stroke(); }
    ctx.beginPath(); let first = true;
    for (let p of trajetoria) { let px = xPx(p.x), py = yPx(p.y); if (first) { ctx.moveTo(px, py); first = false; } else { ctx.lineTo(px, py); } }
    ctx.strokeStyle = '#2563eb'; ctx.lineWidth = 2; ctx.stroke();
    if (trajetoria.length > 0) {
      let last = trajetoria[trajetoria.length-1];
      let px = xPx(last.x), py = yPx(last.y);
      ctx.fillStyle = '#f0f4f8'; ctx.beginPath(); ctx.arc(px, py, 5, 0, 2*Math.PI); ctx.fill();
      ctx.fillStyle = '#3b82f6'; ctx.font = 'bold 10px Courier New'; ctx.fillText(`x=${last.x.toFixed(2)} y=${last.y.toFixed(2)}`, px+8, py-4);
    }
    ctx.fillStyle = '#8fa3b8'; ctx.font = '9px Courier New';
    ctx.fillText(`X: ${xMin.toFixed(1)} → ${xMax.toFixed(1)}`, 8, 20);
    ctx.fillText(`Y: ${yMin.toFixed(1)} → ${yMax.toFixed(1)}`, 8, 35);
  }
  function atualizarML() {
    fetch('/tune').then(r=>r.json()).then(data=>{
      const el=document.getElementById('mlAnalysis');
      if(data.error){el.innerHTML='<div class="empty-state">⚠ '+data.error+'</div>';return;}
      if(!data.metrics){el.innerHTML='<div class="empty-state">SEM DADOS SUFICIENTES</div>';return;}
      const m=data.metrics, rec=data.aiRecommendation, opt=data.optimized;
      let html='<div class="ml-grid"><div class="ml-block"><div class="ml-block-title">Métricas actuais</div>Erro médio : <strong>'+m.avgError+'</strong><br>Erro máx : <strong>'+m.maxError+'</strong><br>Instabilid. : <strong>'+m.instabilityCount+'</strong><br>Acel. Y (avg) : <strong>'+m.accelY.avg.toFixed(3)+' g</strong><br>Acel. Y (max) : <strong>'+m.accelY.max.toFixed(3)+' g</strong></div>';
      if(rec){html+='<div class="ml-block"><div class="ml-block-title">Recomendação ML</div>Kp : <strong id="recKp">'+rec.kp.toFixed(6)+'</strong><br>Ki : <strong>'+rec.ki.toFixed(8)+'</strong><br>Kd : <strong id="recKd">'+rec.kd.toFixed(6)+'</strong><br>Confiança : <strong>'+(rec.confidence*100).toFixed(0)+'%</strong></div><div class="ml-block"><div class="ml-block-title">Aplicar ao robô</div><button class="btn btn-apply" onclick="aplicarRecomendacaoIA('+rec.kp.toFixed(6)+','+rec.ki.toFixed(8)+','+rec.kd.toFixed(6)+')">✓ APLICAR RECOMENDAÇÃO</button></div>';}
      html+='<div class="ml-block"><div class="ml-block-title">Histórico</div>Buffer : <strong>'+data.bufferSize+'</strong> pontos<br>';
      if(opt&&opt.kp>0) html+='Último Kp : <strong>'+opt.kp.toFixed(6)+'</strong><br>Último Kd : <strong>'+opt.kd.toFixed(6)+'</strong><br>Status : <strong style="color:#4ade80">'+(opt.applied?'✓ Aplicado':'Pendente')+'</strong>';
      else html+='Nenhuma otimização aplicada ainda.';
      html+='</div></div>';
      el.innerHTML=html;
    }).catch(()=>document.getElementById('mlAnalysis').innerHTML='<div class="empty-state">ERRO AO CARREGAR ANÁLISE</div>');
  }
  function aplicarRecomendacaoIA(kp,ki,kd){
    if(!confirm(`Aplicar ao robô agora?\n\nKp = ${kp.toFixed(6)}\nKi = ${ki.toFixed(8)}\nKd = ${kd.toFixed(6)}`)) return;
    fetch(`/aplicar-pid?kp=${kp}&ki=${ki}&kd=${kd}`).then(r=>r.json()).then(data=>{
      if(data.status==='OK'){document.getElementById('kp').textContent=parseFloat(data.kp).toFixed(6);document.getElementById('ki').textContent=parseFloat(data.ki).toFixed(8);document.getElementById('kd').textContent=parseFloat(data.kd).toFixed(6);document.getElementById('pidAppliedBadge').style.display='block';atualizarML();}
      else alert('Erro: '+data.error);
    }).catch(()=>alert('Erro de conexão'));
  }
  function atualizarDados(){
    fetch('/dados.json').then(r=>r.json()).then(data=>{
      if(!data||!data.t)return;
      document.getElementById('posicao').textContent=data.pos; document.getElementById('erro').textContent=data.err; document.getElementById('correcao').textContent=data.cor.toFixed(1);
      document.getElementById('rpmE').textContent=data.pwmE; document.getElementById('rpmD').textContent=data.pwmD;
      document.getElementById('pwmE').textContent=data.pwmE; document.getElementById('pwmD').textContent=data.pwmD;
      document.getElementById('kp').textContent=data.kp.toFixed(6); document.getElementById('ki').textContent=data.ki.toFixed(8); document.getElementById('kd').textContent=data.kd.toFixed(6);
      document.getElementById('ax').textContent=(data.ax||0).toFixed(2); document.getElementById('ay').textContent=(data.ay||0).toFixed(2); document.getElementById('az').textContent=(data.az||0).toFixed(2);
      document.getElementById('gx').textContent=(data.gx||0).toFixed(2); document.getElementById('gy').textContent=(data.gy||0).toFixed(2); document.getElementById('gz').textContent=(data.gz||0).toFixed(2);
      document.getElementById('posBar').style.width=((data.pos/7000)*100)+'%';
      document.getElementById('barE').style.width=((Math.abs(data.pwmE)/255)*100)+'%';
      document.getElementById('barD').style.width=((Math.abs(data.pwmD)/255)*100)+'%';
      document.getElementById('pwmEBar').style.width=((Math.abs(data.pwmE)/255)*100)+'%';
      document.getElementById('pwmDBar').style.width=((Math.abs(data.pwmD)/255)*100)+'%';
      const acclPct=v=>Math.min(100,Math.max(0,((parseFloat(v)+2)/4)*100));
      const giroPct=v=>Math.min(100,Math.max(0,((parseFloat(v)+250)/500)*100));
      document.getElementById('axBar').style.width=acclPct(data.ax)+'%';
      document.getElementById('ayBar').style.width=acclPct(data.ay)+'%';
      document.getElementById('azBar').style.width=acclPct(data.az)+'%';
      document.getElementById('gxBar').style.width=giroPct(data.gx)+'%';
      document.getElementById('gyBar').style.width=giroPct(data.gy)+'%';
      document.getElementById('gzBar').style.width=giroPct(data.gz)+'%';
      document.getElementById('statusPill').className='status-pill online';
      document.getElementById('statusText').textContent='CONECTADO';
      document.getElementById('coordInfo').innerHTML=`x: ${data.x?.toFixed(2)??'?'} | y: ${data.y?.toFixed(2)??'?'}`;
      if(data.x!==undefined&&data.y!==undefined){
        trajetoria.push({x:data.x,y:data.y});
        if(trajetoria.length>800)trajetoria.shift();
        atualizarLimites(data.x,data.y);
        desenharPlano();
      } else desenharPlano();
    }).catch(()=>{document.getElementById('statusPill').className='status-pill offline';document.getElementById('statusText').textContent='OFFLINE';});
  }
  function resetarTrilha(){resetarLimites(); fetch('/reset').catch(()=>{});}
  function baixarDados(){fetch('/dados.csv').then(r=>r.text()).then(csv=>{const a=document.createElement('a');a.href=URL.createObjectURL(new Blob([csv],{type:'text/csv'}));a.download='telemetria_xy_'+new Date().toISOString().slice(0,10)+'.csv';a.click();});}
  function limparDados(){resetarLimites(); document.getElementById('pidAppliedBadge').style.display='none';}
  desenharPlano(); atualizarML(); setInterval(atualizarDados,200);
</script>
</body>
</html>
)rawliteral";

  webServer.send(200, "text/html", html);
}

void handleTelemetria()
{
  clientesConectados = true;
  webServer.setContentLength(CONTENT_LENGTH_UNKNOWN);
  webServer.send(200, "text/event-stream", "");

  for (int i = 0; i < min(50, bufferCount); i++)
  {
    TelemetriaData &d = dataBuffer[i];
    char jsonLine[400];
    snprintf(jsonLine, sizeof(jsonLine),
             "data: {\"t\":%lu,\"pos\":%u,\"err\":%d,\"cor\":%.2f,\"pwmE\":%d,\"pwmD\":%d,"
             "\"kp\":%.6f,\"ki\":%.6f,\"kd\":%.6f,\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
             "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,\"x\":%.3f,\"y\":%.3f}\n\n",
             d.timestamp, d.position, d.error, d.correction, d.pwmL, d.pwmR,
             d.kp, d.ki, d.kd, d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.x, d.y);
    webServer.sendContent(jsonLine);
  }
  clientesConectados = false;
}

void handleJSON()
{
  webServer.send(200, "application/json", ultimoJSON);
}

void handleCSV()
{
  String csv = "timestamp,posicao,erro,correcao,pwm_esq,pwm_dir,kp,ki,kd,ax,ay,az,gx,gy,gz,x,y\n";
  for (int i = 0; i < bufferCount; i++)
  {
    TelemetriaData &d = dataBuffer[i];
    char line[350];
    snprintf(line, sizeof(line),
             "%lu,%u,%d,%.2f,%u,%u,%.6f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f\n",
             d.timestamp, d.position, d.error, d.correction, d.pwmL, d.pwmR,
             d.kp, d.ki, d.kd, d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.x, d.y);
    csv += line;
  }
  webServer.send(200, "text/csv; charset=utf-8", csv);
}

CurveAnalysis analisarCurva(int startIdx, int endIdx)
{
  CurveAnalysis analysis;
  analysis.avgAccelY = 0;
  analysis.peakAccelY = 0;

  if (endIdx <= startIdx)
  {
    analysis.curveType = 0;
    analysis.recommendedKp = 0.05;
    analysis.recommendedKd = 0.2;
    return analysis;
  }

  int count = 0;
  for (int i = startIdx; i < endIdx && i < bufferCount; i++)
  {
    analysis.avgAccelY += dataBuffer[i].ay;
    if (fabs(dataBuffer[i].ay) > fabs(analysis.peakAccelY))
      analysis.peakAccelY = dataBuffer[i].ay;
    count++;
  }
  if (count > 0)
    analysis.avgAccelY /= count;

  float peak = fabs(analysis.peakAccelY);
  if (peak < 0.5)
  {
    analysis.curveType = 0;
    analysis.recommendedKp = 0.03;
    analysis.recommendedKd = 0.15;
  }
  else if (peak < 1.0)
  {
    analysis.curveType = 1;
    analysis.recommendedKp = 0.05;
    analysis.recommendedKd = 0.20;
  }
  else if (peak < 1.5)
  {
    analysis.curveType = 2;
    analysis.recommendedKp = 0.07;
    analysis.recommendedKd = 0.25;
  }
  else
  {
    analysis.curveType = 3;
    analysis.recommendedKp = 0.10;
    analysis.recommendedKd = 0.35;
  }

  return analysis;
}

void handleML()
{
  String json = "{";
  json += "\"bufferSize\":" + String(bufferCount) + ",";

  if (bufferCount > 10)
  {
    CurveAnalysis full = analisarCurva(0, bufferCount);
    CurveAnalysis s1 = analisarCurva(0, bufferCount / 4);
    CurveAnalysis s2 = analisarCurva(bufferCount / 4, bufferCount / 2);
    CurveAnalysis s3 = analisarCurva(bufferCount / 2, 3 * bufferCount / 4);
    CurveAnalysis s4 = analisarCurva(3 * bufferCount / 4, bufferCount);

    json += "\"overall\":{";
    json += "\"peakAccelY\":" + String(full.peakAccelY, 2) + ",";
    json += "\"curveType\":" + String(full.curveType) + ",";
    json += "\"recommendation\":{\"kp\":" + String(full.recommendedKp, 4) + ",\"kd\":" + String(full.recommendedKd, 4) + "}},";

    const char *names[] = {"reta", "suave", "media", "forte"};
    CurveAnalysis segs[] = {s1, s2, s3, s4};
    json += "\"segments\":[";
    for (int i = 0; i < 4; i++)
    {
      if (i > 0)
        json += ",";
      json += "{\"type\":\"" + String(names[segs[i].curveType]) + "\"," + "\"peakAccelY\":" + String(segs[i].peakAccelY, 2) + "," + "\"suggestedKp\":" + String(segs[i].recommendedKp, 4) + "}";
    }
    json += "],";

    float minPos = 7000, maxPos = 0, avgErr = 0;
    for (int i = 0; i < bufferCount; i++)
    {
      if (dataBuffer[i].position < minPos)
        minPos = dataBuffer[i].position;
      if (dataBuffer[i].position > maxPos)
        maxPos = dataBuffer[i].position;
      avgErr += abs(dataBuffer[i].error);
    }
    avgErr /= bufferCount;
    json += "\"stats\":{\"positionRange\":[" + String((int)minPos) + "," + String((int)maxPos) + "]," + "\"avgError\":" + String((int)avgErr) + "," + "\"totalPoints\":" + String(bufferCount) + "}";
  }
  else
  {
    json += "\"message\":\"Insuficientes dados para analise\"";
  }

  json += "}";
  webServer.send(200, "application/json; charset=utf-8", json);
}

void iniciarTelemetriaWiFi()
{
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

void processarTelemetriaWiFi()
{
  webServer.handleClient();
}

void enviarDadosTelemetria(int erro, float correcao, int pwmE, int pwmD,
                           float kp, float ki, float kd, uint16_t posicaoSensor,
                           float ax, float ay, float az, float gx, float gy, float gz,
                           float x, float y)
{
  char jsonBuffer[450];
  snprintf(jsonBuffer, sizeof(jsonBuffer),
           "{\"t\":%lu,\"pos\":%u,\"err\":%d,\"cor\":%.2f,\"pwmE\":%d,\"pwmD\":%d,"
           "\"kp\":%.6f,\"ki\":%.6f,\"kd\":%.6f,"
           "\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
           "\"x\":%.3f,\"y\":%.3f}",
           millis(), posicaoSensor, erro, correcao, pwmE, pwmD,
           kp, ki, kd, ax, ay, az, gx, gy, gz, x, y);
  ultimoJSON = jsonBuffer;

  dataBuffer[bufferIndex] = {
      millis(), posicaoSensor, (int16_t)erro, correcao,
      (uint8_t)pwmE, (uint8_t)pwmD,
      kp, ki, kd,
      ax, ay, az, gx, gy, gz,
      x, y};
  bufferIndex = (bufferIndex + 1) % BUFFER_SIZE;
  if (bufferCount < BUFFER_SIZE)
    bufferCount++;
}

void handleReset()
{
  bufferIndex = 0;
  bufferCount = 0;
  webServer.send(200, "application/json", "{\"status\":\"buffer resetado\"}");
}

void handleTuneML()
{
  String json = "{";
  json += "\"timestamp\":" + String(millis()) + ",";
  json += "\"bufferSize\":" + String(bufferCount) + ",";

  if (bufferCount > 20)
  {
    float sumAbsErr = 0, maxErr = 0, sumCorr = 0, maxCorr = 0;
    float sumAy = 0, maxAy = 0, sumAccel = 0;
    float sumErrSq = 0;          // para variância
    int outOfLineCount = 0;      // contagem de pontos fora da linha
    int largeJumps = 0;          // saltos bruscos de erro

    for (int i = 0; i < bufferCount; i++)
    {
      TelemetriaData &d = dataBuffer[i];
      int ae = abs(d.error);
      sumAbsErr += ae;
      if (ae > maxErr) maxErr = ae;
      
      float ac = fabs(d.correction);
      sumCorr += ac;
      if (ac > maxCorr) maxCorr = ac;
      
      float ay = fabs(d.ay);
      sumAy += ay;
      if (ay > maxAy) maxAy = ay;
      
      sumAccel += sqrt(d.ax * d.ax + d.ay * d.ay + d.az * d.az);
      sumErrSq += ae * ae;

      // Critério para "fora da linha": erro próximo do máximo possível (3500)
      // ou posição do sensor nos extremos (0 ou 7000)
      if (ae > 3000 || d.position < 500 || d.position > 6500) {
        outOfLineCount++;
      }
    }

    float avgErr = sumAbsErr / bufferCount;
    float varErr = (sumErrSq / bufferCount) - (avgErr * avgErr);
    float stdErr = sqrt(varErr);
    
    // Conta saltos bruscos de erro (diferença entre pontos consecutivos > 500)
    for (int i = 1; i < bufferCount; i++) {
      if (abs(dataBuffer[i].error - dataBuffer[i-1].error) > 500) {
        largeJumps++;
      }
    }
    float instabilityRatio = (float)largeJumps / bufferCount;

    float avgCorr = sumCorr / bufferCount;
    float avgAy = sumAy / bufferCount;
    float avgAccel = sumAccel / bufferCount;

    // ========== CÁLCULO DA CONFIANÇA ==========
    float confidence = 0.85; // valor base
    
    // 1. Penalidade por erro alto
    if (avgErr > 1000) confidence -= 0.20;
    else if (avgErr > 500) confidence -= 0.10;
    else if (avgErr > 200) confidence -= 0.05;
    
    // 2. Penalidade por variância alta (oscilação)
    if (stdErr > 400) confidence -= 0.15;
    else if (stdErr > 200) confidence -= 0.08;
    
    // 3. Penalidade por instabilidade (saltos bruscos)
    if (instabilityRatio > 0.3) confidence -= 0.20;
    else if (instabilityRatio > 0.15) confidence -= 0.10;
    
    // 4. Penalidade por tempo fora da linha
    float outOfLineRatio = (float)outOfLineCount / bufferCount;
    if (outOfLineRatio > 0.2) confidence -= 0.30;
    else if (outOfLineRatio > 0.05) confidence -= 0.15;
    
    // 5. Bônus para buffer grande (mais dados = mais confiável)
    if (bufferCount > 100) confidence += 0.05;
    if (bufferCount > 200) confidence += 0.05;
    
    // Limita entre 0.1 e 0.98
    confidence = constrain(confidence, 0.10, 0.98);
    
    // ========== RECOMENDAÇÃO DE PID (ajustada pela confiança) ==========
    float sKp, sKi, sKd;
    
    // Lógica baseada no erro médio (agora mais conservadora se confiança baixa)
    if (avgErr > 2000) {
      sKp = 0.15; sKi = 0.00005; sKd = 0.30;
    } else if (avgErr > 1000) {
      sKp = 0.10; sKi = 0.00003; sKd = 0.25;
    } else if (avgErr > 500) {
      sKp = 0.07; sKi = 0.00002; sKd = 0.20;
    } else if (avgErr > 200) {
      sKp = 0.05; sKi = 0.00001; sKd = 0.15;
    } else {
      sKp = 0.03; sKi = 0.000005; sKd = 0.10;
    }
    
    // Ajuste fino baseado na aceleração lateral
    if (maxAy > 2.0) sKd += 0.15;
    else if (maxAy > 1.0) sKd += 0.05;
    
    // Se a confiança está baixa, recomendamos valores mais conservadores (menor Kp, maior Kd)
    if (confidence < 0.5) {
      sKp *= 0.7;
      sKd *= 1.2;
    } else if (confidence < 0.7) {
      sKp *= 0.9;
      sKd *= 1.05;
    }
    
    // ========== MONTA RESPOSTA JSON ==========
    json += "\"metrics\":{"
            "\"avgError\":" + String((int)avgErr) + ","
            "\"stdError\":" + String(stdErr, 1) + ","
            "\"maxError\":" + String((int)maxErr) + ","
            "\"avgCorrection\":" + String(avgCorr, 2) + ","
            "\"maxCorrection\":" + String(maxCorr, 2) + ","
            "\"instabilityCount\":" + String(largeJumps) + ","
            "\"outOfLineRatio\":" + String(outOfLineRatio, 3) + ","
            "\"accelY\":{\"avg\":" + String(avgAy, 3) + ",\"max\":" + String(maxAy, 3) + "},"
            "\"accelTotal\":{\"avg\":" + String(avgAccel, 3) + "}"
            "},";
    
    json += "\"aiRecommendation\":{"
            "\"kp\":" + String(sKp, 6) + ","
            "\"ki\":" + String(sKi, 8) + ","
            "\"kd\":" + String(sKd, 6) + ","
            "\"confidence\":" + String(confidence, 4) + ","
            "\"reasoning\":\"Baseado em erro médio=" + String((int)avgErr) 
            + ", variância=" + String(stdErr, 0) 
            + ", instabilidade=" + String(instabilityRatio, 2)
            + ", fora da linha=" + String(outOfLineRatio, 2) + "\""
            "},";
    
    json += "\"optimized\":{"
            "\"kp\":" + String(pidAplicado ? pidKpOtimizado : -1.0f, 6) + ","
            "\"kd\":" + String(pidAplicado ? pidKdOtimizado : -1.0f, 6) + ","
            "\"applied\":" + String(pidAplicado ? "true" : "false") + "}";
  }
  else
  {
    json += "\"error\":\"Insuficientes dados (mínimo 20 pontos)\"";
  }

  json += "}";
  webServer.send(200, "application/json; charset=utf-8", json);
}

void handleAplicarPID()
{
  if (!webServer.hasArg("kp") || !webServer.hasArg("kd"))
  {
    webServer.send(400, "application/json", "{\"error\":\"Faltam parametros kp e kd\"}");
    return;
  }

  float novoKp = webServer.arg("kp").toFloat();
  float novoKd = webServer.arg("kd").toFloat();
  float novoKi = webServer.hasArg("ki") ? webServer.arg("ki").toFloat() : KI;

  if (novoKp < 0 || novoKd < 0 || novoKi < 0)
  {
    webServer.send(400, "application/json", "{\"error\":\"Valores invalidos (devem ser >= 0)\"}");
    return;
  }

  pidKpOtimizado = novoKp;
  pidKdOtimizado = novoKd;
  pidAplicado = true;

  KP = novoKp;
  KI = novoKi;
  KD = novoKd;

  salvarConfiguracoes();

  Serial.println("[ML] PID atualizado via interface web:");
  Serial.print("  KP = ");
  Serial.println(KP, 6);
  Serial.print("  KI = ");
  Serial.println(KI, 8);
  Serial.print("  KD = ");
  Serial.println(KD, 6);

  String resp = "{\"status\":\"OK\","
                "\"kp\":" +
                String(KP, 6) + ","
                                "\"ki\":" +
                String(KI, 8) + ","
                                "\"kd\":" +
                String(KD, 6) + ","
                                "\"appliedAt\":" +
                String(millis()) + "}";
  webServer.send(200, "application/json", resp);
}

bool temClientesConectados()
{
  return clientesConectados;
}