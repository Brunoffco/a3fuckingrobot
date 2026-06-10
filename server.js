const express = require('express');
const path = require('path');
const app = express();

app.use(express.json());

let kp = 0.05;
let ki = 0.00001;
let kd = 0.15;
let bufferSize = 500;

app.get('/dados.json', (req, res) => {
    const t = Date.now() / 1000;
    // Movimento circular com raio 5 metros, período 20 segundos
    const raio = 5;
    const omega = (2 * Math.PI) / 20;
    const x = raio * Math.cos(omega * t);
    const y = raio * Math.sin(omega * t);
    // Pequeno ruído para simular imperfeições
    const noise = 0.05;
    res.json({
        t: Date.now(),
        x: x + (Math.random() - 0.5) * noise,
        y: y + (Math.random() - 0.5) * noise,
        pos: Math.round(3500 + 1500 * Math.sin(omega * t)),
        err: Math.round(1500 * Math.sin(omega * t)),
        cor: 0,
        pwmE: Math.round(180 + 30 * Math.sin(omega * t)),
        pwmD: Math.round(180 - 30 * Math.sin(omega * t)),
        kp, ki, kd,
        ax: 0.56 * Math.cos(omega * t),
        ay: 0.56 * Math.sin(omega * t),
        az: 1.0,
        gx: 0, gy: 0, gz: 18.0
    });
});

app.get('/tune', (req, res) => {
    res.json({
        bufferSize,
        metrics: {
            avgError: 320, maxError: 1250, avgCorrection: 25, maxCorrection: 88,
            instabilityCount: 4, accelY: { avg: 0.72, max: 1.43 }
        },
        aiRecommendation: { kp: 0.07, ki: 0.00002, kd: 0.20, confidence: 0.85 },
        optimized: { kp, kd, applied: true }
    });
});

app.get('/aplicar-pid', (req, res) => {
    kp = Number(req.query.kp || kp);
    ki = Number(req.query.ki || ki);
    kd = Number(req.query.kd || kd);
    res.json({ status: 'OK', kp, ki, kd, appliedAt: Date.now() });
});

app.get('/reset', (req, res) => {
    bufferSize = 0;
    res.json({ status: 'buffer resetado' });
});

app.get('/dados.csv', (req, res) => {
    res.setHeader('Content-Type', 'text/csv');
    res.send(`timestamp,posicao,erro,x,y\n${Date.now()},3500,0,0,0\n${Date.now()+1},3600,20,1,1\n${Date.now()+2},3400,-10,2,-1`);
});

app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'index.html'));
});

app.listen(3000, () => {
    console.log('Servidor iniciado em http://localhost:3000');
    console.log('Gráfico XY ativo – trajetória circular com ruído.');
});