'use strict';

(() => {
  const WHEELS = ['FL', 'FR', 'RL', 'RR'];
  const NAMES = ['Front-left', 'Front-right', 'Rear-left', 'Rear-right'];
  const COLORS = ['#5aa9ff', '#3dd6b5', '#c792ea', '#ffb454'];
  const INDICATORS = [
    { name: 'Diagonal', color: '#5aa9ff' },
    { name: 'Side', color: '#3dd6b5' },
    { name: 'Axle', color: '#ffb454' },
  ];
  const GATES = [
    { name: 'accepted', color: '#3ddc84' },
    { name: 'speed', color: '#4a5a6d' },
    { name: 'braking', color: '#ff5d5d' },
    { name: 'accel', color: '#ffb454' },
    { name: 'cornering', color: '#c792ea' },
    { name: 'wheel slip', color: '#5aa9ff' },
  ];
  const RAM_PARTS = [
    ['queue', 'Edge queue', '#5aa9ff'],
    ['teeth', 'Tooth tracking', '#3dd6b5'],
    ['resampler', 'Resampler', '#c792ea'],
    ['spectra', 'Spectra', '#ffb454'],
    ['fft', 'FFT scratch', '#ff7a90'],
    ['model', 'Radius model', '#8a99ad'],
  ];
  // Tire rectangles in the car SVG; labels sit outside the body.
  const TIRE_POS = [
    { x: 118, y: 78, side: 'l' },
    { x: 272, y: 78, side: 'r' },
    { x: 118, y: 256, side: 'l' },
    { x: 272, y: 256, side: 'r' },
  ];
  const HISTORY_MAX = 3600;  // samples (1 per simulated second)
  const WINDOW_MIN = 20;     // chart window, simulated minutes
  const SLOW_LEAK = 0.7;     // psi/min
  const PUNCTURE = 7;        // psi/min
  const SVG_NS = 'http://www.w3.org/2000/svg';

  const $ = (id) => document.getElementById(id);

  let demo = null;
  let mem = null;
  let snap = null;
  let running = true;
  let speed = 60;
  let selected = 0;
  let dragging = false;
  let lastFrame = 0;
  let hist = null;
  let lastHistT = -1;
  let perfPrev = null;
  let perfAt = 0;
  const tireEls = [];
  const specEls = [];

  function resetHistory() {
    hist = { t: [], loss: [[], [], [], []], truth: [[], [], [], []], res: [[], [], []] };
    lastHistT = -1;
    perfPrev = null;
  }

  // ---------------------------------------------------------------- setup
  function svg(tag, attrs, parent) {
    const el = document.createElementNS(SVG_NS, tag);
    for (const [k, v] of Object.entries(attrs)) el.setAttribute(k, v);
    if (parent) parent.appendChild(el);
    return el;
  }

  function buildTires() {
    const root = $('tires');
    TIRE_POS.forEach((p, i) => {
      const g = svg('g', { class: 'tire', tabindex: '0', role: 'button', 'aria-label': `Select ${NAMES[i]} tire` }, root);
      svg('rect', { class: 'selbox', x: p.x - 7, y: p.y - 7, width: 44, height: 80, rx: 10 }, g);
      svg('rect', { class: 't', x: p.x, y: p.y, width: 30, height: 66, rx: 8 }, g);
      for (let k = 1; k < 6; k++) {
        const y = p.y + k * 11;
        svg('line', { class: 'tread', x1: p.x + 5, x2: p.x + 25, y1: y, y2: y }, g);
      }
      const left = p.side === 'l';
      const tx = left ? 10 : 410;
      const anchor = left ? 'start' : 'end';
      const name = svg('text', { class: 'name', x: tx, y: p.y + 12, 'text-anchor': anchor }, g);
      name.textContent = WHEELS[i];
      const sub = svg('text', { class: 'sub', x: tx, y: p.y + 30, 'text-anchor': anchor }, g);
      const est = svg('text', { class: 'est', x: tx, y: p.y + 48, 'text-anchor': anchor }, g);
      const bx = left ? 10 : 310;
      svg('rect', { class: 'gauge-bg', x: bx, y: p.y + 56, width: 100, height: 6, rx: 3 }, g);
      const fill = svg('rect', { x: bx, y: p.y + 56, width: 0, height: 6, rx: 3, fill: COLORS[i] }, g);
      const truth = svg('rect', { x: bx, y: p.y + 53, width: 2, height: 12, fill: '#e6edf3' }, g);
      g.addEventListener('click', () => select(i));
      g.addEventListener('keydown', (e) => {
        if (e.key === 'Enter' || e.key === ' ') { e.preventDefault(); select(i); }
      });
      tireEls.push({ g, sub, est, fill, truth, bx });
    });
  }

  function buildTabs() {
    const tabs = $('tireTabs');
    WHEELS.forEach((w, i) => {
      const b = document.createElement('button');
      b.className = 'btn';
      b.textContent = w;
      b.setAttribute('role', 'tab');
      b.addEventListener('click', () => select(i));
      tabs.appendChild(b);
    });
  }

  function buildSpectra() {
    const root = $('spectra');
    WHEELS.forEach((w, i) => {
      const box = document.createElement('div');
      box.className = 'spec';
      box.innerHTML = `<div class="spec-h"><b style="color:${COLORS[i]}">${w}</b><span>–</span></div><canvas></canvas>`;
      root.appendChild(box);
      specEls.push({ label: box.querySelector('span'), canvas: box.querySelector('canvas') });
    });
  }

  function buildRam() {
    const bar = $('ramBar');
    const legend = $('ramLegend');
    for (const [key, label, color] of RAM_PARTS) {
      const seg = document.createElement('div');
      seg.style.width = `${(100 * mem[key]) / mem.budget}%`;
      seg.style.background = color;
      seg.title = `${label}: ${mem[key]} B`;
      bar.appendChild(seg);
      legend.insertAdjacentHTML('beforeend', `<span><i style="background:${color}"></i>${label} ${fmtBytes(mem[key])}</span>`);
    }
    $('ramText').textContent = `${fmtBytes(mem.total)} / ${fmtBytes(mem.budget)} (${Math.round((100 * mem.total) / mem.budget)}%)`;
    const gl = $('gateLegend');
    GATES.forEach((g) => gl.insertAdjacentHTML('beforeend', `<span><i style="background:${g.color}"></i>${g.name}</span>`));
  }

  function select(i) {
    selected = i;
    tireEls.forEach((t, k) => t.g.classList.toggle('sel', k === i));
    [...$('tireTabs').children].forEach((b, k) => b.setAttribute('aria-selected', String(k === i)));
  }

  function wireControls() {
    $('play').addEventListener('click', () => {
      running = !running;
      $('play').textContent = running ? 'Pause' : 'Play';
      $('play').setAttribute('aria-label', running ? 'Pause simulation' : 'Resume simulation');
    });
    $('speeds').addEventListener('click', (e) => {
      const b = e.target.closest('button');
      if (!b) return;
      speed = Number(b.dataset.speed);
      [...$('speeds').children].forEach((x) => x.classList.toggle('on', x === b));
    });
    $('scenario').addEventListener('change', (e) => demo.setScenario(Number(e.target.value)));
    $('road').addEventListener('change', (e) => demo.setRoad(Number(e.target.value)));
    $('recal').addEventListener('click', () => demo.recalibrate());
    $('restart').addEventListener('click', () => {
      demo.reset(Math.floor(Math.random() * 1e9));
      resetHistory();
    });

    const slider = $('pSlider');
    slider.addEventListener('input', () => {
      dragging = true;
      demo.setPressure(selected, Number(slider.value) / 100);
      $('pOut').textContent = `${slider.value}%`;
    });
    slider.addEventListener('change', () => { dragging = false; });

    document.querySelectorAll('[data-act]').forEach((b) => b.addEventListener('click', () => {
      switch (b.dataset.act) {
        case 'slow': demo.setLeak(selected, SLOW_LEAK); break;
        case 'fast': demo.setLeak(selected, PUNCTURE); break;
        case 'fix': demo.setPressure(selected, 1); break;
        case 'all': for (let w = 0; w < 4; w++) demo.setPressure(w, 0.7); break;
        case 'fixall': demo.inflateAll(); break;
      }
    }));
  }

  // ---------------------------------------------------------------- utils
  function fmtBytes(n) { return n >= 1024 ? `${(n / 1024).toFixed(1)} KB` : `${n} B`; }
  function fmtTime(s) {
    const m = Math.floor(s / 60);
    const h = Math.floor(m / 60);
    const ss = String(Math.floor(s % 60)).padStart(2, '0');
    return h ? `${h}:${String(m % 60).padStart(2, '0')}:${ss}` : `${m}:${ss}`;
  }
  function pct(x, d = 0) { return `${(100 * x).toFixed(d)}%`; }

  function fitCanvas(c) {
    const r = c.getBoundingClientRect();
    const dpr = window.devicePixelRatio || 1;
    const w = Math.max(1, Math.round(r.width * dpr));
    const h = Math.max(1, Math.round(r.height * dpr));
    if (c.width !== w || c.height !== h) { c.width = w; c.height = h; }
    const ctx = c.getContext('2d');
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    ctx.clearRect(0, 0, r.width, r.height);
    return { ctx, w: r.width, h: r.height };
  }

  function lowerBound(arr, x) {
    let lo = 0, hi = arr.length;
    while (lo < hi) { const mid = (lo + hi) >> 1; if (arr[mid] < x) lo = mid + 1; else hi = mid; }
    return lo;
  }

  // ---------------------------------------------------------------- charts
  function lineChart(canvas, o) {
    const { ctx, w, h } = fitCanvas(canvas);
    const pad = { l: 46, r: 10, t: 22, b: 22 };
    const X = (t) => pad.l + ((t - o.xMin) / (o.xMax - o.xMin)) * (w - pad.l - pad.r);
    const Y = (v) => pad.t + (1 - (v - o.yMin) / (o.yMax - o.yMin)) * (h - pad.t - pad.b);

    ctx.font = '11px ui-monospace, Menlo, monospace';
    ctx.lineWidth = 1;
    ctx.strokeStyle = '#1a2430';
    ctx.fillStyle = '#5c6e84';
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (const v of o.yTicks) {
      const y = Math.round(Y(v)) + 0.5;
      ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(w - pad.r, y); ctx.stroke();
      ctx.fillText(o.yFmt(v), pad.l - 6, y);
    }
    const span = o.xMax - o.xMin;
    const step = span > 10 ? 5 : span > 4 ? 2 : 1;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    for (let m = Math.ceil(o.xMin / step) * step; m <= o.xMax + 1e-9; m += step) {
      ctx.fillText(`${m}m`, X(m), h - pad.b + 6);
    }
    for (const hl of o.hlines || []) {
      ctx.save();
      ctx.strokeStyle = hl.color;
      ctx.setLineDash(hl.dash || []);
      const y = Math.round(Y(hl.v)) + 0.5;
      ctx.beginPath(); ctx.moveTo(pad.l, y); ctx.lineTo(w - pad.r, y); ctx.stroke();
      ctx.restore();
    }

    ctx.save();
    ctx.beginPath();
    ctx.rect(pad.l, pad.t, w - pad.l - pad.r, h - pad.t - pad.b);
    ctx.clip();
    const t = hist.t;
    const start = Math.max(0, lowerBound(t, o.xMin) - 1);
    for (const s of o.series) {
      ctx.strokeStyle = s.color;
      ctx.lineWidth = s.width || 1.8;
      ctx.globalAlpha = s.alpha ?? 1;
      ctx.setLineDash(s.dash || []);
      ctx.beginPath();
      let pen = false;
      for (let i = start; i < t.length; i++) {
        const v = s.v[i];
        if (Number.isNaN(v)) { pen = false; continue; }
        const y = Y(Math.min(o.yMax * 2, Math.max(o.yMin * 2, v)));
        if (pen) ctx.lineTo(X(t[i]), y); else { ctx.moveTo(X(t[i]), y); pen = true; }
      }
      ctx.stroke();
    }
    ctx.restore();
    ctx.globalAlpha = 1;
    ctx.setLineDash([]);

    // Legend.
    ctx.textAlign = 'left';
    ctx.textBaseline = 'middle';
    ctx.font = '600 11px system-ui, sans-serif';
    let lx = pad.l;
    for (const l of o.legend) {
      ctx.fillStyle = l.color;
      ctx.fillRect(lx, 8, 10, 3);
      ctx.fillStyle = '#c4cfdc';
      ctx.fillText(l.name, lx + 14, 10);
      lx += ctx.measureText(l.name).width + 30;
    }
  }

  function drawLossChart() {
    const now = snap.t / 60;
    const xMax = Math.max(now, 2);
    const xMin = Math.max(0, xMax - WINDOW_MIN);
    let yMax = 0.5;
    for (let k = 0; k < 4; k++) for (const v of hist.truth[k]) yMax = Math.max(yMax, v + 0.05);
    yMax = Math.min(0.8, Math.ceil(yMax * 10) / 10);
    const series = [];
    for (let k = 0; k < 4; k++) {
      series.push({ v: hist.truth[k], color: COLORS[k], dash: [5, 4], width: 1.4, alpha: 0.7 });
      series.push({ v: hist.loss[k], color: COLORS[k], width: 2 });
    }
    const ticks = [];
    for (let v = -0.1; v <= yMax + 1e-9; v += 0.1) ticks.push(Math.round(v * 10) / 10);
    lineChart($('lossChart'), {
      xMin, xMax, yMin: -0.1, yMax, yTicks: ticks, yFmt: (v) => pct(v),
      series, legend: WHEELS.map((n, k) => ({ name: n, color: COLORS[k] })),
      hlines: [{ v: 0.2, color: 'rgba(255,93,93,.55)', dash: [3, 3] }],
    });
  }

  function drawRadiusChart() {
    const now = snap.t / 60;
    const xMax = Math.max(now, 2);
    const xMin = Math.max(0, xMax - WINDOW_MIN);
    let m = 0.0005;
    const start = lowerBound(hist.t, xMin);
    for (const s of hist.res) for (let i = start; i < s.length; i++) if (!Number.isNaN(s[i])) m = Math.max(m, Math.abs(s[i]));
    const lim = Math.ceil((m * 1.15) / 0.0005) * 0.0005;
    const step = lim / 2;
    lineChart($('radChart'), {
      xMin, xMax, yMin: -lim, yMax: lim, yTicks: [-lim, -step, 0, step, lim],
      yFmt: (v) => `${(v * 100).toFixed(2)}%`,
      series: hist.res.map((v, j) => ({ v, color: INDICATORS[j].color, width: 1.8 })),
      legend: INDICATORS,
      hlines: [{ v: 0, color: '#33465c' }],
    });
  }

  function drawSpectrum(i) {
    const { canvas, label } = specEls[i];
    const { ctx, w, h } = fitCanvas(canvas);
    const cur = demo.spectrum(i);
    const ref = demo.reference(i);
    const ws = snap.wheels[i];
    const n = mem.bins;
    const f0 = mem.psdLoHz;
    const f1 = f0 + mem.binHz * (n - 1);
    const hasRef = snap.monitoring && ws.fRef > 0;
    let max = 1e-9;
    for (let k = 0; k < n; k++) max = Math.max(max, cur[k], hasRef ? ref[k] : 0);
    const X = (f) => ((f - f0) / (f1 - f0)) * w;
    const Y = (v) => h - 14 - (v / max) * (h - 22);

    ctx.font = '10px ui-monospace, Menlo, monospace';
    ctx.fillStyle = '#4a5a6d';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'bottom';
    for (const f of [20, 40, 60]) {
      ctx.fillRect(X(f), h - 14, 1, 3);
      ctx.fillText(`${f}`, X(f), h);
    }

    const path = (arr) => {
      ctx.beginPath();
      for (let k = 0; k < n; k++) {
        const x = X(f0 + k * mem.binHz);
        if (k) ctx.lineTo(x, Y(arr[k])); else ctx.moveTo(x, Y(arr[k]));
      }
    };
    if (ws.frames > 0) {
      path(cur);
      ctx.lineTo(w, h - 14); ctx.lineTo(0, h - 14); ctx.closePath();
      ctx.fillStyle = `${COLORS[i]}2a`;
      ctx.fill();
      path(cur);
      ctx.strokeStyle = COLORS[i];
      ctx.lineWidth = 1.6;
      ctx.stroke();
    }
    if (hasRef) {
      path(ref);
      ctx.setLineDash([4, 3]);
      ctx.strokeStyle = '#8a99ad';
      ctx.lineWidth = 1.2;
      ctx.stroke();
      ctx.setLineDash([]);
    }
    const vline = (f, color, dash) => {
      if (!(f > f0 && f < f1)) return;
      ctx.save();
      ctx.strokeStyle = color;
      ctx.setLineDash(dash);
      ctx.beginPath(); ctx.moveTo(X(f) + 0.5, 4); ctx.lineTo(X(f) + 0.5, h - 14); ctx.stroke();
      ctx.restore();
    };
    if (hasRef) vline(ws.fRef, '#8a99ad', [2, 3]);
    if (ws.frames >= 16) vline(ws.f, COLORS[i], []);
    // True resonance (simulator ground truth).
    const xt = X(ws.fTrue);
    ctx.fillStyle = '#e6edf3';
    ctx.beginPath(); ctx.moveTo(xt - 4, 0); ctx.lineTo(xt + 4, 0); ctx.lineTo(xt, 6); ctx.closePath(); ctx.fill();

    label.textContent = ws.frames < 16 ? 'collecting…'
      : hasRef ? `${ws.fRef.toFixed(1)} → ${ws.f.toFixed(1)} Hz` : `${ws.f.toFixed(1)} Hz`;
  }

  // ---------------------------------------------------------------- render
  function record() {
    if (lastHistT >= 0 && snap.t - lastHistT < 1) return;
    lastHistT = snap.t;
    hist.t.push(snap.t / 60);
    for (let k = 0; k < 4; k++) {
      const ws = snap.wheels[k];
      hist.loss[k].push(snap.monitoring ? ws.loss : NaN);
      hist.truth[k].push(1 - ws.frac);
    }
    for (let j = 0; j < 3; j++) hist.res[j].push(snap.monitoring ? snap.residual[j] : NaN);
    if (hist.t.length > HISTORY_MAX) {
      hist.t.shift();
      hist.loss.forEach((a) => a.shift());
      hist.truth.forEach((a) => a.shift());
      hist.res.forEach((a) => a.shift());
    }
  }

  function renderStatus() {
    const pill = $('pill');
    const msg = $('msg');
    const bar = $('calBar');
    const warned = snap.wheels.map((w, i) => ({ ...w, i })).filter((w) => w.warn);
    let state;
    if (!snap.monitoring) {
      state = 'cal';
      pill.textContent = 'CALIBRATING';
      const off = snap.wheels.some((w) => Math.abs(w.frac - 1) > 0.03);
      msg.textContent = off
        ? 'Learning baseline. A tire is not at nominal pressure, so it will be learned as normal.'
        : `Learning baseline at nominal pressures… ${pct(snap.calProgress)}`;
    } else if (snap.allLow) {
      state = 'warn';
      pill.textContent = 'LOW PRESSURE';
      msg.textContent = `All four tires low (avg est. ${pct(snap.wheels.reduce((s, w) => s + w.loss, 0) / 4)})`;
    } else if (warned.length) {
      state = 'warn';
      pill.textContent = 'LOW PRESSURE';
      msg.textContent = warned.map((w) => `${NAMES[w.i]} ~${pct(w.loss)} low`).join(' · ');
    } else {
      state = 'ok';
      pill.textContent = 'MONITORING';
      msg.textContent = 'All tires OK';
    }
    pill.className = `pill ${state}`;
    bar.firstElementChild.style.width = pct(snap.monitoring ? 1 : snap.calProgress, 1);
    bar.classList.toggle('done', snap.monitoring);
    $('telltale').classList.toggle('on', state === 'warn');

    $('sTime').textContent = fmtTime(snap.t);
    $('sSpeed').textContent = `${snap.speedTrue.toFixed(0)} km/h`;
    $('sLat').textContent = `${Math.abs(snap.latAccel).toFixed(1)} m/s²`;
    $('sSamples').textContent = snap.radiusSamples.toLocaleString();
  }

  function renderTires() {
    snap.wheels.forEach((w, i) => {
      const el = tireEls[i];
      el.g.classList.toggle('warn', w.warn);
      const trend = w.leak > 0 ? (w.leak >= PUNCTURE ? ' ⇊' : ' ↓') : '';
      el.sub.textContent = `${w.psi.toFixed(1)} psi ${pct(w.frac)}${trend}`;
      el.est.textContent = snap.monitoring ? `est ${pct(Math.max(0, w.loss))} low` : 'est: learning';
      el.est.style.fill = w.warn ? '#ff5d5d' : snap.monitoring ? COLORS[i] : '#5c6e84';
      const scale = (x) => Math.max(0, Math.min(1, x / 0.5)) * 100;
      el.fill.setAttribute('width', snap.monitoring ? scale(w.loss) : 0);
      el.truth.setAttribute('x', el.bx + scale(1 - w.frac) - 1);
    });
    if (!dragging) {
      const v = Math.round(snap.wheels[selected].frac * 100);
      $('pSlider').value = v;
      $('pOut').textContent = `${v}%`;
    }
  }

  function renderBudget(ts) {
    const p = snap.perf;
    $('kDrop').textContent = p.dropped.toLocaleString();
    const show = (dNs, dEdges, dSim) => {
      if (dEdges <= 1000 || dSim <= 0) return;
      $('kNs').textContent = `${(dNs / dEdges).toFixed(1)} ns`;
      $('kRate').textContent = `${Math.round(dEdges / dSim).toLocaleString()} /s`;
      $('kLoad').textContent = `${((100 * dNs) / (dSim * 1e9)).toFixed(4)}%`;
    };
    if (!perfPrev) {
      // First frame (or after a fast-forward): show the run-so-far average.
      show(p.detectorNs, p.edges, p.simSeconds);
      perfPrev = p;
      perfAt = ts;
    }
    if (ts - perfAt > 1000) {
      show(p.detectorNs - perfPrev.detectorNs, p.edges - perfPrev.edges, p.simSeconds - perfPrev.simSeconds);
      perfPrev = p;
      perfAt = ts;
    }

    const total = snap.gates.reduce((a, b) => a + b, 0);
    const bar = $('gateBar');
    if (!bar.children.length) GATES.forEach((g) => {
      const d = document.createElement('div');
      d.style.background = g.color;
      bar.appendChild(d);
    });
    GATES.forEach((g, k) => {
      const d = bar.children[k];
      d.style.width = total ? `${(100 * snap.gates[k]) / total}%` : '0';
      d.title = `${g.name}: ${snap.gates[k]}`;
    });
    $('gateText').textContent = total ? `${pct(snap.gates[0] / total)} accepted` : '–';
  }

  // ---------------------------------------------------------------- URL script
  // Reproducible scenes for docs and links, e.g.
  //   ?seed=3&t=20&ev=12:FL:l0.5,15:RR:p0.8&pause=1
  // t: fast-forward (sim minutes); ev: MIN:WHEEL:(p=pressure fraction|l=leak psi/min)VALUE.
  let pending = [];

  function applyEvents() {
    while (pending.length && pending[0].min * 60 <= snap.t) {
      const e = pending.shift();
      const wheels = e.wheel === 'ALL' ? [0, 1, 2, 3] : [WHEELS.indexOf(e.wheel)];
      for (const w of wheels) {
        if (w < 0) continue;
        if (e.kind === 'l') demo.setLeak(w, e.value); else demo.setPressure(w, e.value);
      }
    }
  }

  function applyUrlScript() {
    const q = new URLSearchParams(location.search);
    if (q.has('seed')) demo.reset(Number(q.get('seed')));
    for (const [key, id, setter] of [['scenario', 'scenario', 'setScenario'], ['road', 'road', 'setRoad']]) {
      if (!q.has(key)) continue;
      $(id).value = q.get(key);
      demo[setter](Number(q.get(key)));
    }
    if (q.has('sel')) select(Number(q.get('sel')));
    if (q.has('speed')) {
      speed = Number(q.get('speed'));
      [...$('speeds').children].forEach((b) => b.classList.toggle('on', Number(b.dataset.speed) === speed));
    }
    pending = (q.get('ev') || '').split(',').filter(Boolean).map((s) => {
      const [min, wheel, spec] = s.split(':');
      return { min: Number(min), wheel: wheel.toUpperCase(), kind: spec[0], value: Number(spec.slice(1)) };
    }).sort((a, b) => a.min - b.min);

    const target = Number(q.get('t') || 0) * 60;
    snap = demo.snapshot();
    while (snap.t < target) {
      applyEvents();
      demo.step(Math.min(1, target - snap.t));
      snap = demo.snapshot();
      record();
    }
    if (q.get('pause') === '1') $('play').click();
  }

  function frame(ts) {
    const dt = lastFrame ? Math.min((ts - lastFrame) / 1000, 0.1) : 0;
    lastFrame = ts;
    if (running && dt > 0) {
      applyEvents();
      demo.step(dt * speed);
    }
    snap = demo.snapshot();
    record();
    renderStatus();
    renderTires();
    renderBudget(ts);
    drawLossChart();
    drawRadiusChart();
    for (let i = 0; i < 4; i++) drawSpectrum(i);
    requestAnimationFrame(frame);
  }

  createItpms().then((Module) => {
    demo = new Module.Demo();
    mem = Module.Demo.memory();
    resetHistory();
    buildTires();
    buildTabs();
    buildSpectra();
    buildRam();
    wireControls();
    select(0);
    applyUrlScript();
    $('loading').classList.add('gone');
    requestAnimationFrame(frame);
  }).catch((err) => {
    $('loading').textContent = `Failed to load WebAssembly: ${err}`;
  });
})();
