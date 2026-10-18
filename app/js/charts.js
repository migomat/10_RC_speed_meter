// Wykresy: uPlot (kopia lokalna w vendor/, globalny uPlot z uPlot.iife.min.js).
// Zsynchronizowane wykresy na wspólnej osi czasu, zaznaczanie zakresu palcem.

const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

export const PALETTE = ['#2a7de1', '#e8590c', '#2f9e44', '#ae3ec9', '#f59f00', '#1098ad', '#d6336c', '#5c940d'];

function axisOpts(label) {
  const grid = { stroke: css('--grid'), width: 1 };
  const ticks = { stroke: css('--grid'), width: 1 };
  return { stroke: css('--muted'), grid, ticks, label, labelSize: label ? 18 : 0, size: 46, font: '12px system-ui', labelFont: '12px system-ui' };
}

/** Plugin: przeciąganie palcem zaznacza zakres osi X (jak myszą). */
function touchZoomPlugin(onRange) {
  return {
    hooks: {
      init(u) {
        const over = u.over;
        let x0 = null, startT = 0;
        over.addEventListener('touchstart', (e) => {
          if (e.touches.length !== 1) return;
          const r = over.getBoundingClientRect();
          x0 = e.touches[0].clientX - r.left;
          startT = Date.now();
          u.setCursor({ left: x0, top: e.touches[0].clientY - r.top });
        }, { passive: true });
        over.addEventListener('touchmove', (e) => {
          if (x0 === null || e.touches.length !== 1) return;
          const r = over.getBoundingClientRect();
          const x = Math.max(0, Math.min(r.width, e.touches[0].clientX - r.left));
          const left = Math.min(x0, x), width = Math.abs(x - x0);
          if (width > 8) {
            e.preventDefault();
            u.setSelect({ left, width, top: 0, height: u.over.clientHeight }, false);
          } else {
            u.setCursor({ left: x, top: e.touches[0].clientY - r.top });
          }
        }, { passive: false });
        over.addEventListener('touchend', () => {
          if (x0 === null) return;
          const s = u.select;
          if (s.width > 8 && Date.now() - startT > 80) {
            const a = u.posToVal(s.left, 'x'), b = u.posToVal(s.left + s.width, 'x');
            onRange(a, b);
          }
          u.setSelect({ left: 0, width: 0, top: 0, height: 0 }, false);
          x0 = null;
        });
      },
    },
  };
}

/**
 * groups: [{label, unit, height, series: [{label, values, color, width, dash}]}]
 * x: Float64Array czasu. Zwraca {setRange(a,b), resetRange(), destroy()}.
 */
export function syncedCharts(container, x, groups, { syncKey = 'run', onRange = null } = {}) {
  container.innerHTML = '';
  const charts = [];
  let syncing = false;
  const fullRange = [x[0], x[x.length - 1]];
  const setAll = (a, b) => {
    syncing = true;
    for (const c of charts) c.setScale('x', { min: a, max: b });
    syncing = false;
    if (onRange) onRange(a, b);
  };
  const width = () => Math.max(260, container.clientWidth);
  for (const g of groups) {
    const wrap = document.createElement('div');
    wrap.className = 'chart';
    container.appendChild(wrap);
    const opts = {
      width: width(),
      height: g.height || 150,
      legend: { show: true, live: true },
      cursor: { sync: { key: syncKey, setSeries: false }, drag: { x: true, y: false, setScale: false }, points: { size: 6 } },
      select: { show: true },
      scales: { x: { time: false } },
      axes: [axisOpts(''), axisOpts(g.unit ? `${g.label} [${g.unit}]` : g.label)],
      series: [
        { label: 't [s]', value: (u, v) => (v == null ? '–' : v.toFixed(3)) },
        ...g.series.map((s) => ({
          label: s.label, stroke: s.color, width: s.width || 1.5, dash: s.dash, points: { show: false },
          value: (u, v) => (v == null || Number.isNaN(v) ? '–' : v.toFixed(2)),
        })),
      ],
      plugins: [touchZoomPlugin((a, b) => setAll(a, b))],
      hooks: {
        setSelect: [(u) => {
          if (u.select.width > 4) {
            setAll(u.posToVal(u.select.left, 'x'), u.posToVal(u.select.left + u.select.width, 'x'));
            u.setSelect({ left: 0, width: 0, top: 0, height: 0 }, false);
          }
        }],
      },
    };
    const data = [x, ...g.series.map((s) => s.values)];
    // eslint-disable-next-line no-undef
    charts.push(new uPlot(opts, data, wrap));
  }
  const ro = new ResizeObserver(() => {
    for (const c of charts) c.setSize({ width: width(), height: c.height });
  });
  ro.observe(container);
  return {
    charts,
    setRange: (a, b) => setAll(a, b),
    resetRange: () => setAll(fullRange[0], fullRange[1]),
    destroy: () => { ro.disconnect(); charts.forEach((c) => c.destroy()); container.innerHTML = ''; },
  };
}

/** Prosty wykres punktowy na canvasie: points = [{x, y, group}], groups → kolory. */
export function scatterChart(canvas, points, { xLabel = '', yLabel = '' } = {}) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.clientWidth || 320, h = canvas.clientHeight || 260;
  canvas.width = w * dpr;
  canvas.height = h * dpr;
  const g = canvas.getContext('2d');
  g.scale(dpr, dpr);
  g.clearRect(0, 0, w, h);
  const pad = { l: 48, r: 10, t: 10, b: 38 };
  const valid = points.filter((p) => Number.isFinite(p.x) && Number.isFinite(p.y));
  g.font = '12px system-ui';
  g.fillStyle = css('--muted');
  if (!valid.length) {
    g.fillText('Brak danych', w / 2 - 30, h / 2);
    return;
  }
  let x0 = Math.min(...valid.map((p) => p.x)), x1 = Math.max(...valid.map((p) => p.x));
  let y0 = Math.min(...valid.map((p) => p.y)), y1 = Math.max(...valid.map((p) => p.y));
  if (x1 - x0 < 1e-9) { x0 -= 1; x1 += 1; }
  if (y1 - y0 < 1e-9) { y0 -= 1; y1 += 1; }
  const mx = (x1 - x0) * 0.08, my = (y1 - y0) * 0.08;
  x0 -= mx; x1 += mx; y0 -= my; y1 += my;
  const X = (v) => pad.l + (v - x0) / (x1 - x0) * (w - pad.l - pad.r);
  const Y = (v) => h - pad.b - (v - y0) / (y1 - y0) * (h - pad.t - pad.b);
  g.strokeStyle = css('--grid');
  g.lineWidth = 1;
  for (let i = 0; i <= 4; i++) {
    const xv = x0 + (x1 - x0) * i / 4, yv = y0 + (y1 - y0) * i / 4;
    g.beginPath(); g.moveTo(X(xv), pad.t); g.lineTo(X(xv), h - pad.b); g.stroke();
    g.beginPath(); g.moveTo(pad.l, Y(yv)); g.lineTo(w - pad.r, Y(yv)); g.stroke();
    g.fillText(xv.toFixed(2), X(xv) - 12, h - pad.b + 14);
    g.fillText(yv.toFixed(2), 4, Y(yv) + 4);
  }
  g.fillText(xLabel, w / 2 - 40, h - 6);
  g.save(); g.translate(12, h / 2 + 40); g.rotate(-Math.PI / 2); g.restore();
  g.fillText(yLabel, pad.l + 4, pad.t + 12);
  const groups = [...new Set(valid.map((p) => p.group))];
  for (const p of valid) {
    g.fillStyle = PALETTE[groups.indexOf(p.group) % PALETTE.length];
    g.beginPath();
    g.arc(X(p.x), Y(p.y), 5, 0, 2 * Math.PI);
    g.fill();
  }
  return groups.map((name, i) => ({ name, color: PALETTE[i % PALETTE.length] }));
}
