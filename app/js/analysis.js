// Analiza startu – port 1:1 tools/rc_launch/analyze.py (implementacja referencyjna).
// Każda zmiana algorytmu: jednocześnie tutaj i w analyze.py + wektory testowe.

import { DEFAULT_EST_PARAMS, estInit, estStep } from './estimator.js';

const G = 9.80665;

export const DEFAULT_SETTINGS = {
  final_drive_ratio: 5.0,
  front_tire_diameter_mm: 64.0,
  imu_axes: { forward: '+x', left: '+y', up: '+z' },
  lowpass_hz: 25,
  v_min_mps: 0.3,
  rest_gyro_std_dps: 1.5,
  rest_acc_std_g: 0.06,   // po filtrze lowpass_hz (drgania wentylatora ESC ~0,03–0,05 g)
  estimator: { ...DEFAULT_EST_PARAMS },
};

// Zmienione wartości domyślne: zapisane ustawienie równe staremu domyślnemu → nowe.
const SETTINGS_MIGRATIONS = [
  [['rest_acc_std_g'], 0.02, 0.06],
  [['estimator', 'zupt_acc_g'], 0.04, 0.06],
];

export function mergeSettings(user) {
  const s = JSON.parse(JSON.stringify(DEFAULT_SETTINGS));
  if (!user) return s;
  user = JSON.parse(JSON.stringify(user));
  for (const [path, oldV, newV] of SETTINGS_MIGRATIONS) {
    const o = path.length > 1 ? user[path[0]] : user;
    const k = path[path.length - 1];
    if (o && o[k] === oldV) o[k] = newV;
  }
  for (const [k, v] of Object.entries(user)) {
    if (v && typeof v === 'object' && !Array.isArray(v) && typeof s[k] === 'object') Object.assign(s[k], v);
    else if (v !== undefined && v !== null) s[k] = v;
  }
  return s;
}

// ------------------------------------------------------------------ filtr
export function butter2Coeffs(fc, fs) {
  const k = Math.tan(Math.PI * fc / fs);
  const norm = 1 / (1 + Math.SQRT2 * k + k * k);
  const b0 = k * k * norm;
  return { b: [b0, 2 * b0, b0], a: [2 * (k * k - 1) * norm, (1 - Math.SQRT2 * k + k * k) * norm] };
}

function biquadPass(x, b, a) {
  const [b0, b1, b2] = b, [a1, a2] = a;
  const n = x.length;
  const y = new Float64Array(n);
  if (!n) return y;
  const u = x[0];
  let z1 = (b1 - a1 + b2 - a2) * u;
  let z2 = (b2 - a2) * u;
  for (let i = 0; i < n; i++) {
    const xi = x[i];
    const yi = b0 * xi + z1;
    z1 = b1 * xi - a1 * yi + z2;
    z2 = b2 * xi - a2 * yi;
    y[i] = yi;
  }
  return y;
}

/** Zero-fazowy LP – odbicie nieparzyste, padlen = min(n−1, 3·ceil(fs/fc)). */
export function filtfiltButter2(x, fc, fs) {
  const n = x.length;
  if (n < 2 || fc <= 0 || fc >= fs / 2) return Float64Array.from(x);
  const { b, a } = butter2Coeffs(fc, fs);
  const pad = Math.min(n - 1, 3 * Math.ceil(fs / fc));
  const ext = new Float64Array(n + 2 * pad);
  for (let i = 0; i < pad; i++) ext[i] = 2 * x[0] - x[pad - i];
  for (let i = 0; i < n; i++) ext[pad + i] = x[i];
  for (let i = 0; i < pad; i++) ext[pad + n + i] = 2 * x[n - 1] - x[n - 2 - i];
  let y = biquadPass(ext, b, a);
  y.reverse();
  y = biquadPass(y, b, a);
  y.reverse();
  return y.slice(pad, pad + n);
}

export function gradient(v, t) {
  const n = v.length;
  const out = new Float64Array(n);
  if (n < 2) return out;
  out[0] = (v[1] - v[0]) / (t[1] - t[0]);
  out[n - 1] = (v[n - 1] - v[n - 2]) / (t[n - 1] - t[n - 2]);
  for (let i = 1; i < n - 1; i++) out[i] = (v[i + 1] - v[i - 1]) / (t[i + 1] - t[i - 1]);
  return out;
}

export function interpAt(t, y, tq) {
  const n = t.length;
  if (!n || tq < t[0] || tq > t[n - 1]) return NaN;
  for (let i = 1; i < n; i++) {
    if (t[i] >= tq) {
      if (t[i] === t[i - 1]) return y[i];
      const f = (tq - t[i - 1]) / (t[i] - t[i - 1]);
      return y[i - 1] + f * (y[i] - y[i - 1]);
    }
  }
  return y[n - 1];
}

export function firstCrossing(t, y, level, iFrom = 0) {
  for (let i = Math.max(iFrom, 1); i < y.length; i++) {
    if (y[i] >= level && y[i - 1] < level) {
      const f = (level - y[i - 1]) / (y[i] - y[i - 1]);
      return t[i - 1] + f * (t[i] - t[i - 1]);
    }
    if (i === iFrom && y[i] >= level) return t[i];
  }
  return NaN;
}

// ------------------------------------------------------------------ osie
function parseAxis(spec) {
  spec = String(spec).trim().toLowerCase();
  return { idx: 'xyz'.indexOf(spec[spec.length - 1]), sign: spec.startsWith('-') ? -1 : 1 };
}

function mapAxes(bx, by, bz, axes) {
  const board = [bx, by, bz];
  return ['forward', 'left', 'up'].map((name) => {
    const { idx, sign } = parseAxis(axes[name]);
    const src = board[idx];
    const out = new Float64Array(src.length);
    for (let i = 0; i < src.length; i++) out[i] = src[i] * sign;
    return out;
  });
}

// ------------------------------------------------------------------ v_front
export function vFrontFromPeriods(period, mPerRev) {
  const out = new Float64Array(period.length);
  for (let i = 0; i < period.length; i++) out[i] = period[i] > 0 ? mPerRev * 1e6 / period[i] : 0;
  return out;
}

export function vFrontFromEdges(tS, eT, epr, timeoutS, mPerRev) {
  const n = tS.length, ne = eT.length;
  const vk = new Float64Array(ne);
  let runStart = 0;
  for (let k = 0; k < ne; k++) {
    if (k === 0 || eT[k] - eT[k - 1] > timeoutS) runStart = k;
    const kr = k - runStart;
    if (kr >= epr) vk[k] = mPerRev / (eT[k] - eT[k - epr]);
    else if (kr >= 1) vk[k] = mPerRev * (kr / epr) / (eT[k] - eT[k - kr]);
    else vk[k] = 0;
  }
  const out = new Float64Array(n);
  let k = -1;
  runStart = 0;
  for (let i = 0; i < n; i++) {
    const t = tS[i];
    while (k + 1 < ne && eT[k + 1] <= t) {
      k++;
      if (k === 0 || eT[k] - eT[k - 1] > timeoutS) runStart = k;
    }
    if (k < 0 || t - eT[k] > timeoutS) { out[i] = 0; continue; }
    let v = vk[k];
    const j = k + 1 - epr;
    const ref = j >= runStart ? eT[j] : eT[runStart];
    if (t > ref) {
      const bound = mPerRev / (t - ref);
      if (bound < v) v = bound;
    }
    out[i] = v;
  }
  return out;
}

// ------------------------------------------------------------------ analiza
export function analyze(run, settings) {
  const cfg = mergeSettings(settings);
  const h = run.header;
  const n = run.n;
  const fs = h.sample_rate_hz;
  const t = run.t_s;
  const ti = h.trigger_index;
  const warnings = [];

  const ka = h.accel_scale_g, kg = h.gyro_scale_dps;
  const sc = (arr, k) => { const o = new Float64Array(arr.length); for (let i = 0; i < arr.length; i++) o[i] = arr[i] * k; return o; };
  const [aFwd, aLeft, aUp] = mapAxes(sc(run.ax, ka), sc(run.ay, ka), sc(run.az, ka), cfg.imu_axes);
  const [gFwd, gLeft, gUp] = mapAxes(sc(run.gx, kg), sc(run.gy, kg), sc(run.gz, kg), cfg.imu_axes);

  // start: wcześniejsze z (wyzwolenie gazem, pierwsze zbocze halla po ostatnim
  // postoju przed wyzwoleniem); postój w próbce: period_rev_us = 0 i brak zbocza
  // dłużej niż hall_timeout (period = 0 bywa też tuż po ruszeniu – okres nieznany)
  const edges = run.hall_edges;
  const period = run.period_rev_us;
  const timeoutS = h.hall_timeout_us * 1e-6;
  const lastChg = new Int32Array(n).fill(-1);
  for (let i = 1; i < n; i++) lastChg[i] = edges[i] !== edges[i - 1] ? i : lastChg[i - 1];
  const stopped = (i) => period[i] === 0 && (lastChg[i] < 0 || t[i] - t[lastChg[i]] > timeoutS);
  let iStop = -1;
  for (let i = Math.min(ti, n - 1); i >= 0; i--) if (stopped(i)) { iStop = i; break; }
  let iEdge0 = n;
  if (iStop >= 0) {
    for (let i = iStop + 1; i < n; i++) if (edges[i] !== edges[i - 1]) { iEdge0 = i; break; }
  } else {
    warnings.push('brak postoju przed wyzwoleniem');
  }
  const i0 = Math.min(ti, iEdge0);
  // okno spoczynku: ciągły postój kończący się w iStop (najwyżej do i0)
  let restStart = 0;
  if (iStop >= 0) {
    restStart = iStop;
    while (restStart > 0 && stopped(restStart - 1)) restStart--;
  }

  // 3. bias
  let restEnd = i0;
  if (restEnd - restStart < 50) {
    warnings.push('krótkie okno spoczynku przed startem');
    restStart = Math.max(0, restEnd - 50);
    restEnd = restStart + Math.min(50, n);
  }
  const rn = restEnd - restStart;
  const mean = (x) => { let s = 0; for (let i = restStart; i < restEnd; i++) s += x[i]; return s / rn; };
  const std = (x) => { const m = mean(x); let s = 0; for (let i = restStart; i < restEnd; i++) s += (x[i] - m) ** 2; return Math.sqrt(s / rn); };
  const amag = new Float64Array(n);
  for (let i = 0; i < n; i++) amag[i] = Math.sqrt(aFwd[i] ** 2 + aLeft[i] ** 2 + aUp[i] ** 2);
  // rozrzut po filtrze lowpass_hz (tylko okno spoczynku) – drgania wentylatora
  // ESC (~180 Hz) nie mogą udawać ruchu auta
  const lp = (x) => {
    const y = new Float64Array(n);
    y.set(filtfiltButter2(x.subarray(restStart, restEnd), cfg.lowpass_hz, fs), restStart);
    return y;
  };
  const atRest = Math.max(std(lp(gFwd)), std(lp(gLeft)), std(lp(gUp))) < cfg.rest_gyro_std_dps &&
    std(lp(amag)) < cfg.rest_acc_std_g && edges[restEnd - 1] === edges[restStart];
  if (!atRest) warnings.push('auto nie stało przed startem – bias niepewny');
  const aRef = mean(aFwd);
  const gbLeft = mean(gLeft);

  // 4.
  const dt = new Float64Array(n);
  for (let i = 1; i < n; i++) dt[i] = t[i] - t[i - 1];
  const pitch = new Float64Array(n);
  let acc = 0;
  for (let i = 0; i < n; i++) {
    acc += -(gLeft[i] - gbLeft) * dt[i] * (Math.PI / 180);
    pitch[i] = acc;
  }
  const aLongRaw = new Float64Array(n);
  for (let i = 0; i < n; i++) aLongRaw[i] = G * (aFwd[i] - aRef - Math.sin(pitch[i]));

  // 5.
  const fc = cfg.lowpass_hz;
  const aLong = filtfiltButter2(aLongRaw, fc, fs);

  // 6.
  const vImu = new Float64Array(n);
  for (let i = i0 + 1; i < n; i++) vImu[i] = vImu[i - 1] + aLong[i] * dt[i];

  // 7.
  const epr = h.edges_per_rev;
  const mPerRev = Math.PI * cfg.front_tire_diameter_mm * 1e-3 / cfg.final_drive_ratio;
  const useEdges = h.n_edges > 0 && !(h.flags & 0x0001);
  const vFront = useEdges ? vFrontFromEdges(t, run.edges_t_s, epr, timeoutS, mPerRev)
    : vFrontFromPeriods(run.period_rev_us, mPerRev);
  const vFrontSample = vFrontFromPeriods(run.period_rev_us, mPerRev);

  // 8.
  const aFront = gradient(filtfiltButter2(vFront, fc, fs), t);

  // 9.–10.
  const vmin = cfg.v_min_mps;
  const slip = new Float64Array(n);
  for (let i = i0; i < n; i++) slip[i] = (vFront[i] - vImu[i]) / Math.max(vFront[i], vmin);
  const spin = new Float64Array(n);
  for (let i = 0; i < n; i++) spin[i] = aFront[i] - aLong[i];

  // 11.
  const dFront = new Float64Array(n);
  for (let i = i0; i < n; i++) dFront[i] = Math.max(0, edges[i] - edges[i0]) / epr * mPerRev;
  const dImu = new Float64Array(n);
  for (let i = i0 + 1; i < n; i++) dImu[i] = dImu[i - 1] + vImu[i] * dt[i];

  // 12. v_disp
  const neutral = h.throttle_neutral_us;
  const full = h.throttle_full_us || 500;
  const sgn = h.throttle_fwd_higher ? 1 : -1;
  const thrPct = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const w = run.throttle_us[i];
    thrPct[i] = (w && neutral) ? (w - neutral) * 100 / full * sgn : 0;
  }
  const p = { ...DEFAULT_EST_PARAMS, ...(cfg.estimator || {}) };
  const st = estInit();
  const vDisp = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    if (i > 0) {
      const stopped = run.period_rev_us[i] === 0 && edges[i] === edges[i - 1];
      estStep(st, p, dt[i], aFwd[i], aLeft[i], aUp[i], gFwd[i], gLeft[i], gUp[i], vFrontSample[i], stopped, thrPct[i]);
    }
    vDisp[i] = st.v;
  }

  let tFirstEdge = NaN;
  // (pierwsze zbocze po próbce postoju iStop)
  if (h.n_edges > 0 && iStop >= 0) {
    const tStop = t[iStop];
    for (const te of run.edges_t_s) if (te > tStop) { tFirstEdge = te; break; }
  } else if (iEdge0 < n) tFirstEdge = t[iEdge0];

  const pitchDeg = new Float64Array(n);
  for (let i = 0; i < n; i++) pitchDeg[i] = pitch[i] * 180 / Math.PI;

  const series = {
    t, throttle_pct: thrPct, v_front: vFront, v_imu: vImu, v_disp: vDisp, a_front: aFront,
    a_long: aLong, slip, spin, pitch_deg: pitchDeg, d_front: dFront, d_imu: dImu,
  };
  const metrics = computeMetrics(series, i0, vmin, tFirstEdge);
  return { series, metrics, warnings, at_rest: atRest, header: h, used_raw_edges: useEdges, i0 };
}

function windowMean(t, y, t0, t1) {
  let s = 0, c = 0;
  for (let i = 0; i < t.length; i++) if (t0 <= t[i] && t[i] < t1) { s += y[i]; c++; }
  return c ? s / c : NaN;
}

export function computeMetrics(sr, i0, vmin, tFirstEdge) {
  const t = sr.t;
  const t0 = t[i0];
  const n = t.length;
  let peakSlip = NaN, peakT = NaN;
  for (let i = i0; i < n; i++) {
    if (sr.v_front[i] >= vmin && (Number.isNaN(peakSlip) || sr.slip[i] > peakSlip)) {
      peakSlip = sr.slip[i];
      peakT = t[i] - t0;
    }
  }
  let spinInt = 0;
  for (let i = i0 + 1; i < n; i++) {
    const x = sr.spin[i];
    if (x > 0) spinInt += x * (t[i] - t[i - 1]);
  }
  let vmax = -Infinity;
  for (let i = 0; i < n; i++) if (sr.v_disp[i] > vmax) vmax = sr.v_disp[i];
  return {
    a_long_0_300ms: windowMean(t, sr.a_long, t0, t0 + 0.3),
    a_long_0_1000ms: windowMean(t, sr.a_long, t0, t0 + 1.0),
    v_imu_0_5s: interpAt(t, sr.v_imu, t0 + 0.5),
    v_imu_1_0s: interpAt(t, sr.v_imu, t0 + 1.0),
    v_max: vmax,
    t_1m: firstCrossing(t, sr.d_imu, 1.0, i0) - t0,
    t_2m: firstCrossing(t, sr.d_imu, 2.0, i0) - t0,
    peak_slip: peakSlip,
    peak_slip_t: peakT,
    spin_integral: spinInt,
    esc_delay_s: tFirstEdge,
    t_launch_s: t0,
  };
}

export const METRIC_LABELS = {
  a_long_0_300ms: ['a śr. 0–0,3 s', 'm/s²'],
  a_long_0_1000ms: ['a śr. 0–1 s', 'm/s²'],
  v_imu_0_5s: ['v po 0,5 s', 'm/s'],
  v_imu_1_0s: ['v po 1 s', 'm/s'],
  v_max: ['v maks.', 'm/s'],
  t_1m: ['czas 1 m', 's'],
  t_2m: ['czas 2 m', 's'],
  peak_slip: ['szczyt poślizgu', '%'],
  peak_slip_t: ['czas szczytu', 's'],
  spin_integral: ['przebuksowanie', 'm/s'],
  esc_delay_s: ['opóźnienie ESC', 'ms'],
};
