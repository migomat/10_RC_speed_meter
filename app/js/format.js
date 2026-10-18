// Dekodowanie formatu binarnego startu (docs/FORMAT.md, firmware/src/run_format.h,
// tools/rc_launch/parse.py). Little-endian. Konfiguracja czytana tylko z nagłówka.

export const RUN_MAGIC = 0x52463458;
export const SUPPORTED_FORMAT = 1;
export const HEADER_SIZE = 128;
export const SAMPLE_SIZE = 28;

export const SF = {
  HALL_VALID: 0x01, THROTTLE_VALID: 0x02, DIR_FORWARD: 0x04, DIR_VALID: 0x08,
  IMU_OVERRUN: 0x10, IMU_STALE: 0x20, TRIGGER: 0x40,
};
export const RF = {
  EDGES_TRUNCATED: 0x0001, TIME_SET: 0x0002, NEUTRAL_LEARNED: 0x0004,
  IMU_ERROR: 0x0008, RAM_ONLY: 0x0010, SIMULATED: 0x0020,
};

// --- CRC32 IEEE (jak zlib.crc32)
const CRC_TABLE = (() => {
  const t = new Uint32Array(256);
  for (let i = 0; i < 256; i++) {
    let c = i;
    for (let k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320 ^ (c >>> 1)) : (c >>> 1);
    t[i] = c >>> 0;
  }
  return t;
})();

export function crc32Update(crc, bytes) {
  for (let i = 0; i < bytes.length; i++) crc = CRC_TABLE[(crc ^ bytes[i]) & 0xFF] ^ (crc >>> 8);
  return crc >>> 0;
}
export function crc32(bytes) {
  return (crc32Update(0xFFFFFFFF, bytes) ^ 0xFFFFFFFF) >>> 0;
}

/** CRC startu: CRC32 całości z polem RunHeader.crc32 (bajty 124..127) = 0. */
export function runCrc(bytes) {
  if (bytes.length < HEADER_SIZE) return crc32(bytes);
  let c = crc32Update(0xFFFFFFFF, bytes.subarray(0, 124));
  c = crc32Update(c, new Uint8Array(4));
  c = crc32Update(c, bytes.subarray(128));
  return (c ^ 0xFFFFFFFF) >>> 0;
}

export class FormatError extends Error {}

export function parseHeader(buf) {
  const u8 = buf instanceof Uint8Array ? buf : new Uint8Array(buf);
  if (u8.length < HEADER_SIZE) throw new FormatError('Za krótki nagłówek');
  const dv = new DataView(u8.buffer, u8.byteOffset, u8.byteLength);
  const U32 = (o) => dv.getUint32(o, true), U16 = (o) => dv.getUint16(o, true), U8 = (o) => dv.getUint8(o);
  const h = {
    magic: U32(0), format_version: U16(4), header_size: U16(6), seq: U32(8), unix_s: U32(12),
    boot_count: U32(16), uptime_ms: U32(20), hall_lines: U8(24), motor_pole_pairs: U8(25),
    edges_per_rev: U8(26), throttle_fwd_higher: U8(27), sample_rate_hz: U16(28), imu_odr_hz: U16(30),
    accel_range_g: U16(32), gyro_range_dps: U16(34), accel_scale_g: dv.getFloat32(36, true),
    gyro_scale_dps: dv.getFloat32(40, true), throttle_neutral_us: U16(44), throttle_full_us: U16(46),
    trigger_index: U32(48), n_samples: U32(52), n_edges: U32(56), flags: U32(60), t0_us: U32(64),
    trigger_t_us: U32(68), sample_size: U16(72), edge_size: U16(74), fw_version: U32(76),
    hall_min_edge_us: U16(80), pretrigger_ms: U16(82), posttrigger_ms: U16(84), trigger_pct: U8(86),
    hall_pullup_v_x10: U8(87), hall_timeout_us: U32(88), crc32: U32(124),
  };
  if (h.magic !== RUN_MAGIC) throw new FormatError(`Zły magic 0x${h.magic.toString(16)}`);
  if (h.format_version !== SUPPORTED_FORMAT) throw new FormatError(`Nieobsługiwana wersja formatu ${h.format_version}`);
  if (h.header_size !== HEADER_SIZE || h.sample_size !== SAMPLE_SIZE || h.edge_size !== 4) {
    throw new FormatError('Niezgodne rozmiary struktur');
  }
  return h;
}

/**
 * Dekoduje start. Zwraca {header, n, t_us, hall_edges, period_rev_us, throttle_us,
 * ax..gz (Int16Array), flags, edges_t_us, edges_line, edges_level, t_s, edges_t_s}.
 */
export function parseRun(buf, { verifyCrc = true } = {}) {
  const u8 = buf instanceof Uint8Array ? buf : new Uint8Array(buf);
  const h = parseHeader(u8);
  const ns = h.n_samples, ne = h.n_edges;
  const need = HEADER_SIZE + ns * SAMPLE_SIZE + ne * 4;
  if (u8.length < need) throw new FormatError(`Za krótkie dane: ${u8.length} < ${need}`);
  if (verifyCrc && runCrc(u8.subarray(0, need)) !== h.crc32) throw new FormatError('CRC startu niezgodne');
  const dv = new DataView(u8.buffer, u8.byteOffset, need);
  const r = {
    header: h, n: ns,
    t_us: new Uint32Array(ns), hall_edges: new Uint32Array(ns), period_rev_us: new Uint32Array(ns),
    throttle_us: new Uint16Array(ns), ax: new Int16Array(ns), ay: new Int16Array(ns), az: new Int16Array(ns),
    gx: new Int16Array(ns), gy: new Int16Array(ns), gz: new Int16Array(ns), flags: new Uint8Array(ns),
    edges_t_us: new Uint32Array(ne), edges_line: new Uint8Array(ne), edges_level: new Uint8Array(ne),
    t_s: new Float64Array(ns), edges_t_s: new Float64Array(ne),
  };
  for (let i = 0; i < ns; i++) {
    const o = HEADER_SIZE + i * SAMPLE_SIZE;
    r.t_us[i] = dv.getUint32(o, true);
    r.hall_edges[i] = dv.getUint32(o + 4, true);
    r.period_rev_us[i] = dv.getUint32(o + 8, true);
    r.throttle_us[i] = dv.getUint16(o + 12, true);
    r.ax[i] = dv.getInt16(o + 14, true); r.ay[i] = dv.getInt16(o + 16, true); r.az[i] = dv.getInt16(o + 18, true);
    r.gx[i] = dv.getInt16(o + 20, true); r.gy[i] = dv.getInt16(o + 22, true); r.gz[i] = dv.getInt16(o + 24, true);
    r.flags[i] = dv.getUint8(o + 26);
    // czas względem próbki wyzwolenia (arytmetyka uint32 → int32)
    r.t_s[i] = ((r.t_us[i] - h.trigger_t_us) | 0) * 1e-6;
  }
  const eo = HEADER_SIZE + ns * SAMPLE_SIZE;
  const trigRel = ((h.trigger_t_us - h.t0_us) >>> 0);
  for (let k = 0; k < ne; k++) {
    const w = dv.getUint32(eo + k * 4, true);
    r.edges_t_us[k] = w & 0x1FFFFFFF;
    r.edges_level[k] = (w >>> 29) & 1;
    r.edges_line[k] = w >>> 30;
    r.edges_t_s[k] = (r.edges_t_us[k] - trigRel) * 1e-6;
  }
  return r;
}

export function fwVersionString(v) {
  return `${(v >>> 16) & 0xFF}.${(v >>> 8) & 0xFF}.${v & 0xFF}`;
}

// --- base64 (eksport/import bazy)
export function toBase64(u8) {
  let s = '';
  const CH = 0x8000;
  for (let i = 0; i < u8.length; i += CH) s += String.fromCharCode.apply(null, u8.subarray(i, i + CH));
  return btoa(s);
}
export function fromBase64(b64) {
  const s = atob(b64);
  const u8 = new Uint8Array(s.length);
  for (let i = 0; i < s.length; i++) u8[i] = s.charCodeAt(i);
  return u8;
}
