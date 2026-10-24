// Wyścigi: parseRace (format.js) i analyzeRace (analysis.js) na wspólnych wektorach.
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

import { parseRace, raceCrc, fileKind, FormatError, RSF_GAP } from '../js/format.js';
import { analyzeRace } from '../js/analysis.js';

const here = dirname(fileURLToPath(import.meta.url));
const VEC = join(here, '..', '..', 'test_vectors');
const races = JSON.parse(readFileSync(join(VEC, 'races.json'), 'utf8'));
const load = (name) => new Uint8Array(readFileSync(join(VEC, `${name}.bin`)));

function close(got, exp, rel = 0.01, abs = 1e-3) {
  if (exp === null) return got === null || Number.isNaN(got);
  return Math.abs(got - exp) <= Math.max(abs, rel * Math.abs(exp));
}

for (const name of races) {
  test(`wyścig ${name}: metryki i przebiegi jak analyze_race`, () => {
    const exp = JSON.parse(readFileSync(join(VEC, `${name}.expected.json`), 'utf8'));
    const res = analyzeRace(parseRace(load(name)), exp.config);
    for (const [k, v] of Object.entries(exp.metrics)) {
      assert.ok(close(res.metrics[k], v), `${k}: ${res.metrics[k]} != ${v}`);
    }
    for (const [k, vals] of Object.entries(exp.series)) {
      exp.check_idx.forEach((i, j) => {
        assert.ok(close(res.series[k][i], vals[j], 0.01, 2e-3), `${k}[${i}]: ${res.series[k][i]} != ${vals[j]}`);
      });
    }
    assert.equal(res.end_reason, exp.end_reason);
    assert.equal(res.i_start, exp.i_start);
    assert.equal(res.i_last, exp.i_last);
    assert.deepEqual(res.warnings, exp.warnings);
    assert.equal(res.laps.length, exp.laps.length);
    res.laps.forEach((lp, j) => {
      const e = exp.laps[j];
      assert.deepEqual([lp.i_a, lp.i_b, lp.good], [e.i_a, e.i_b, e.good]);
      for (const k of ['time_s', 'dist_m', 'v_max', 'err_m']) assert.ok(close(lp[k], e[k]), `lap ${j} ${k}: ${lp[k]} != ${e[k]}`);
    });
    assert.ok(close(res.track.length_m, exp.track.length_m));
    for (const k of ['x', 'y', 'v']) {
      exp.track[k].forEach((v, j) => assert.ok(close(res.track[k][j * 20], v, 0.01, 2e-3), `track ${k}[${j * 20}]`));
    }
  });
}

test('wyścig: rodzaj pliku, CRC, wypełnienie i przerwa', () => {
  const data = load('race_recovered');
  assert.equal(fileKind(data), 'race');
  const r = parseRace(data);
  assert.equal(raceCrc(data), r.crc32);
  assert.equal(r.end_reason, 2);
  assert.ok(data.length > 128 + r.n * 20 + 16);
  let gap = -1;
  for (let i = 0; i < r.n; i++) if (r.flags[i] & RSF_GAP) gap = i;
  assert.ok(gap > 0);
  assert.ok(Math.abs(r.t_s[gap] - r.t_s[gap - 1] - 30 / 500) < 1e-9);
  assert.equal(r.t_s[r.header.trigger_index], 0);
  const bad = data.slice();
  bad[300] ^= 1;
  assert.throws(() => parseRace(bad), FormatError);
});
