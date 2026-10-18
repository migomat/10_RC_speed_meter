// Testy analysis.js na wspólnych wektorach testowych (tolerancja metryk 1 %).
// Uruchomienie: node --test app/test/   (Node ≥ 18)
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

import { parseRun, runCrc, FormatError } from '../js/format.js';
import { analyze, filtfiltButter2 } from '../js/analysis.js';

const here = dirname(fileURLToPath(import.meta.url));
const VEC = join(here, '..', '..', 'test_vectors');
const names = JSON.parse(readFileSync(join(VEC, 'index.json'), 'utf8'));

function close(got, exp, rel = 0.01, abs = 1e-3) {
  if (exp === null) return got === null || Number.isNaN(got);
  return Math.abs(got - exp) <= Math.max(abs, rel * Math.abs(exp));
}

for (const name of names) {
  test(`wektor ${name}: metryki i przebiegi jak analyze.py`, () => {
    const data = new Uint8Array(readFileSync(join(VEC, `${name}.bin`)));
    const exp = JSON.parse(readFileSync(join(VEC, `${name}.expected.json`), 'utf8'));
    const run = parseRun(data);
    const res = analyze(run, exp.config);
    for (const [k, v] of Object.entries(exp.metrics)) {
      assert.ok(close(res.metrics[k], v), `${k}: ${res.metrics[k]} != ${v}`);
    }
    for (const [k, vals] of Object.entries(exp.series)) {
      exp.check_idx.forEach((i, j) => {
        assert.ok(close(res.series[k][i], vals[j], 0.01, 2e-3), `${k}[${i}]: ${res.series[k][i]} != ${vals[j]}`);
      });
    }
    // estymator: cały przebieg v_disp
    exp.v_disp_full.forEach((v, i) => {
      assert.ok(close(res.series.v_disp[i], v, 0.01, 2e-3), `v_disp[${i}]`);
    });
    assert.equal(res.at_rest, exp.at_rest);
    assert.equal(res.i0, exp.i0);
    assert.deepEqual(res.warnings, exp.warnings);
  });
}

test('CRC startu i wykrywanie uszkodzenia', () => {
  const data = new Uint8Array(readFileSync(join(VEC, `${names[0]}.bin`)));
  const run = parseRun(data);
  assert.equal(runCrc(data), run.header.crc32);
  const bad = data.slice();
  bad[500] ^= 0xFF;
  assert.throws(() => parseRun(bad), FormatError);
});

test('przepełnienie licznika czasu (uint32)', () => {
  const run = parseRun(new Uint8Array(readFileSync(join(VEC, `${names[0]}.bin`))));
  let wraps = 0;
  for (let i = 1; i < run.n; i++) {
    if (run.t_us[i] < run.t_us[i - 1]) wraps++;
    assert.ok(Math.abs(run.t_s[i] - run.t_s[i - 1] - 0.002) < 1e-9);
  }
  assert.equal(wraps, 1);
});

test('filtr: stała na wejściu → stała na wyjściu', () => {
  const y = filtfiltButter2(new Float64Array(100).fill(3.5), 25, 500);
  for (const v of y) assert.ok(Math.abs(v - 3.5) < 1e-12);
});
