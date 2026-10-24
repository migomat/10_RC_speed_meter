// Synchronizacja (sync.js) dwóch „urządzeń” przez atrapę API GitHub (Git Data API w pamięci).
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

import { GitHubStore, syncAll } from '../js/sync.js';

const here = dirname(fileURLToPath(import.meta.url));
const VEC = join(here, '..', '..', 'test_vectors');
const bin = (name) => { const u = new Uint8Array(readFileSync(join(VEC, `${name}.bin`))); return u.buffer.slice(u.byteOffset, u.byteOffset + u.byteLength); };

// --- atrapa GitHub
function fakeGitHub() {
  let n = 0;
  const sha = () => (++n).toString(16).padStart(40, '0');
  const blobs = new Map(), trees = new Map(), commits = new Map();
  const t0 = sha(); trees.set(t0, new Map());
  const c0 = sha(); commits.set(c0, { tree: t0 });
  const gh = { head: c0, calls: 0, failNextPatch: false };
  const json = (status, obj) => ({ ok: status < 300, status, json: async () => obj, text: async () => JSON.stringify(obj) });
  gh.fetch = async (url, opt) => {
    gh.calls++;
    const path = url.replace(/^https:\/\/api\.github\.com\/repos\/[^/]+\/[^/]+/, '');
    const body = opt.body ? JSON.parse(opt.body) : null;
    const m = opt.method;
    if (m === 'GET' && path === '/git/ref/heads/main') return json(200, { object: { sha: gh.head } });
    let r;
    if (m === 'GET' && (r = path.match(/^\/git\/commits\/(\w+)$/))) return json(200, { tree: { sha: commits.get(r[1]).tree } });
    if (m === 'GET' && (r = path.match(/^\/git\/trees\/(\w+)\?recursive=1$/))) {
      return json(200, { truncated: false, tree: [...trees.get(r[1])].map(([p, s]) => ({ path: p, type: 'blob', sha: s })) });
    }
    if (m === 'GET' && (r = path.match(/^\/git\/blobs\/(\w+)$/))) return json(200, { content: blobs.get(r[1]), encoding: 'base64' });
    if (m === 'POST' && path === '/git/blobs') { const s = sha(); blobs.set(s, body.content); return json(201, { sha: s }); }
    if (m === 'POST' && path === '/git/trees') {
      const t = new Map(trees.get(body.base_tree));
      for (const e of body.tree) t.set(e.path, e.sha);
      const s = sha(); trees.set(s, t); return json(201, { sha: s });
    }
    if (m === 'POST' && path === '/git/commits') { const s = sha(); commits.set(s, { tree: body.tree, parent: body.parents[0] }); return json(201, { sha: s }); }
    if (m === 'PATCH' && path === '/git/refs/heads/main') {
      const c = commits.get(body.sha);
      if (gh.failNextPatch || c.parent !== gh.head) { gh.failNextPatch = false; return json(422, { message: 'Update is not a fast forward' }); }
      gh.head = body.sha; return json(200, {});
    }
    return json(404, { message: `nieznane ${m} ${path}` });
  };
  gh.paths = () => [...trees.get(commits.get(gh.head).tree).keys()].sort();
  return gh;
}

// --- atrapa IndexedDB (storage.js)
function fakeDb() {
  const runs = new Map(), races = new Map(), kv = new Map();
  const clone = (r) => (r ? { ...r } : r);
  return {
    runs, races,
    allRuns: async () => [...runs.values()].map(clone), putRun: async (r) => { runs.set(r.id, clone(r)); },
    deleteRun: async (id) => { runs.delete(id); },
    allRaces: async () => [...races.values()].map(clone), putRace: async (r) => { races.set(r.id, clone(r)); },
    deleteRace: async (id) => { races.delete(id); },
    kvGet: async (k) => kv.get(k), kvSet: async (k, v) => { kv.set(k, v); },
  };
}

const store = (gh) => new GitHubStore({ repo: 'u/x4f-dane', token: 't', fetchFn: gh.fetch });

test('synchronizacja: telefon → repo → komputer, tagi w obie strony, usuwanie, konflikt', async () => {
  const gh = fakeGitHub();
  const phone = fakeDb(), pc = fakeDb();
  await phone.putRun({ id: 'r1', seq: 1, tag: 'punch 3', note: '', updated_at: 100, raw: bin('mode1_grip') });
  await phone.putRace({ id: 'race-1', seq: 2, tag: '', note: 'salon', updated_at: 100, raw: bin('race_oval') });

  let st = await syncAll(store(gh), phone, { device: 'telefon' });
  assert.equal(st.up, 2);
  assert.deepEqual(gh.paths(), ['races/race-1.bin', 'races/race-1.json', 'runs/r1.bin', 'runs/r1.json']);

  st = await syncAll(store(gh), pc, { device: 'komputer' });
  assert.equal(st.down, 2);
  assert.equal(pc.runs.get('r1').tag, 'punch 3');
  assert.deepEqual(new Uint8Array(pc.races.get('race-1').raw), new Uint8Array(bin('race_oval')));

  // bez zmian → bez commitu
  const head = gh.head;
  st = await syncAll(store(gh), pc, { device: 'komputer' });
  assert.equal(gh.head, head);
  assert.deepEqual(st, { up: 0, down: 0, deleted: 0, meta: 0 });

  // zmiana tagu na komputerze → telefon dostaje
  await pc.putRun({ ...pc.runs.get('r1'), tag: 'punch 4', updated_at: 200 });
  await syncAll(store(gh), pc, { device: 'komputer' });
  st = await syncAll(store(gh), phone, { device: 'telefon' });
  assert.equal(phone.runs.get('r1').tag, 'punch 4');

  // konflikt: obie strony zmieniły notatkę – wygrywa nowsza
  await phone.putRun({ ...phone.runs.get('r1'), note: 'telefon', updated_at: 300 });
  await pc.putRun({ ...pc.runs.get('r1'), note: 'komputer', updated_at: 400 });
  await syncAll(store(gh), phone, { device: 'telefon' });
  await syncAll(store(gh), pc, { device: 'komputer' });
  await syncAll(store(gh), phone, { device: 'telefon' });
  assert.equal(phone.runs.get('r1').note, 'komputer');
  assert.equal(pc.runs.get('r1').note, 'komputer');

  // usunięcie na telefonie → komputer usuwa, surowe dane zostają w repo (kopia)
  await phone.deleteRace('race-1');
  await phone.kvSet('sync_deleted', [{ dir: 'races', id: 'race-1' }]);
  await syncAll(store(gh), phone, { device: 'telefon' });
  st = await syncAll(store(gh), pc, { device: 'komputer' });
  assert.equal(st.deleted, 1);
  assert.equal(pc.races.size, 0);
  assert.ok(gh.paths().includes('races/race-1.bin'));
  // i nie wraca na telefon
  await syncAll(store(gh), phone, { device: 'telefon' });
  assert.equal(phone.races.size, 0);
});

test('synchronizacja: równoczesny commit innego urządzenia → ponowienie', async () => {
  const gh = fakeGitHub();
  const a = fakeDb();
  await a.putRun({ id: 'r9', seq: 9, tag: '', note: '', updated_at: 1, raw: bin('mode3_grip') });
  gh.failNextPatch = true;
  const st = await syncAll(store(gh), a, { device: 'a' });
  assert.equal(st.up, 1);
  assert.ok(gh.paths().includes('runs/r9.bin'));
});
