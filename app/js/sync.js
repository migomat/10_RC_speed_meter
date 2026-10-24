// Synchronizacja bazy między urządzeniami przez prywatne repozytorium GitHub.
// Układ repozytorium:  runs/<id>.bin + runs/<id>.json,  races/<id>.bin + races/<id>.json
//   .bin  – surowy start / wyścig (oryginalny format binarny, nigdy nie zmieniany),
//   .json – metadane (tag, notatka, czasy, updated_at; deleted: true = usunięty na którymś urządzeniu).
// Jedna synchronizacja = najwyżej jeden commit (Git Data API). Token tylko na urządzeniu (kv 'sync').
// Konflikt metadanych: wygrywa nowszy updated_at. Surowe dane są niezmienne – nie ma konfliktów.

import { toBase64, fromBase64, parseRun, parseRace } from './format.js';

const API = 'https://api.github.com';
const SYNC_FIELDS = ['raw', 'sync_meta', 'sync_meta_sha'];

export class SyncError extends Error {
  constructor(msg, status) { super(msg); this.status = status; }
}

export class GitHubStore {
  constructor({ repo, token, branch = 'main', fetchFn = (...a) => fetch(...a) }) {
    this.repo = repo;
    this.token = token;
    this.branch = branch;
    this.fetch = fetchFn;
  }

  async req(method, path, body) {
    let r;
    try {
      r = await this.fetch(`${API}/repos/${this.repo}${path}`, {
        method,
        headers: {
          Authorization: `Bearer ${this.token}`,
          Accept: 'application/vnd.github+json',
          'X-GitHub-Api-Version': '2022-11-28',
          ...(body ? { 'Content-Type': 'application/json' } : {}),
        },
        body: body ? JSON.stringify(body) : undefined,
        cache: 'no-store',
      });
    } catch (e) {
      throw new SyncError('Brak połączenia z GitHub (internet?)', 0);
    }
    if (!r.ok) {
      let msg = '';
      try { msg = (await r.json()).message || ''; } catch (e) { /* */ }
      if (r.status === 401) throw new SyncError('Token odrzucony przez GitHub (nieważny albo wygasł)', 401);
      if (r.status === 403 || r.status === 404) throw new SyncError(`Brak dostępu do repozytorium ${this.repo} (token bez uprawnień „Contents: read and write” do tego repo?)`, r.status);
      throw new SyncError(`GitHub ${r.status}: ${msg}`, r.status);
    }
    return r.status === 204 ? null : r.json();
  }

  async head() {
    const ref = await this.req('GET', `/git/ref/heads/${this.branch}`);
    return ref.object.sha;
  }

  /** Pliki w commicie: Map ścieżka → sha bloba. */
  async files(commitSha) {
    const c = await this.req('GET', `/git/commits/${commitSha}`);
    const t = await this.req('GET', `/git/trees/${c.tree.sha}?recursive=1`);
    if (t.truncated) throw new SyncError('Repozytorium za duże do odczytu jednym zapytaniem', 0);
    const m = new Map();
    for (const e of t.tree) if (e.type === 'blob') m.set(e.path, e.sha);
    return { treeSha: c.tree.sha, files: m };
  }

  async blob(sha) {
    const b = await this.req('GET', `/git/blobs/${sha}`);
    return fromBase64(b.content.replace(/\s/g, ''));
  }

  /** Commit z plikami [{path, bytes}]; zwraca Map ścieżka → sha bloba. Konflikt (ktoś zdążył) → status 422. */
  async commit(parent, baseTree, files, message) {
    const tree = [];
    const shas = new Map();
    for (const f of files) {
      const b = await this.req('POST', '/git/blobs', { content: toBase64(f.bytes), encoding: 'base64' });
      tree.push({ path: f.path, mode: '100644', type: 'blob', sha: b.sha });
      shas.set(f.path, b.sha);
    }
    const t = await this.req('POST', '/git/trees', { base_tree: baseTree, tree });
    const c = await this.req('POST', '/git/commits', { message, tree: t.sha, parents: [parent] });
    await this.req('PATCH', `/git/refs/heads/${this.branch}`, { sha: c.sha, force: false });
    return shas;
  }
}

// JSON z posortowanymi kluczami – porównanie metadanych niezależne od kolejności pól
export function canonical(o) {
  if (Array.isArray(o)) return `[${o.map(canonical).join(',')}]`;
  if (o && typeof o === 'object') {
    return `{${Object.keys(o).sort().filter((k) => o[k] !== undefined).map((k) => `${JSON.stringify(k)}:${canonical(o[k])}`).join(',')}}`;
  }
  return JSON.stringify(o);
}

export function metaOf(rec) {
  const m = {};
  for (const [k, v] of Object.entries(rec)) if (!SYNC_FIELDS.includes(k)) m[k] = v;
  return m;
}

const enc = new TextEncoder(), dec = new TextDecoder();

/**
 * Synchronizacja w obie strony. db: {allRuns, putRun, deleteRun, allRaces, putRace, deleteRace, kvGet, kvSet}.
 * Zwraca {up, down, deleted, meta} – liczby wysłanych / pobranych / usuniętych nagrań i zmian metadanych.
 */
export async function syncAll(store, db, { onProgress = () => {}, device = 'urządzenie', retry = 1 } = {}) {
  const kinds = [
    { dir: 'runs', all: db.allRuns, put: db.putRun, del: db.deleteRun, parse: parseRun, name: 'start' },
    { dir: 'races', all: db.allRaces, put: db.putRace, del: db.deleteRace, parse: parseRace, name: 'wyścig' },
  ];
  onProgress('Odczyt repozytorium…');
  const head = await store.head();
  const { treeSha, files } = await store.files(head);
  const tomb = (await db.kvGet('sync_deleted')) || [];   // [{dir, id}] usunięte lokalnie, do wysłania
  const isTomb = (dir, id) => tomb.some((t) => t.dir === dir && t.id === id);
  const uploads = [];           // [{path, bytes}]
  const afterCommit = [];       // [{rec, put, metaStr, path}] – sync_meta_sha po commicie
  const stats = { up: 0, down: 0, deleted: 0, meta: 0 };
  const readJson = async (sha) => JSON.parse(dec.decode(await store.blob(sha)));

  for (const k of kinds) {
    const local = await k.all();
    const localIds = new Set(local.map((r) => r.id));
    for (const rec of local) {
      const binPath = `${k.dir}/${rec.id}.bin`, jsonPath = `${k.dir}/${rec.id}.json`;
      const meta = metaOf(rec);
      const metaStr = canonical(meta);
      if (!files.has(binPath)) {
        uploads.push({ path: binPath, bytes: new Uint8Array(rec.raw) });
        stats.up++;
      }
      const rSha = files.get(jsonPath);
      if (!rSha) {
        uploads.push({ path: jsonPath, bytes: enc.encode(JSON.stringify(meta, null, 1)) });
        afterCommit.push({ rec, put: k.put, metaStr, path: jsonPath });
        continue;
      }
      if (rSha === rec.sync_meta_sha) {
        // zdalne bez zmian od ostatniej synchronizacji – wysyłamy tylko lokalną zmianę
        if (rec.sync_meta !== metaStr) {
          uploads.push({ path: jsonPath, bytes: enc.encode(JSON.stringify(meta, null, 1)) });
          afterCommit.push({ rec, put: k.put, metaStr, path: jsonPath });
          stats.meta++;
        }
        continue;
      }
      const remote = await readJson(rSha);
      if (remote.deleted) {
        await k.del(rec.id);
        stats.deleted++;
        continue;
      }
      // nigdy nie synchronizowany (np. z importu JSON) – rozstrzyga updated_at
      const localChanged = rec.sync_meta === undefined
        ? (rec.updated_at || 0) > (remote.updated_at || 0) : rec.sync_meta !== metaStr;
      if (!localChanged || (remote.updated_at || 0) >= (rec.updated_at || 0)) {
        const merged = { ...remote, raw: rec.raw };
        merged.sync_meta = canonical(metaOf(merged));
        merged.sync_meta_sha = rSha;
        if (merged.sync_meta !== metaStr) stats.meta++;
        await k.put(merged);
      } else {
        uploads.push({ path: jsonPath, bytes: enc.encode(JSON.stringify(meta, null, 1)) });
        afterCommit.push({ rec, put: k.put, metaStr, path: jsonPath });
        stats.meta++;
      }
    }
    // zdalne, których nie ma lokalnie
    const prefix = `${k.dir}/`;
    const remoteIds = [...files.keys()].filter((p) => p.startsWith(prefix) && p.endsWith('.json'))
      .map((p) => p.slice(prefix.length, -5));
    for (const id of remoteIds) {
      if (localIds.has(id)) continue;
      const jsonPath = `${k.dir}/${id}.json`;
      const remote = await readJson(files.get(jsonPath));
      if (remote.deleted) continue;
      if (isTomb(k.dir, id)) {
        const m = { ...remote, deleted: true, updated_at: Date.now() };
        uploads.push({ path: jsonPath, bytes: enc.encode(JSON.stringify(m, null, 1)) });
        continue;
      }
      const binSha = files.get(`${k.dir}/${id}.bin`);
      if (!binSha) continue;
      onProgress(`Pobieranie: ${k.name} ${id}`);
      const raw = await store.blob(binSha);
      try { k.parse(raw); } catch (e) { continue; }          // uszkodzony plik w repo – pomijamy
      const rec = { ...remote, raw: raw.buffer.slice(raw.byteOffset, raw.byteOffset + raw.byteLength) };
      rec.sync_meta = canonical(metaOf(rec));
      rec.sync_meta_sha = files.get(jsonPath);
      await k.put(rec);
      stats.down++;
    }
  }

  if (uploads.length) {
    onProgress(`Wysyłanie (${uploads.length} plików)…`);
    let shas;
    try {
      shas = await store.commit(head, treeSha, uploads,
        `Synchronizacja z ${device}: ${stats.up} nowych, ${stats.meta} zmian, ${new Date().toISOString().slice(0, 16)}`);
    } catch (e) {
      // ktoś zsynchronizował w międzyczasie (ref nie przesunął się „do przodu”) – od nowa
      if (e.status === 422 && retry > 0) return syncAll(store, db, { onProgress, device, retry: retry - 1 });
      throw e;
    }
    for (const a of afterCommit) {
      const cur = { ...a.rec, sync_meta: a.metaStr, sync_meta_sha: shas.get(a.path) };
      await a.put(cur);
    }
  }
  await db.kvSet('sync_deleted', []);
  await db.kvSet('sync_last', Date.now());
  return stats;
}
