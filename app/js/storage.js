// IndexedDB: surowe starty (oryginalna postać binarna), tagi, notatki, ustawienia.
// Wyniki analizy liczone przy wyświetlaniu (zmiana ustawień działa wstecz).

const DB_NAME = 'x4f-logger';
const DB_VER = 2;   // 2: magazyn 'races' (wyścigi)
let dbp = null;

function db() {
  if (dbp) return dbp;
  dbp = new Promise((resolve, reject) => {
    const req = indexedDB.open(DB_NAME, DB_VER);
    req.onupgradeneeded = () => {
      const d = req.result;
      if (!d.objectStoreNames.contains('runs')) {
        const s = d.createObjectStore('runs', { keyPath: 'id' });
        s.createIndex('tag', 'tag');
        s.createIndex('downloaded_at', 'downloaded_at');
      }
      if (!d.objectStoreNames.contains('kv')) d.createObjectStore('kv');
      if (!d.objectStoreNames.contains('races')) d.createObjectStore('races', { keyPath: 'id' });
    };
    req.onsuccess = () => resolve(req.result);
    req.onerror = () => reject(req.error);
  });
  return dbp;
}

function tx(store, mode, fn) {
  return db().then((d) => new Promise((resolve, reject) => {
    const t = d.transaction(store, mode);
    let result;
    const r = fn(t.objectStore(store));
    if (r && 'onsuccess' in r) r.onsuccess = () => { result = r.result; };
    // „complete” = dane trwale zapisane; dopiero wtedy wolno wysłać DEL
    t.oncomplete = () => resolve(result);
    t.onerror = () => reject(t.error);
    t.onabort = () => reject(t.error || new Error('Transakcja przerwana'));
  }));
}

export function runId(meta) {
  return `${meta.crc >>> 0}-${meta.seq}-${meta.boot}`;
}

/** Zapis startu; promise rozwiązywany dopiero po zakończeniu transakcji (oncomplete). */
export function putRun(rec) { return tx('runs', 'readwrite', (s) => s.put(rec)); }
export function getRun(id) { return tx('runs', 'readonly', (s) => s.get(id)); }
export function deleteRun(id) { return tx('runs', 'readwrite', (s) => s.delete(id)); }
export function allRuns() { return tx('runs', 'readonly', (s) => s.getAll()); }
export async function hasRun(id) { return !!(await getRun(id)); }

export async function updateRun(id, patch) {
  const r = await getRun(id);
  if (!r) return;
  Object.assign(r, patch, { updated_at: Date.now() });   // updated_at – konflikty synchronizacji
  await putRun(r);
}

// Wyścigi – osobny magazyn (inny format pliku; lista startów, porównanie i analiza startów ich nie widzą)
export function raceId(meta) {
  return `race-${meta.crc >>> 0}-${meta.seq}-${meta.boot}`;
}
export function putRace(rec) { return tx('races', 'readwrite', (s) => s.put(rec)); }
export function getRace(id) { return tx('races', 'readonly', (s) => s.get(id)); }
export function deleteRace(id) { return tx('races', 'readwrite', (s) => s.delete(id)); }
export function allRaces() { return tx('races', 'readonly', (s) => s.getAll()); }
export async function hasRace(id) { return !!(await getRace(id)); }
export async function updateRace(id, patch) {
  const r = await getRace(id);
  if (!r) return;
  Object.assign(r, patch, { updated_at: Date.now() });
  await putRace(r);
}

export function kvGet(key) { return tx('kv', 'readonly', (s) => s.get(key)); }
export function kvSet(key, val) { return tx('kv', 'readwrite', (s) => s.put(val, key)); }

export async function recentTags() { return (await kvGet('recent_tags')) || []; }
export async function pushRecentTag(tag) {
  if (!tag) return;
  const t = (await recentTags()).filter((x) => x !== tag);
  t.unshift(tag);
  await kvSet('recent_tags', t.slice(0, 8));
}

/** Usunięcie lokalne zapamiętane dla synchronizacji (inne urządzenia też usuną). */
export async function addTombstone(dir, id) {
  const t = (await kvGet('sync_deleted')) || [];
  t.push({ dir, id });
  await kvSet('sync_deleted', t);
}

export async function clearAll() {
  await tx('runs', 'readwrite', (s) => s.clear());
}
