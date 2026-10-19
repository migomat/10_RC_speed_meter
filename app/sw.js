// Service worker – praca offline. Wszystkie zasoby lokalne (bez CDN).
// Zmiana dowolnego pliku aplikacji = podbicie CACHE.
const CACHE = 'x4f-v1.0.3';
const ASSETS = [
  './', './index.html', './manifest.webmanifest', './css/app.css',
  './vendor/uPlot.iife.min.js', './vendor/uPlot.min.css',
  './js/ui.js', './js/transport.js', './js/protocol.js', './js/format.js', './js/analysis.js',
  './js/estimator.js', './js/storage.js', './js/charts.js',
  './icons/icon-192.png', './icons/icon-512.png', './icons/icon-maskable-512.png',
  './demo/index.json', './demo/mode1_grip.bin', './demo/mode1_slip.bin', './demo/mode3_grip.bin',
  './demo/mode3_slip.bin', './demo/mode1_slip_noedges.bin', './demo/mode1_lifted.bin',
];

self.addEventListener('install', (e) => {
  e.waitUntil(caches.open(CACHE).then((c) => c.addAll(ASSETS)).then(() => self.skipWaiting()));
});

self.addEventListener('activate', (e) => {
  e.waitUntil(caches.keys()
    .then((keys) => Promise.all(keys.filter((k) => k !== CACHE).map((k) => caches.delete(k))))
    .then(() => self.clients.claim()));
});

// Najpierw pamięć podręczna (działanie na torze bez zasięgu), w tle odświeżenie.
self.addEventListener('fetch', (e) => {
  if (e.request.method !== 'GET' || new URL(e.request.url).origin !== location.origin) return;
  e.respondWith(caches.match(e.request, { ignoreSearch: true }).then((hit) => {
    const net = fetch(e.request).then((res) => {
      if (res.ok) {
        const copy = res.clone();
        caches.open(CACHE).then((c) => c.put(e.request, copy));
      }
      return res;
    }).catch(() => hit);
    return hit || net;
  }));
});
