// Offline cache so the app opens with no signal after the first visit.
// Bump CACHE when you change any file, or phones will keep serving the old copy.
const CACHE = 'ballistics-v1';
const FILES = ['./', './index.html', './protocol.js', './manifest.webmanifest', './icon-192.png', './icon-512.png'];

self.addEventListener('install', (e) => {
  e.waitUntil(caches.open(CACHE).then((c) => c.addAll(FILES)).then(() => self.skipWaiting()));
});
self.addEventListener('activate', (e) => {
  e.waitUntil(caches.keys()
    .then((keys) => Promise.all(keys.filter((k) => k !== CACHE).map((k) => caches.delete(k))))
    .then(() => self.clients.claim()));
});
self.addEventListener('fetch', (e) => {
  if (e.request.method !== 'GET') return;
  // Cache first, fall back to network. Offline-safe for the files above.
  e.respondWith(caches.match(e.request).then((hit) => hit || fetch(e.request)));
});
