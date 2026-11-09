/* Offline cache for the PreMaid AI simulator. Same-origin GET only: answers from the cache at once and refreshes it in the background
   (stale-while-revalidate). Bump CACHE when the precache list changes. */
var CACHE = 'premaid-sim-v1';
var CORE = ['./', 'premaid-sim.html', 'manifest.webmanifest', 'icon.svg', 'icon-192.png', 'icon-512.png',
  'vendor/three.min.js', 'vendor/OrbitControls.js', 'vendor/mujoco/mujoco.js', 'vendor/mujoco/mujoco.wasm', '../pma/library.js'];
self.addEventListener('install', function (e) {
  e.waitUntil(caches.open(CACHE).then(function (c) { return Promise.all(CORE.map(function (u) { return c.add(u).catch(function () {}); })); }).then(function () { return self.skipWaiting(); }));
});
self.addEventListener('activate', function (e) {
  e.waitUntil(caches.keys().then(function (ks) { return Promise.all(ks.filter(function (k) { return k !== CACHE; }).map(function (k) { return caches.delete(k); })); }).then(function () { return self.clients.claim(); }));
});
self.addEventListener('fetch', function (e) {
  var r = e.request, u = new URL(r.url);
  if (r.method !== 'GET' || u.origin !== location.origin) return;
  e.respondWith(caches.match(r, { ignoreSearch: true }).then(function (hit) {
    var net = fetch(r).then(function (res) { if (res && res.ok && res.type === 'basic') { var copy = res.clone(); caches.open(CACHE).then(function (c) { c.put(r, copy); }); } return res; }).catch(function () { return hit; });
    return hit || net;
  }));
});
