// TapTune offline support: after the first visit the page and piano samples
// load from the phone's cache, so it works with no internet at the shoot.
const CACHE = "taptune-v10";
const NAMES = ["A0"];
for (let o = 1; o <= 7; o++) NAMES.push(`C${o}`, `Ds${o}`, `Fs${o}`, `A${o}`);
NAMES.push("C8");
const FILES = ["./", "index.html", "manifest.webmanifest", "icon.svg", ...NAMES.map(n => `samples/${n}.mp3`)];

self.addEventListener("install", e => {
  e.waitUntil(caches.open(CACHE).then(c => c.addAll(FILES)).then(() => self.skipWaiting()));
});

self.addEventListener("activate", e => {
  e.waitUntil(caches.keys()
    .then(keys => Promise.all(keys.filter(k => k !== CACHE).map(k => caches.delete(k))))
    .then(() => self.clients.claim()));
});

// Page: network first (so updates show up), cache when offline.
// Samples and icons: cache first (they never change).
self.addEventListener("fetch", e => {
  const req = e.request;
  if (req.method !== "GET") return;
  const isPage = req.mode === "navigate" || req.url.endsWith("index.html");
  if (isPage) {
    e.respondWith(fetch(req).then(res => {
      const copy = res.clone();
      caches.open(CACHE).then(c => c.put(req, copy));
      return res;
    }).catch(() => caches.match(req).then(r => r || caches.match("index.html"))));
  } else {
    e.respondWith(caches.match(req).then(r => r || fetch(req)));
  }
});
