/**
 * sw.js — Service Worker for ESP32 DAC PWA
 * Author: Danny Liu  |  v3.0.0
 *
 * Caches all PWA assets on install so the app loads fully
 * offline after the first visit.
 *
 * Cache version: bump CACHE_NAME whenever you deploy a new
 * version of index.html so users get the latest app.
 */

const CACHE_NAME = 'esp32-dac-v3';

const PRECACHE_ASSETS = [
  './',
  './index.html',
  './manifest.json',
  './icons/icon-192.png',
  './icons/icon-512.png',
  './icons/icon-maskable-512.png',
  './icons/apple-touch-icon.png',
  './icons/favicon-32.png'
];

// ── Install: pre-cache all assets ────────────────────────────
self.addEventListener('install', event => {
  event.waitUntil(
    caches.open(CACHE_NAME)
      .then(cache => cache.addAll(PRECACHE_ASSETS))
      .then(() => self.skipWaiting())   // activate immediately
  );
});

// ── Activate: delete old caches ──────────────────────────────
self.addEventListener('activate', event => {
  event.waitUntil(
    caches.keys()
      .then(keys => Promise.all(
        keys
          .filter(key => key !== CACHE_NAME)
          .map(key => caches.delete(key))
      ))
      .then(() => self.clients.claim())  // take control immediately
  );
});

// ── Fetch: cache-first for local assets, network for external ─
self.addEventListener('fetch', event => {
  const url = new URL(event.request.url);

  // Always go to network for external APIs (NAS, BLE, fonts)
  if (
    url.origin !== self.location.origin ||
    url.pathname.includes('/rest/') ||       // Subsonic/Navidrome API
    url.pathname.includes('/Items') ||        // Jellyfin API
    url.pathname.includes('/Audio/') ||       // Jellyfin audio stream
    url.hostname.includes('googleapis') ||
    url.hostname.includes('gstatic')
  ) {
    return;  // fall through to network — no caching for streams/API
  }

  // Cache-first for all local app assets
  event.respondWith(
    caches.match(event.request)
      .then(cached => {
        if (cached) return cached;
        return fetch(event.request)
          .then(response => {
            // Cache valid GET responses
            if (
              response &&
              response.status === 200 &&
              event.request.method === 'GET'
            ) {
              const clone = response.clone();
              caches.open(CACHE_NAME).then(cache => cache.put(event.request, clone));
            }
            return response;
          })
          .catch(() => caches.match('./index.html'));  // offline fallback
      })
  );
});
