/* Moonshot Avionics — service worker.
 *
 * Why this exists: Web Bluetooth, the Web Speech API and clipboard access all
 * require a secure context. Out at a launch site there is no internet, and the
 * copy of this dashboard served from the base station's WiFi AP is plain HTTP,
 * so those APIs are unavailable there. Installing the GitHub Pages copy as a
 * PWA keeps the https:// origin — and everything it unlocks — working with no
 * network at all.
 *
 * Strategy
 *   precache            everything the dashboard needs to boot, on install
 *   same-origin GETs    stale-while-revalidate — serve from cache instantly,
 *                       refresh in the background, tell the page if the copy
 *                       on the server changed so it can offer a reload.
 *                       Navigations resolve to the cached index.html under the
 *                       same rule. Never network-first: when the phone is
 *                       joined to the base station AP there IS a network, it
 *                       just has no route to GitHub, so a network-first fetch
 *                       would stall on every launch.
 *   everything else     passthrough, untouched. In particular the base
 *                       station's http://192.168.4.1 API and its ws:// socket
 *                       are cross-origin and must never be cached or proxied.
 *
 * Because of stale-while-revalidate you do NOT normally need to bump
 * CACHE_VERSION when you edit app2.js or style.css: deploy, load the page once
 * (with internet), and the page offers to reload into the new build. Bump it
 * only to force-evict every client, e.g. after removing a precached file.
 */

// bucketshoes.github.io also hosts EspRangeTest, with its own worker. Caches
// are origin-wide, so this app namespaces its own and only ever deletes caches
// carrying this prefix.
var CACHE_PREFIX  = 'moonshot-';
var CACHE_VERSION = 'v2';   // bumped 2026-09-25 to force every client to re-precache
var CACHE_NAME    = CACHE_PREFIX + CACHE_VERSION;

// Everything required to boot with zero network. Relative to this file, so the
// whole app can move to a different path (or a different domain) untouched.
var PRECACHE = [
  'index.html',
  'style.css',
  'app2.js',
  'ble_adapter.js',
  'manifest.webmanifest',
  'lib/chart.umd.min.js',
  'lib/hammer.min.js',
  'lib/chartjs-plugin-zoom.min.js',
  'icons/icon-192.png',
  'icons/icon-512.png',
  'icons/icon-192-maskable.png',
  'icons/icon-512-maskable.png',
  'icons/apple-touch-icon.png',
  'icons/favicon-32.png'
];

// ---------------------------------------------------------------- install --

self.addEventListener('install', function(ev) {
  ev.waitUntil(
    caches.open(CACHE_NAME).then(function(cache) {
      // cache:'reload' bypasses the HTTP cache so an install always pulls the
      // build that is actually on the server, not GitHub Pages' 10-minute copy.
      return Promise.all(PRECACHE.map(function(url) {
        return fetch(new Request(url, { cache: 'reload' })).then(function(res) {
          if (!res.ok) throw new Error(res.status + ' ' + url);
          return cache.put(url, res);
        }).catch(function(err) {
          // One missing optional asset must not abort the whole install and
          // leave the user with no offline copy at all.
          console.warn('[sw] precache failed:', url, err.message);
        });
      }));
    })
  );
});

// --------------------------------------------------------------- activate --

self.addEventListener('activate', function(ev) {
  ev.waitUntil(
    caches.keys().then(function(names) {
      return Promise.all(names.map(function(n) {
        if (n !== CACHE_NAME && n.indexOf(CACHE_PREFIX) === 0) return caches.delete(n);
      }));
    }).then(function() {
      return self.clients.claim();
    })
  );
});

// URLs whose bytes changed on the server since the current page was served.
// Kept here as well as pushed to clients because a background revalidate often
// finishes before the page has attached its message listener — the page asks
// for the backlog with 'check-updates' once it is ready. Cleared on every
// navigation, since by then the page is loading the refreshed files.
var pendingUpdates = [];

// Set by a 'purge' message. Once purging, this worker must not write to the
// cache again: the page is about to delete it, and an in-flight revalidate
// landing its cache.put() afterwards would silently recreate it.
var purging = false;

// The files worth explicitly re-checking on every launch. A reload does not
// reliably route subresources through this worker — Chrome can satisfy them
// from the renderer's memory cache — so relying on passive revalidation alone
// would miss deploys. These four are the whole dashboard; the icons and the
// chart libs change about never.
var WATCH = ['index.html', 'style.css', 'app2.js', 'ble_adapter.js'];

self.addEventListener('message', function(ev) {
  // Sent after the user accepts the update prompt.
  if (ev.data === 'skip-waiting') { self.skipWaiting(); return; }

  // Sent by the ?nosw recovery hatch. The worker deletes its own caches,
  // because only it knows when its own writes have stopped. Scoped to this
  // app's prefix — the other app on this origin keeps its offline copy.
  if (ev.data === 'purge') {
    purging = true;
    ev.waitUntil(caches.keys().then(function(names) {
      return Promise.all(names.filter(function(n) {
        return n.indexOf(CACHE_PREFIX) === 0;
      }).map(function(n) { return caches.delete(n); }));
    }).then(function() {
      if (ev.source) ev.source.postMessage({ type: 'purged' });
    }));
    return;
  }

  if (ev.data === 'check-updates') {
    // Anything a background revalidate already spotted before the page was
    // listening.
    if (pendingUpdates.length && ev.source) {
      ev.source.postMessage({ type: 'asset-updated', url: pendingUpdates[0] });
      return;
    }
    // Otherwise go and look. Conditional requests, so this is a few hundred
    // bytes of 304s when nothing changed, and nothing at all when offline.
    ev.waitUntil(caches.open(CACHE_NAME).then(function(cache) {
      return Promise.all(WATCH.map(function(name) {
        var url = new URL(name, self.location.href).href;
        return cache.match(url).then(function(cached) {
          if (!cached) return null;
          return revalidate(cache, url, cached);
        });
      }));
    }));
  }
});

// ------------------------------------------------------------------ fetch --

function notifyClients(msg) {
  if (msg.type === 'asset-updated' && pendingUpdates.indexOf(msg.url) < 0) {
    pendingUpdates.push(msg.url);
  }
  self.clients.matchAll({ type: 'window' }).then(function(cs) {
    cs.forEach(function(c) { c.postMessage(msg); });
  });
}

// Cheap change detection: GitHub Pages sends a strong ETag, and the ESP32
// sends neither, in which case we fall back to Last-Modified and then to
// "assume unchanged" rather than nagging on every load.
function stamp(res) {
  if (!res) return null;
  return res.headers.get('ETag') || res.headers.get('Last-Modified') || null;
}

// Re-fetch one URL, update the cache, and tell open pages if the bytes on the
// server actually changed. Resolves to null when offline.
function revalidate(cache, url, cached) {
  // no-cache forces a conditional request, so an edit deployed minutes ago is
  // not hidden behind GitHub Pages' max-age.
  if (purging) return Promise.resolve(null);
  return fetch(new Request(url, { cache: 'no-cache' })).then(function(res) {
    if (!res || !res.ok || purging) return null;
    var before = stamp(cached), after = stamp(res);
    return cache.put(url, res.clone()).then(function() {
      if (cached && before && after && before !== after) {
        notifyClients({ type: 'asset-updated', url: url });
      }
      return res;
    });
  }).catch(function() {
    return null;   // offline: the cached copy is the answer
  });
}

// Stale-while-revalidate for one URL.
//
// ev.waitUntil() has to be called synchronously from the fetch handler — do it
// from inside a .then() and the event may already have finished, which throws
// and silently kills the background refresh. So the revalidate promise is
// built here and handed to waitUntil straight away.
function staleWhileRevalidate(ev, url) {
  var cacheP = caches.open(CACHE_NAME);
  // ignoreSearch so a cache-busted URL still hits its precached entry. Older
  // copies of the page (and any tab still running one) request
  // `app.js?_=<timestamp>`, a URL that can never be in the cache (and the old
  // filename too, since app.js was renamed to app2.js); without this
  // they get a 504 offline, no script and no stylesheet load, and the page
  // renders as dead unstyled HTML that looks like a broken app rather than a
  // stale one.
  var cachedP = cacheP.then(function(cache) {
    return cache.match(url, { ignoreSearch: true });
  });
  var freshP = Promise.all([cacheP, cachedP]).then(function(r) {
    return revalidate(r[0], url, r[1]);
  });

  ev.waitUntil(freshP);

  ev.respondWith(cachedP.then(function(cached) {
    if (cached) return cached;                    // instant, works offline
    return freshP.then(function(res) {
      return res || new Response('offline and not cached', {
        status: 504, statusText: 'Offline',
        headers: { 'Content-Type': 'text/plain' }
      });
    });
  }));
}

self.addEventListener('fetch', function(ev) {
  var req = ev.request;
  if (req.method !== 'GET') return;

  // Mid-purge the cache is being torn down, so stop intercepting and let
  // requests go straight to the network rather than answering 504 from a
  // cache that no longer exists.
  if (purging) return;

  var url;
  try { url = new URL(req.url); } catch (e) { return; }

  // Leave the base station (and anything else off-origin) completely alone.
  if (url.origin !== self.location.origin) return;

  // Only handle files under this app's own directory.
  var scope = new URL('./', self.location.href).pathname;
  if (url.pathname.indexOf(scope) !== 0) return;

  // Navigations always render the cached shell, then check for a new one in
  // the background. index.html is normalised to a single cache key so that
  // "/", "/index.html" and the PWA start_url all share one entry.
  if (req.mode === 'navigate') {
    pendingUpdates.length = 0;
    staleWhileRevalidate(ev, new URL('index.html', self.location.href).href);
    return;
  }

  staleWhileRevalidate(ev, req.url);
});
