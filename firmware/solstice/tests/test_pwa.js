// Checks the install/offline pieces: manifest, icons, and service-worker behaviour (with a fake Cache API).
'use strict';
const fs = require('fs'), path = require('path'), vm = require('vm');
const web = path.join(__dirname, '../web');
let fails = 0;
const check = (n, ok, d) => { console.log(`[${ok ? 'PASS' : 'FAIL'}] ${n}${d ? '  ' + d : ''}`); if (!ok) fails++; };

const m = JSON.parse(fs.readFileSync(path.join(web, 'manifest.webmanifest'), 'utf8'));
const html = fs.readFileSync(path.join(web, 'index.html'), 'utf8');
check('app is named Solstice (title, header, manifest)', /<title>Solstice<\/title>/.test(html) && /<h1>Solstice<\/h1>/.test(html) && m.name === 'Solstice' && m.short_name === 'Solstice');
check('connect filters on the service UUID, not the board name, so a renamed board is still found', /requestDevice\(\{ filters: \[\{ services: \[Proto\.UUID\.service\] \}\] \}\)/.test(html) && !/namePrefix|BallisticCalc/.test(html));
check('manifest is standalone with a start_url', m.display === 'standalone' && !!m.start_url);
for (const ic of m.icons) {
  const buf = fs.readFileSync(path.join(web, ic.src));
  const w = buf.readUInt32BE(16), h = buf.readUInt32BE(20);
  check(`icon ${ic.src} is a real PNG of ${ic.sizes}`, buf.slice(1, 4).toString() === 'PNG' && `${w}x${h}` === ic.sizes, `${w}x${h}`);
}

// Fake service worker environment
const reloaded = [];
class Request { constructor(url, init) { this.url = url; this.cache = init && init.cache; } }
const store = {}; const handlers = {}; let skipped = false, claimed = false;
const fakeCaches = {
  async open(name) { store[name] = store[name] || {}; return { async addAll(reqs) { reqs.forEach(r => { const u = r.url || r; store[name][u] = 'cached:' + u; reloaded.push(r.cache); }); } }; },
  async keys() { return Object.keys(store); },
  async delete(k) { delete store[k]; return true; },
  async match(req) { for (const c of Object.values(store)) { const k = Object.keys(c).find(u => u !== './' && req.url.endsWith(u.replace('./', ''))); if (k && k !== './') return c[k]; } },
};
const sw = { addEventListener: (t, f) => handlers[t] = f, skipWaiting: () => { skipped = true; }, clients: { claim: async () => { claimed = true; } } };
const ctx = { self: sw, caches: fakeCaches, fetch: async () => 'network', Request, Promise, Object, console };
vm.createContext(ctx);
const swSrc = fs.readFileSync(path.join(web, 'sw.js'), 'utf8');
const CACHE_NAME = (swSrc.match(/const CACHE = '([^']+)'/) || [])[1];
vm.runInContext(swSrc, ctx);

(async () => {
  store['ballistics-old'] = { x: 1 };                       // stale cache from an earlier version
  let p; handlers.install({ waitUntil: (x) => p = x }); await p;
  const listed = [...swSrc.matchAll(/'(\.\/[^']*)'/g)].map(x => x[1]).filter(x => x !== './');
  check('install caches every listed file', listed.every(f => store[CACHE_NAME][f]), listed.join(' '));
  check('install fetches past the browser HTTP cache (cache: reload) so a new version never stores the old file', reloaded.length >= 6 && reloaded.every(c => c === 'reload'), reloaded.join(','));
  check('every cached file exists on disk', listed.every(f => fs.existsSync(path.join(web, f))));
  handlers.activate({ waitUntil: (x) => p = x }); await p;
  check('activate deletes old caches', !store['ballistics-old'] && !!store[CACHE_NAME]);
  let resp; handlers.fetch({ request: { method: 'GET', url: 'https://x.example/index.html' }, respondWith: (x) => resp = x });
  check('offline request served from cache', (await resp) === 'cached:./index.html', await resp);
  handlers.fetch({ request: { method: 'GET', url: 'https://x.example/other.json' }, respondWith: (x) => resp = x });
  check('uncached request falls through to network', (await resp) === 'network');
  console.log(fails ? `\nFAILED (${fails})` : '\nALL PASSED');
  process.exit(fails ? 1 : 0);
})();
