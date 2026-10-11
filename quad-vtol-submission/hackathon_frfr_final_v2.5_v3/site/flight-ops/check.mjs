import { readFileSync, existsSync } from 'node:fs';
import { resolve, dirname } from 'node:path';

const root = resolve(import.meta.dirname, 'dist');
const html = readFileSync(resolve(root, 'index.html'), 'utf8');
const importMap = JSON.parse(html.match(/<script type="importmap">(.*?)<\/script>/s)[1]).imports;
const seen = new Set();

function checkModule(path) {
  if (seen.has(path)) return;
  seen.add(path);
  const source = readFileSync(path, 'utf8').replace(/\/\*[\s\S]*?\*\/|^\s*\/\/.*$/gm, '');
  for (const match of source.matchAll(/^\s*(?:\}|import[^\n]*|export[^\n]*)\s+from\s*['"]([^'"]+)['"]/gm)) {
    const alias = importMap[match[1]];
    const specifier = alias || match[1];
    if (!specifier.startsWith('.')) throw new Error(`Unresolved import ${specifier} in ${path}`);
    checkModule(resolve(alias ? root : dirname(path), specifier));
  }
}

for (const asset of ['flightpath.css','dashboard.css','data/columbia-routes.json','favicon.svg','data/archive.json','data/test-evidence.json',
                     'assets/quad_delivery_vtol.glb','vendor/three.core.js']) {
  if (!existsSync(resolve(root, asset))) throw new Error(`Missing ${asset}`);
}
checkModule(resolve(root, 'app.js'));
const glb = readFileSync(resolve(root, 'assets/quad_delivery_vtol.glb'));
if (glb.toString('ascii', 0, 4) !== 'glTF' || glb.readUInt32LE(4) !== 2)
  throw new Error('Invalid STEP-derived GLB');
const archive = JSON.parse(readFileSync(resolve(root, 'data/archive.json')));
if (!archive.frames?.length) throw new Error('Recorded replay is empty');
for (const id of ['battery','altitude','groundspeed','roll','pitch','heading'])
  if (!html.includes(`id="graphs"`) || !readFileSync(resolve(root, 'telemetry.js'),'utf8').includes(`id:'${id}'`))
    throw new Error(`Missing graph ${id}`);
for (const removed of ['position-form','scenario-grid','local-map-wrap'])
  if (html.includes(removed)) throw new Error(`Removed control remains: ${removed}`);
console.log(`Site assets valid: ${seen.size} modules, ${archive.frames.length} replay frames`);
