// Dev server for the simulator:  npm install && npm run dev   ->  http://localhost:5173/web/dev.html
// The page is web/src/index.template.html with the JS parts (web/src/js/*.js, ONE script cut into files) concatenated on every request, so an edit + reload is enough;
// the robot data (window.PREMAID = MJCF + meshes) is read from the generated web/premaid-sim.html (run `npm run build` once, and again after changing the model).
// Production output stays the single file web/premaid-sim.html produced by `npm run build` (python mujoco/make_premaid_mjcf.py).
import { defineConfig } from 'vite';
import fs from 'node:fs';
import path from 'node:path';

const root = path.dirname(new URL(import.meta.url).pathname);
const src = path.join(root, 'web', 'src');

function parts() {
  return fs.readdirSync(path.join(src, 'js')).filter((f) => f.endsWith('.js')).sort().map((f) => fs.readFileSync(path.join(src, 'js', f), 'utf8')).join('');
}

function assets() {
  const html = fs.readFileSync(path.join(root, 'web', 'premaid-sim.html'), 'utf8');
  const i = html.indexOf('window.PREMAID=');
  if (i < 0) throw new Error('web/premaid-sim.html has no window.PREMAID: run `npm run build` first');
  return html.slice(i, html.indexOf(';</script>', i) + 1);
}

function premaidDev() {
  return {
    name: 'premaid-dev',
    configureServer(server) {
      server.middlewares.use((req, res, next) => {
        const url = (req.url || '').split('?')[0];
        if (url === '/web/dev.html' || url === '/web/') {
          let html = fs.readFileSync(path.join(src, 'index.template.html'), 'utf8');
          html = html.replace(/<script>\s*\/\*__ASSETS__\*\/\s*<\/script>/, '<script src="/web/dev/assets.js"></script>').replace('<!--@MAIN_SCRIPT-->', '</script><script src="/web/dev/main.js"></script><script>');
          res.setHeader('Content-Type', 'text/html'); res.end(html);
        } else if (url === '/web/dev/main.js') {
          res.setHeader('Content-Type', 'text/javascript'); res.setHeader('Cache-Control', 'no-store'); res.end(parts());
        } else if (url === '/web/dev/assets.js') {
          res.setHeader('Content-Type', 'text/javascript'); res.setHeader('Cache-Control', 'no-store'); res.end(assets());
        } else next();
      });
    },
  };
}

export default defineConfig({ root, plugins: [premaidDev()], server: { port: 5173, fs: { allow: [root] } }, appType: 'mpa' });
