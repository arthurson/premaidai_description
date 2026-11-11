"""Landing page (index.html): language switch, motion links, a deep link opens the simulator with that motion.   python tests/e2e_landing.py"""
import os, subprocess, sys, time
from playwright.sync_api import sync_playwright
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); PORT = 8800; fails = []
def check(n, ok, i=''):
    print(('PASS ' if ok else 'FAIL ') + n + (' ' + str(i) if i != '' else '')); ok or fails.append(n)
srv = subprocess.Popen([sys.executable, '-m', 'http.server', str(PORT), '--bind', '127.0.0.1'], cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL); time.sleep(1)
try:
    with sync_playwright() as pw:
        b = pw.chromium.launch(); pg = b.new_page(viewport={'width': 1200, 'height': 800}); errs = []
        pg.on('console', lambda m: errs.append(m.text) if m.type == 'error' else None); pg.on('pageerror', lambda e: errs.append(str(e)))
        pg.goto('http://127.0.0.1:%d/index.html' % PORT); pg.wait_for_function("document.querySelectorAll('#motions a').length >= 10", timeout=10000)
        check('10 bundled motions listed', pg.evaluate("document.querySelectorAll('#motions a').length") == 10); check('6 feature cards', pg.evaluate("document.querySelectorAll('#features .card').length") == 6)
        pg.click('[data-l=ja]'); check('Japanese UI', 'シミュレータを開く' in pg.inner_text('.btn.primary') and pg.evaluate("document.documentElement.lang") == 'ja')
        pg.click('[data-l=en]'); check('English UI', 'Open the simulator' in pg.inner_text('.btn.primary'))
        check('demo gif and screenshots load', pg.evaluate("Array.from(document.images).every(i => i.complete && i.naturalWidth > 100)") or pg.wait_for_function("Array.from(document.images).every(i => i.complete && i.naturalWidth > 100)", timeout=10000) is None)
        href = pg.evaluate("document.querySelectorAll('#motions a')[1].getAttribute('href')"); title = pg.evaluate("document.querySelectorAll('#motions a')[1].firstChild.textContent")
        pg.add_init_script('window.requestAnimationFrame = f => setTimeout(() => f(performance.now()), 1500)')
        pg.click('#motions a:nth-child(2)'); pg.wait_for_function('!!window.PMSIM && window.PMSIM.pma.P.frames', timeout=90000); time.sleep(1)
        check('deep link opens the simulator with that motion', pg.evaluate("document.querySelector('#pList button.on') && document.querySelector('#pList button.on').textContent") == title, (title, href[:40]))
        check('no console errors', not [e for e in errs if 'Failed to load resource' not in e], errs[:2]); b.close()
finally:
    srv.terminate()
print('\n%d failed' % len(fails) if fails else '\nall passed'); sys.exit(1 if fails else 0)
