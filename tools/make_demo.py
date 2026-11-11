#!/usr/bin/env python3
"""Render docs/img/demo.gif and screenshots from the web simulator (Chromium + Pillow): plays a library motion deterministically (fixed time steps, no real-time rendering).
    pip install playwright pillow && playwright install chromium && python tools/make_demo.py"""
import base64, io, os, subprocess, sys, time
from PIL import Image
from playwright.sync_api import sync_playwright
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); OUT = os.path.join(ROOT, 'docs', 'img'); PORT = 8799
srv = subprocess.Popen([sys.executable, '-m', 'http.server', str(PORT), '--bind', '127.0.0.1'], cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL); time.sleep(1)
try:
    with sync_playwright() as pw:
        b = pw.chromium.launch()
        # ---- demo GIF: kiosk page, 480 x 360, 10 fps
        pg = b.new_page(viewport={'width': 480, 'height': 360}); pg.add_init_script('window.requestAnimationFrame = f => setTimeout(() => f(performance.now()), 5000)')
        pg.goto('http://127.0.0.1:%d/web/premaid-sim.html#kiosk=1&embed=1&bar=0&play=0&m=Eternal%%20Blue%%EF%%BD%%9E%%E3%%82%%AD%%E3%%83%%9F%%E3%%81%%A8%%E5%%83%%95%%E3%%81%%A8%%E7%%A9%%BA%%E3%%81%%A8%%E6%%B5%%B7%%E3%%81%%AE%%E7%%89%%A9%%E8%%AA%%9E%%EF%%BD%%9E' % PORT)
        pg.wait_for_function('!!window.PMSIM && window.PMSIM.pma.P.frames', timeout=90000); ev = pg.evaluate; time.sleep(1)
        ev("(() => { const P = window.PMSIM.pma.P; P.playing = false; P.loop = false; })()"); frames = []; t0, fps, secs = 3000, 10, 6
        for i in range(int(secs * fps)):
            png = ev("(t) => { const P = window.PMSIM.pma.P; P.t = t; window.PMSIM.pma.poseAt(t); window.PMSIM.advance(0.1); return window.PMSIM.render(); }", t0 + i * 1000 / fps)
            frames.append(Image.open(io.BytesIO(base64.b64decode(png.split(',')[1]))).convert('RGB'))
        pal = frames[0].quantize(colors=128, method=Image.Quantize.MEDIANCUT)
        q = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]
        q[0].save(os.path.join(OUT, 'demo.gif'), save_all=True, append_images=q[1:], duration=int(1000 / fps), loop=0, optimize=True, disposal=1)
        print('demo.gif', os.path.getsize(os.path.join(OUT, 'demo.gif')) // 1024, 'KB', len(frames), 'frames'); pg.close()
        # ---- screenshots of the full UI
        pg = b.new_page(viewport={'width': 1366, 'height': 768}); pg.goto('http://127.0.0.1:%d/web/premaid-sim.html' % PORT); pg.wait_for_function('!!window.PMSIM', timeout=90000); time.sleep(1.5)
        pg.evaluate("document.querySelectorAll('#pList button')[1].click()"); time.sleep(3); pg.screenshot(path=os.path.join(OUT, 'sim-motion.png'))
        pg.click('[data-tab=rec]'); time.sleep(1); pg.screenshot(path=os.path.join(OUT, 'sim-editor.png'))
        pg.evaluate("document.getElementById('cSen').click(); document.getElementById('cTrail').click()"); pg.click('[data-tab=robot]'); time.sleep(2.5); pg.screenshot(path=os.path.join(OUT, 'sim-robot-hud.png')); b.close()
finally:
    srv.terminate()
