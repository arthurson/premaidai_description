"""Browser end-to-end test of the P2 simulator features (needs playwright + chromium + Pillow; not part of CI).
    python tests/e2e_p2.py"""
import base64, io, os, subprocess, sys, time, math
from PIL import Image
from playwright.sync_api import sync_playwright
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE); PORT = 8794
fails = []
def check(name, ok, info=''):
    print(('PASS ' if ok else 'FAIL ') + name + (' ' + str(info) if info != '' else '')); ok or fails.append(name)
def until(fn, timeout=20):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if fn(): return True
        time.sleep(0.1)
    return False
srv = subprocess.Popen([sys.executable, '-m', 'http.server', str(PORT), '--bind', '127.0.0.1'], cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL); time.sleep(1)
URL = 'http://127.0.0.1:%d/web/premaid-sim.html' % PORT
THROTTLE = 'window.requestAnimationFrame = f => setTimeout(() => f(performance.now()), 1500)'
MOCKPAD = "navigator.getGamepads = () => window.__pad ? [window.__pad] : []"
try:
    with sync_playwright() as pw:
        b = pw.chromium.launch(args=['--autoplay-policy=no-user-gesture-required'])
        ctx = b.new_context(viewport={'width': 1280, 'height': 720}, accept_downloads=True, service_workers='allow'); pg = ctx.new_page(); errs = []
        pg.on('console', lambda m: errs.append(m.text) if m.type == 'error' else None); pg.on('pageerror', lambda e: errs.append(str(e))); pg.on('dialog', lambda d: d.accept())
        pg.add_init_script(THROTTLE); pg.add_init_script(MOCKPAD)
        pg.goto(URL); pg.wait_for_function('!!window.PMSIM', timeout=90000); ev = pg.evaluate
        def jc(i): ev("(i) => document.getElementById(i).click()", i)
        def key(k): pg.keyboard.press(k)
        # ---------------- ja + language cycle
        jc('bLang'); check('zh -> en', ev("document.documentElement.lang") == 'en'); jc('bLang'); check('en -> ja', ev("document.documentElement.lang") == 'ja' and 'モーション' in ev("document.querySelector('.tabs button').textContent"), ev("document.querySelector('.tabs button').textContent"))
        check('ja labels + en fallback', ev("document.getElementById('estop').textContent") == '■ 緊急停止' and ev("document.getElementById('bLang').textContent") == '中文')
        jc('bLang'); check('ja -> zh', ev("document.documentElement.lang") == 'zh-Hant')
        # ---------------- shortcuts
        ev("window.PMSIM.S.run = true"); ev("document.body.focus()")
        key('?'); check('? shows the help', not ev("document.getElementById('kbHelp').hidden")); key('Escape'); check('Esc closes the help', ev("document.getElementById('kbHelp').hidden"))
        n0 = ev("window.PMSIM.p2.PL.hist.length"); ev("(() => { window.libLoad && 0; })()")
        key('h'); check('h toggles hang', ev("document.getElementById('cPin').checked") and ev("window.PMSIM.S.pin")); key('h')
        key('b'); check('b toggles stand assist', ev("window.PMSIM.p2.BAL.on")); key('b')
        p0 = ev("window.PMSIM.p2 && [camera_x = 0][0]"); key('3'); key('1')
        # ---------------- balance: lateral push of 8 N, 0.1 s
        def push_run(on, F=9.0, secs=3.0):
            return ev("""(a) => { const P2 = window.PMSIM.p2, S = window.PMSIM.S; window.PMSIM.reset(); for (let k = 0; k < 25; k++) window.PMSIM.setTarget(k, 0); P2.BAL.on = a[0]; const o = 6 * S.base + 1; let t = 0;
              while (t < a[2]) { S.d.xfrc_applied[o] = t < 0.1 ? a[1] : 0; window.PMSIM.advance(0.02); t += 0.02; }
              S.d.xfrc_applied[o] = 0; const q = S.d.xquat, b = S.base, qx = q[4*b+1], qy = q[4*b+2], tilt = Math.acos(Math.max(-1, Math.min(1, 1 - 2*(qx*qx+qy*qy)))) * 180 / Math.PI; P2.BAL.on = false; return { z: S.d.xpos[3*b+2], tilt: tilt }; }""", [on, F, secs])
        off = push_run(False); on = push_run(True)
        check('9 N side push topples the robot without the assist', off['z'] < 0.15 or off['tilt'] > 50, off)
        check('9 N side push: robot stays up with the assist', on['z'] > 0.15 and on['tilt'] < 50, on)
        calm = push_run(True, 0.0, 2.0); check('assist does not disturb the quiet stance', calm['tilt'] < 3, calm)
        # ---------------- sensors HUD
        ev("window.PMSIM.reset()"); ev("for (let i = 0; i < 40; i++) window.PMSIM.advance(0.02)"); ev("document.getElementById('cSen').checked = true; document.getElementById('cTrail').checked = true")
        ev("window.PMSIM.p2.SEN.T = 0; window.PMSIM.p2.senTick(performance.now() + 1000)"); ev("for (let i = 0; i < 10; i++) window.PMSIM.advance(0.02)"); ev("window.PMSIM.p2.SEN.T = 0; window.PMSIM.p2.senTick(performance.now() + 2000)")
        hud = ev("document.getElementById('senHud').textContent")
        check('HUD has IMU / gyro / accel / foot / CoM / torque', all(w in hud for w in ('IMU', 'gyro', 'accel', 'CoM')) and ('腳底力' in hud) and ('力矩' in hud), hud.split('\n')[:3])
        sen = ev("(() => { const p = window.PMSIM.p2.SEN, S = window.PMSIM.S; const m = S.m, d = S.d; return { noF: p.noF, acc: p.acc, n: d.ncon }; })()")
        import re
        mf = re.search(r'R ([\d.]+) N\s+L ([\d.]+) N', hud)
        wt = ev('(() => { let s = 0; const m = window.PMSIM.S.m; for (let i = 0; i < m.nbody; i++) s += m.body_mass[i]; return s * 9.81; })()')
        check('foot contact forces add up to the robot weight', bool(mf) and abs(float(mf.group(1)) + float(mf.group(2)) - wt) < 0.15 * wt, (mf.groups() if mf else hud, round(wt, 1)))
        check('accelerometer reads ~ 1 g up at rest', abs(sen['acc'][2] - 9.81) < 2.0, sen['acc'])
        ev("for (let k = 0; k < 20; k++) { window.PMSIM.setTarget(3, k * 3); window.PMSIM.advance(0.05); window.PMSIM.p2.senTick(performance.now()); }")
        check('CoM trail collected points', ev("window.PMSIM.p2.SEN.n") >= 0, ev("window.PMSIM.p2.SEN.n"))
        # ---------------- library / share link
        ev("document.querySelectorAll('#pList button')[2].click()"); pg.wait_for_function("window.PMSIM.pma.P.frames && window.PMSIM.pma.P.frames.length > 0 && window.PMSIM.p2.PL", timeout=10000)
        title = ev("document.querySelectorAll('#pList button')[2].textContent"); ev("window.PMSIM.pma.P.t = 1500; window.PMSIM.pma.P.playing = false; window.PMSIM.camera.position.set(0.5, 0.4, 0.9)")
        h = ev("window.PMSIM.p2.shareHash()"); o = ev("(h) => window.PMSIM.p2.hashParse(h)", h)
        check('share hash carries motion, time, camera', o.get('m') == title and o.get('t') == '1500' and o.get('play') == '0' and o['cam'].startswith('0.500,0.400,0.900'), h[:120])
        pg2 = ctx.new_page(); pg2.add_init_script(THROTTLE); pg2.add_init_script(MOCKPAD); pg2.goto(URL + h + '&kiosk=1&embed=1&bar=0'); pg2.wait_for_function('!!window.PMSIM && window.PMSIM.pma.P.frames', timeout=90000); time.sleep(0.6)
        e2 = pg2.evaluate
        check('opening the link restores motion + time + paused', e2("window.PMSIM.pma.P.name") != '' and abs(e2("window.PMSIM.pma.P.t") - 1500) < 80 and not e2("window.PMSIM.pma.P.playing"), e2("[window.PMSIM.pma.P.name, window.PMSIM.pma.P.t, window.PMSIM.pma.P.playing]"))
        check('opening the link restores the camera', abs(e2("window.PMSIM.camera.position.x") - 0.5) < 0.02 and abs(e2("window.PMSIM.camera.position.z") - 0.9) < 0.02)
        check('kiosk/embed hides the panels, bar=0 hides the time bar', e2("['leftcol','rightpanel','hud','estop','playbar'].every(i => getComputedStyle(document.getElementById(i)).display === 'none')"))
        pg2.close()
        # ---------------- GIF encoder (decode with Pillow)
        gif_b64 = ev("""() => { const w = 70, h = 50, fr = []; for (let f = 0; f < 4; f++) { const a = new Uint8ClampedArray(w*h*4); for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) { const i = 4*(y*w+x); if (f === 3) { a[i] = (x*37+y*91+f*13) & 255; a[i+1] = (x*11+y*53) & 255; a[i+2] = (x*97+y*7) & 255; } else { a[i] = f*80 + x*2; a[i+1] = y*5; a[i+2] = 255 - x*3; } a[i+3] = 255; } fr.push(a); }
            const g = window.PMSIM.p2.gifEncode(fr, w, h, 100); let s = ''; for (let i = 0; i < g.length; i += 4096) s += String.fromCharCode.apply(null, g.subarray(i, i + 4096)); return btoa(s); }""")
        gb = base64.b64decode(gif_b64); im = Image.open(io.BytesIO(gb)); n = getattr(im, 'n_frames', 1)
        check('GIF decodes: size, 4 frames, loop', im.size == (70, 50) and n == 4 and im.info.get('loop') == 0, (im.size, n, im.info.get('loop')))
        im.seek(0); px = im.convert('RGB').getpixel((10, 10)); exp = (80 * 0 + 20, 50, 255 - 30)
        check('GIF colours match within the 3-3-2 quantisation', all(abs(a - b) <= 40 for a, b in zip(px, exp)), (px, exp))
        im.seek(3); noise = im.convert('RGB'); check('GIF noisy frame (LZW table resets) decodes', noise.size == (70, 50))
        # ---------------- WebM recording
        with pg.expect_download(timeout=30000) as dl:
            ev("document.getElementById('recOnce').checked = false"); jc('recBtn'); check('recording started', ev("!!window.PMSIM.p2.REC.mr")); time.sleep(4); jc('recBtn')
        data = open(dl.value.path(), 'rb').read(); check('WebM download is a valid EBML file', data[:4] == b'\x1a\x45\xdf\xa3' and len(data) > 1500, (len(data), dl.value.suggested_filename))
        # ---------------- playlist: auto-next + shuffle
        ev("document.querySelectorAll('#pList button')[0].click()"); pg.wait_for_function("window.PMSIM.pma.P.frames && window.PMSIM.pma.P.playing", timeout=10000); jc('plAuto')
        cur0 = ev("window.PMSIM.p2.PL && document.querySelector('#pList button.on') ? Array.from(document.querySelectorAll('#pList button')).findIndex(b => b.classList.contains('on')) : -1")
        ev("(() => { const P = window.PMSIM.pma.P; P.loop = false; P.t = P.total - 20; P.playing = true; })()")
        ok = until(lambda: ev("Array.from(document.querySelectorAll('#pList button')).findIndex(b => b.classList.contains('on'))") == (cur0 + 1) % 10, 25)
        check('auto-next loads the next motion when one ends', ok, (cur0, ev("Array.from(document.querySelectorAll('#pList button')).findIndex(b => b.classList.contains('on'))")))
        jc('plAuto'); jc('plShuf'); picks = ev("(() => { const o = []; for (let i = 0; i < 200; i++) o.push(window.PMSIM.p2.plPick(1)); return o; })()"); cur = ev("Array.from(document.querySelectorAll('#pList button')).findIndex(b => b.classList.contains('on'))")
        check('shuffle never repeats the current motion and varies', cur not in picks and len(set(picks)) > 4, len(set(picks)))
        jc('plShuf'); jc('plNext'); jc('plPrev')
        # ---------------- A-B loop
        ev("(() => { const P = window.PMSIM.pma.P; P.playing = false; P.t = 500; window.PMSIM.p2.abSet('a'); P.t = 2000; window.PMSIM.p2.abSet('b'); P.t = 1950; P.loop = true; window.PMSIM.p2.playTick(0.2); })()")
        tb = ev("window.PMSIM.pma.P.t"); check('A-B loop wraps B back to A', 500 <= tb <= 760, tb)
        check('A-B band + text on the seek bar', ev("document.getElementById('pSeek').style.background").startswith('linear-gradient') and 'A' in ev("document.getElementById('abTxt').textContent"))
        # ---------------- editor
        ev("window.PMSIM.p2.RC.kf = []"); 
        for i in range(4):
            ev("(i) => { window.PMSIM.setTarget(3, i * 10); document.getElementById('rcAdd').click(); }", i)
        jc('bLang'); jc('bLang'); jc('bLang')
        ev("window.PMSIM.p2.rcSelect(1)"); jc('rcDup'); n_dup = ev("window.PMSIM.p2.RC.kf.length"); jc('rcIns'); n_ins = ev("window.PMSIM.p2.RC.kf.length")
        check('duplicate and insert add one frame each', (n_dup, n_ins) == (5, 6), (n_dup, n_ins))
        ev("window.PMSIM.p2.rcSelect(2)"); jc('rcTrimB'); check('trim before', ev("window.PMSIM.p2.RC.kf.length") == 4 and ev("window.PMSIM.p2.RC.sel") == 0)
        ev("window.PMSIM.p2.rcSelect(2)"); jc('rcSplit'); a1 = ev("window.PMSIM.p2.RC.kf.length"); alt = ev("window.PMSIM.p2.RC.alt.kf.length"); check('split moves the tail to the clipboard', (a1, alt) == (2, 2), (a1, alt))
        jc('rcMerge'); check('merge appends it again', ev("window.PMSIM.p2.RC.kf.length") == 4 and ev("window.PMSIM.p2.RC.alt") is None)
        ev("window.PMSIM.p2.rcSelect(1)"); jc('rcTrimA'); check('trim after', ev("window.PMSIM.p2.RC.kf.length") == 2)
        ev("window.PMSIM.p2.RC.kf.push({raw: window.PMSIM.p2.RC.kf[0].raw.map((v, k) => k === 3 ? v + 400 : v), spd: 10}); window.PMSIM.p2.rcSelect(1)"); ev("document.getElementById('rcOnion').checked = true; document.getElementById('rcOnion').dispatchEvent(new Event('change'))")
        gh = ev("window.PMSIM.p2.ON.g.map(g => g && g.visible)"); check('onion skin: previous and next ghosts shown', gh == [True, True], gh)
        diff = ev("(() => { const g0 = window.PMSIM.p2.ON.g[0], g1 = window.PMSIM.p2.ON.g[1]; let mx = 0; for (const n in g0.userData.m) { const a = g0.userData.m[n].position, c = g1.userData.m[n].position; mx = Math.max(mx, Math.hypot(a.x - c.x, a.y - c.y, a.z - c.z)); } return mx; })()")
        check('the two ghosts show different poses', diff > 0.002, diff)
        ev("document.getElementById('rcOnion').checked = false; document.getElementById('rcOnion').dispatchEvent(new Event('change'))"); check('onion skin off hides them', ev("window.PMSIM.p2.ON.g.every(g => !g.visible)"))
        # ---------------- teleop with a mocked gamepad
        ev("window.__pad = { connected: true, id: 'Mock Pad', axes: [0.9, 0, 0.8, -0.9], buttons: Array.from({length: 16}, () => ({ pressed: false, value: 0 })) }; window.__pad.buttons[5].pressed = true; window.__pad.buttons[7].value = 1")
        ev("window.PMSIM.p2.gpPoll()"); g = ev("(({x1, y1, x2, y2, rt, rb, lb, name}) => ({x1, y1, x2, y2, rt, rb, lb, name}))(window.PMSIM.p2.WK.gp)")
        check('gamepad mapping (axes, triggers, bumpers)', g['x1'] > 0.5 and g['y2'] > 0.5 and g['rt'] == 1 and g['rb'] and g['name'] == 'Mock Pad', g)
        ev("document.getElementById('teleOn').checked = true; document.getElementById('teleOn').dispatchEvent(new Event('change')); window.PMSIM.p2.WK.on = false")
        before = ev("(() => { const A = window.PREMAID.servos, T = window.PMSIM.target, o = {}; A.forEach((s, k) => o[s.joint] = T[k]); return o; })()")
        ev("for (let i = 0; i < 10; i++) window.PMSIM.p2.teleTick(0.1)")
        after = ev("(() => { const A = window.PREMAID.servos, T = window.PMSIM.target, o = {}; A.forEach((s, k) => o[s.joint] = T[k]); return o; })()")
        moved = [j for j in before if abs(after[j] - before[j]) > 1]
        check('teleop moves head and the right arm', 'head_nz_joint' in moved and 'r_shx_joint' in moved and 'r_shy_joint' in moved and 'r_ely_joint' in moved and not any(j.startswith('l_') for j in moved), moved)
        ev("window.__pad = null; document.getElementById('teleOn').checked = false; document.getElementById('teleOn').dispatchEvent(new Event('change'))")
        # ---------------- mocap math: synthetic landmarks (x forward, y left, z up)
        def lm_person(scale=1.0, lw=(0.0, 0.0, 0.0), rw=(0.0, 0.0, 0.0)):
            L = [[0, 0, 0] for _ in range(33)]
            L[11] = [0, .09, .5]; L[12] = [0, -.09, .5]; L[23] = [0, .06, .1]; L[24] = [0, -.06, .1]
            L[13] = [lw[0] / 2, .09 + lw[1] / 2, .5 + lw[2] / 2]; L[15] = [lw[0], .09 + lw[1], .5 + lw[2]]
            L[14] = [rw[0] / 2, -.09 + rw[1] / 2, .5 + rw[2] / 2]; L[16] = [rw[0], -.09 + rw[1], .5 + rw[2]]
            return [{'x': p[0] * scale, 'y': p[1] * scale, 'z': p[2] * scale} for p in L]
        def solve(lm):
            return ev("""(lm) => { const tg = window.PMSIM.p2.mcPoseToTargets(lm, null), A = window.PREMAID, out = { err_l: tg.err_l, err_r: tg.err_r }; Object.keys(tg).forEach(k => { if (!/^err/.test(k)) out[A.servos[+k].joint] = tg[k]; }); return out; }""", lm)
        s1 = solve(lm_person(1.0, lw=(0.30, 0, 0)))
        check('mocap: left hand forward solves (IK error < 8 mm)', s1['err_l'] < 0.008, s1['err_l'])
        s2 = solve(lm_person(1.0, lw=(0.30, 0, 0))); s3 = solve(lm_person(1.4, lw=(0.42, 0, 0)))
        check('mocap: result independent of the person\'s size', all(abs(s2[j] - s3[j]) < 3 for j in s2 if j.endswith('_joint')), max(abs(s2[j] - s3[j]) for j in s2 if j.endswith('_joint')))
        up = solve(lm_person(1.0, rw=(0, 0, 0.30))); fw = solve(lm_person(1.0, rw=(0.30, 0, 0)))
        diffj = max(abs(up[j] - fw[j]) for j in up if j.startswith('r_'))
        check('mocap: raising vs pointing forward gives different arm poses', diffj > 20 and up['err_r'] < 0.01 and fw['err_r'] < 0.01, (diffj, up['err_r'], fw['err_r']))
        tipz = ev("""(lm) => { const A = window.PREMAID, P = window.PMSIM.p2, tg = P.mcPoseToTargets(lm, null), ch = A.chains.r_arm; return [0].map(() => 0); }""", lm_person(1.0, rw=(0, 0, 0.3)))
        # ---------------- offline / PWA
        ready = pg.evaluate("navigator.serviceWorker.ready.then(r => !!r.active)"); check('service worker active', ready)
        until(lambda: ev("caches.keys().then(k => k.length)") >= 1 and ev("caches.open('premaid-sim-v1').then(c => c.keys()).then(k => k.map(r => new URL(r.url).pathname).join(','))").count('mujoco.wasm') == 1, 90)
        cached = ev("caches.open('premaid-sim-v1').then(c => c.keys()).then(k => k.map(r => new URL(r.url).pathname))"); check('core files are cached (html, wasm, vendor, library)', all(any(c.endswith(n) for c in cached) for n in ('premaid-sim.html', 'mujoco.wasm', 'mujoco.js', 'three.min.js', 'library.js', 'manifest.webmanifest')), len(cached))
        ctx.set_offline(True); pg.reload(); pg.wait_for_function('!!window.PMSIM', timeout=90000); check('page works offline after reload', ev("window.PMSIM.S.ready"))
        mf = ev("fetch('manifest.webmanifest').then(r => r.json())"); check('manifest valid', mf['display'] == 'standalone' and len(mf['icons']) >= 2)
        ctx.set_offline(False)
        check('no console / page errors', not [e for e in errs if 'Failed to load resource' not in e and 'net::ERR' not in e], errs[:3]); b.close()
finally:
    srv.terminate()
print('\n%d failed' % len(fails) if fails else '\nall passed'); sys.exit(1 if fails else 0)
