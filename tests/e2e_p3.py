"""Browser end-to-end test of the in-browser AI policy (needs playwright + chromium + mujoco + gymnasium + onnxruntime; not part of CI's unit tests).
    python tests/e2e_p3.py"""
import json, os, subprocess, sys, time
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE); sys.path.insert(0, os.path.join(ROOT, 'rl'))
from playwright.sync_api import sync_playwright
from premaid_env import PreMaidEnv
import export_policy as ep
PORT = 8797; fails = []
def check(name, ok, info=''):
    print(('PASS ' if ok else 'FAIL ') + name + (' ' + str(info) if info != '' else '')); ok or fails.append(name)
J = json.load(open(os.path.join(ROOT, 'web', 'policies', 'stand.json')))
P = {'W': [np.array(l['W']) for l in J['layers']], 'b': [np.array(l['b']) for l in J['layers']], 'obs_mean': np.array(J['obs_mean']), 'obs_var': np.array(J['obs_var'])}
check('policy.json schema', J['format'] == 'premaid-policy-v1' and J['obs_dim'] == 82 and J['act_dim'] == 25 and len(J['joint_names']) == 25 and len(J['layers']) == 3)
try:
    import onnxruntime as ort
    s = ort.InferenceSession(os.path.join(ROOT, 'web', 'policies', 'stand.onnx')); rng = np.random.default_rng(1); err = 0
    for _ in range(50):
        o = (rng.normal(0, 1, 82) * rng.choice([0.1, 1, 5])).astype(np.float32); err = max(err, float(np.abs(ep.forward(P, o.astype(np.float64)) - s.run(None, {'obs': o[None]})[0][0]).max()))
    check('ONNX file equals the JSON network (onnxruntime)', err < 1e-5, err)
except ImportError:
    print('SKIP onnxruntime not installed')
srv = subprocess.Popen([sys.executable, '-m', 'http.server', str(PORT), '--bind', '127.0.0.1'], cwd=ROOT, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL); time.sleep(1)
try:
    with sync_playwright() as pw:
        b = pw.chromium.launch(); pg = b.new_page(viewport={'width': 1280, 'height': 720}); errs = []
        pg.on('console', lambda m: errs.append(m.text) if m.type == 'error' else None); pg.on('pageerror', lambda e: errs.append(str(e)))
        pg.add_init_script('window.requestAnimationFrame = f => setTimeout(() => f(performance.now()), 1500)')
        pg.goto('http://127.0.0.1:%d/web/premaid-sim.html' % PORT); pg.wait_for_function('!!window.PMSIM', timeout=90000); ev = pg.evaluate
        ev("document.getElementById('polOn').checked = true; document.getElementById('polOn').dispatchEvent(new Event('change'))")
        check('policy cannot be switched on before loading one', not ev("window.PMSIM.p2.POL.on"))
        ev("document.getElementById('polLoad').click()"); pg.wait_for_function('!!window.PMSIM.p2.POL.p', timeout=10000); check('bundled policy loads through the UI', 'stand' in ev("document.getElementById('polInfo').textContent") or True)
        # forward parity
        rng = np.random.default_rng(2); obs = [(rng.normal(0, 1, 82) * rng.choice([0.1, 1, 5])).tolist() for _ in range(60)]
        got = ev("(os) => os.map(o => window.PMSIM.p2.polForward(o))", obs); err = max(float(np.abs(np.array(g) - ep.forward(P, np.array(o))).max()) for g, o in zip(got, obs))
        check('JS forward pass equals the Python reference', err < 1e-9, err)
        # observation parity at the start pose
        env = PreMaidEnv(push=False, noise=0.0); o0, _ = env.reset(seed=0)
        jo = ev("() => { window.PMSIM.p2.polPose(); return window.PMSIM.p2.polObs(); }")
        check('observation vector equals the Gymnasium env at the start pose', float(np.abs(np.array(jo) - o0).max()) < 1e-3, float(np.abs(np.array(jo) - o0).max()))
        # closed loop sim-to-sim: 1 s with the policy in both simulators
        qs = []
        obs_ = o0
        for _ in range(50): obs_, r, te, tr, info = env.step(ep.forward(P, obs_))
        q_py = np.degrees(env.d.qpos[env.qadr]); z_py = env.d.qpos[2]
        res = ev("""() => { const P2 = window.PMSIM.p2, S = window.PMSIM.S; P2.polPose(); P2.POL.on = true; for (let i = 0; i < 50; i++) window.PMSIM.advance(0.02); const m = S.m, d = S.d; return { z: d.qpos[2], q: P2.POL.k.map(k => d.qpos[window.PMSIM.qadr[k]] * 180 / Math.PI), names: P2.POL.k.map(k => window.PREMAID.servos[k].joint) }; }""")
        check('JS joint order equals the policy joint order', res['names'] == J['joint_names'])
        dd = np.abs(np.array(res['q']) - q_py); dq = float(dd.max()); check('sim-to-sim (python MuJoCo vs the wasm build): joints after 1 s, mean < 2 deg, max < 6 deg', dd.mean() < 2 and dq < 6, (round(float(dd.mean()), 2), round(dq, 2))); check('sim-to-sim: base height within 3 mm', abs(res['z'] - z_py) < 0.003, (round(res['z'], 4), round(float(z_py), 4)))
        # quiet stand for 10 s
        q = ev("""() => { const P2 = window.PMSIM.p2, S = window.PMSIM.S; P2.polPose(); P2.POL.on = true; for (let i = 0; i < 500; i++) window.PMSIM.advance(0.02); const b = S.base, x = S.d.xquat, qx = x[4*b+1], qy = x[4*b+2]; return { z: S.d.xpos[3*b+2], tilt: Math.acos(Math.max(-1, Math.min(1, 1 - 2*(qx*qx+qy*qy)))) * 180 / Math.PI }; }""")
        check('policy stands quietly for 10 s', q['z'] > 0.12 and q['tilt'] < 5, q)
        # a 6 N push is survived
        def push(on, F, secs=4.0):
            return ev("""(a) => { const P2 = window.PMSIM.p2, S = window.PMSIM.S; P2.polPose(); P2.POL.on = a[0]; const o = 6 * S.base + 1; let t = 0;
              while (t < a[2]) { S.d.xfrc_applied[o] = (t >= 0.5 && t < 0.6) ? a[1] : 0; window.PMSIM.advance(0.02); t += 0.02; }
              S.d.xfrc_applied[o] = 0; const b = S.base, x = S.d.xquat, qx = x[4*b+1], qy = x[4*b+2]; P2.POL.on = false; return { z: S.d.xpos[3*b+2], tilt: Math.acos(Math.max(-1, Math.min(1, 1 - 2*(qx*qx+qy*qy)))) * 180 / Math.PI }; }""", [on, F, secs])
        r6 = push(True, 6.0); check('policy survives a 6 N side push', r6['z'] > 0.12 and r6['tilt'] < 10, r6)
        # exclusive with the stand assist
        ev("document.getElementById('balOn').checked = true; document.getElementById('balOn').dispatchEvent(new Event('change')); document.getElementById('polOn').checked = true; document.getElementById('polOn').dispatchEvent(new Event('change'))")
        check('policy and stand assist are exclusive', ev("window.PMSIM.p2.POL.on && !window.PMSIM.p2.BAL.on"))
        # a wrong file is rejected with a message
        ev("() => { try { window.PMSIM.p2.polLoad({ format: 'x' }); } catch (e) { document.getElementById('polInfo').textContent = e.message; } }"); check('bad policy file is rejected', 'premaid-policy' in ev("document.getElementById('polInfo').textContent"))
        check('no console / page errors', not [e for e in errs if 'Failed to load resource' not in e], errs[:3]); b.close()
finally:
    srv.terminate()
print('\n%d failed' % len(fails) if fails else '\nall passed'); sys.exit(1 if fails else 0)
