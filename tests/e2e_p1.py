"""Browser end-to-end test of the P1 real-robot features (not part of CI: needs playwright + chromium).
Chromium -> WebSocket -> tools/ws_bridge.py -> pty -> FakeRobot.   Web Serial itself cannot be driven headless.
    pip install playwright aiohttp pyserial && playwright install chromium && python tests/e2e_p1.py"""
import os, pty, subprocess, sys, time, tty
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)
from fake_robot import FakeRobot, pkt
from playwright.sync_api import sync_playwright

PORT = 8791; fails = []
def until(fn, timeout=20):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if fn(): return True
        time.sleep(0.1)
    return False
def check(name, ok, info=''):
    print(('PASS ' if ok else 'FAIL ') + name + (' ' + str(info) if info != '' else '')); ok or fails.append(name)

master, slave = pty.openpty(); tty.setraw(master); tty.setraw(slave)
robot = FakeRobot(master, range(1, 0x28))
def start_bridge():
    return subprocess.Popen([sys.executable, os.path.join(ROOT, 'tools', 'ws_bridge.py'), os.ttyname(slave), '--port', str(PORT)], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
br = start_bridge(); time.sleep(1.2)
try:
    with sync_playwright() as pw:
        b = pw.chromium.launch(); pg = b.new_page(viewport={'width': 1280, 'height': 720}); errs = []
        pg.on('console', lambda m: errs.append(m.text) if m.type == 'error' else None); pg.on('pageerror', lambda e: errs.append(str(e)))
        pg.on('dialog', lambda d: d.accept())
        pg.add_init_script('window.requestAnimationFrame = f => setTimeout(() => f(performance.now()), 2000)')   # software GL runs at ~3 fps here and would starve the timers
        pg.goto('http://127.0.0.1:%d/web/premaid-sim.html' % PORT); pg.wait_for_function('!!window.PMSIM', timeout=90000)
        ev = pg.evaluate
        def sel(i, v): ev("(a) => { const e = document.getElementById(a[0]); e.value = a[1]; e.dispatchEvent(new Event('change')); }", [i, v])
        def jc(i): ev("(i) => document.getElementById(i).click()", i)
        sv = ev('window.PREMAID.servos.map(s => [s.joint, s.pid, s.sign])'); N = len(sv)
        pid = {j: p for j, p, _ in sv}; sign = {j: s for j, _, s in sv}; idx = {j: i for i, (j, _, _) in enumerate(sv)}
        # UI layout checks
        check('no PMA mapping table', ev("!document.getElementById('pmaMap')"))
        check('no red/orange warning boxes', ev("document.querySelectorAll('.warnbox').length") == 0)
        check('library box shows names only', ev("Array.from(document.querySelectorAll('#pList button')).every(b => b.children.length === 1 && b.textContent.trim().length > 0)") and ev("document.querySelectorAll('#pList button').length") == 10, ev("document.querySelectorAll('#pList button').length"))
        er = ev("(() => { const r = document.getElementById('estop').getBoundingClientRect(), p = document.getElementById('playbar').getBoundingClientRect(); return [r.bottom <= p.top || r.top >= p.bottom || r.right <= p.left || r.left >= p.right, r.left, r.top, r.width, r.height]; })()")
        check('E-stop does not overlap the time bar', er[0], er)
        pg.click('[data-tab=robot]'); check('20 built-in buttons', ev("document.querySelectorAll('#rbBuiltinGrid button').length") == 20)
        check('default WS url points at the page host', ev("document.getElementById('wsUrl').value") == 'ws://127.0.0.1:%d/ws' % PORT)
        jc('wsConn'); pg.wait_for_function("window.PMSIM.rb.state.msg === 'rbOn'", timeout=5000); check('websocket connected', True)
        robot.rx.clear(); ev("document.querySelectorAll('#rbBuiltinGrid button')[2].click()"); until(lambda: any(r[:4] == bytes([5, 0x1F, 0, 0x3F]) for r in robot.rx), 5); check('built-in button 3 plays 0x3F in one tap', any(r[:4] == bytes([5, 0x1F, 0, 0x3F]) for r in robot.rx))
        pg.evaluate("document.getElementById('stOn').click()")
        pg.wait_for_function('window.PMSIM.p1.ST.ok > %d' % (N + 5), timeout=15000)
        check('every servo answered a status query', ev('window.PMSIM.p1.ST.info.filter(Boolean).length') == N, ev('window.PMSIM.p1.ST.info.filter(Boolean).length'))
        check('status table filled (actual col)', ev("document.querySelector('#stBody tr td:nth-child(3)').textContent") == '7500')
        check('latency + cycle measured', ev('window.PMSIM.p1.ST.lat > 0 && window.PMSIM.p1.ST.cyc > 0'), ev('[window.PMSIM.p1.ST.lat, window.PMSIM.p1.ST.cyc]'))
        pg.wait_for_function('window.PMSIM.p1.BV.length >= 1', timeout=6000); check('battery graph sample (11.1 V)', abs(ev('window.PMSIM.p1.BV[0].v') - 11.1) < 0.02, ev('window.PMSIM.p1.BV[0].v'))
        check('monitor decodes replies', until(lambda: 'volt 11.10 V' in ev("document.getElementById('mnLog').textContent"), 8))
        # digital twin: robot moved by hand -> simulator target follows
        j = 'r_shy_joint'; robot.enc[pid[j]] = 6700; k = idx[j]
        pg.evaluate("document.getElementById('stTwin').click()")
        want = sign[j] * (6700 - 7500) * 270 / 8000
        pg.wait_for_function('Math.abs(window.PMSIM.target[%d] - (%f)) < 0.5' % (k, want), timeout=8000); check('twin: sim target follows robot encoder', True, round(ev('window.PMSIM.target[%d]' % k), 1))
        # E-stop
        robot.rx.clear(); pg.click('#estop'); time.sleep(0.6)
        check('E-stop sends 04 05 00 01 and 05 1F 08 01', pkt(4, 5, 0, 1)[:4] in [r[:4] for r in robot.rx] and pkt(5, 0x1F, 8, 1) in robot.rx, len(robot.rx))
        check('E-stop leaves twin/polling usable, live control off', ev("!document.getElementById('rbLive').checked"))
        # left -> right teaching: leader side released, follower driven to the mirrored position
        sel('mrMode', 'lr'); robot.rx.clear(); pg.click('[data-tab=rec]'); pg.click('#tcStart'); until(lambda: all(robot.cmd[p] == 0 for p in [q for jj, q, _ in sv if jj.startswith('l_')]))
        lead = [p for jj, p, _ in sv if jj.startswith('l_')]; foll = [p for jj, p, _ in sv if jj.startswith('r_')]
        check('teach start releases only the left servos', all(robot.cmd[p] == 0 for p in lead) and all(robot.cmd[p] != 0 for p in foll))
        jl, jr = 'l_shy_joint', 'r_shy_joint'; m = ev('window.PREMAID.mirror.filter(e => e.k === %d && e.p === %d)[0].m' % (idx[jr], idx[jl]))
        robot.enc[pid[jl]] = 8300
        exp = 7500 + sign[jr] * sign[jl] * m * (8300 - 7500); until(lambda: abs(robot.cmd[pid[jr]] - exp) <= 1, 30)
        check('left -> right: right servo driven to mirrored raw', abs(robot.cmd[pid[jr]] - exp) <= 1, (robot.cmd[pid[jr]], exp))
        # capture keyframe from live readings, CSV
        pg.wait_for_function('window.PMSIM.p1.ST.info[%d] && window.PMSIM.p1.ST.info[%d].enc === 8300' % (idx[jl], idx[jl]), timeout=30000); n0 = ev('window.PMSIM.p1.RC.kf.length'); pg.click('#tcCap'); n1 = ev('window.PMSIM.p1.RC.kf.length')
        check('capture adds a keyframe with the hand-posed raw', n1 == n0 + 1 and ev('window.PMSIM.p1.RC.kf[%d].raw[%d]' % (n1 - 1, idx[jl])) == 8300)
        csv = ev('window.PMSIM.p1.rcCsv()'); check('CSV has header + rows', csv.startswith('frame,time_ms,spd,') and len(csv.strip().split('\r\n')) == n1 + 1)
        pg.click('#tcHold'); until(lambda: robot.cmd[pid[jl]] == 8300, 10); check('hold pose stiffens released servos at their position', robot.cmd[pid[jl]] == 8300, robot.cmd[pid[jl]])
        pg.click('#tcAuto'); time.sleep(1.5); pg.click('#tcAuto'); check('auto capture added frames', ev('window.PMSIM.p1.RC.kf.length') >= n1 + 3, ev('window.PMSIM.p1.RC.kf.length'))
        # simulator-only L/R follow on sliders
        jc('stTwin'); sel('mrMode', 'any'); time.sleep(0.3); ev('window.PMSIM.setTarget(%d, 30)' % idx['r_shx_joint']); ev('window.PMSIM.p1.mirFollow(%d)' % idx['r_shx_joint'])
        m2 = ev('window.PREMAID.mirror.filter(e => e.k === %d && e.p === %d)[0].m' % (idx['l_shx_joint'], idx['r_shx_joint']))
        check('sim: partner follows slider', abs(ev('window.PMSIM.target[%d]' % idx['l_shx_joint']) - m2 * ev('window.PMSIM.target[%d]' % idx['r_shx_joint'])) < 0.2)
        # LED probe is locked until acknowledged
        check('LED send locked by default', ev("document.getElementById('ledSend').disabled"))
        # monitor raw send
        robot.rx.clear(); ev("document.getElementById('mnHex').value = '01 00 02 00 02'"); jc('mnSend'); time.sleep(0.4); check('raw send adds LEN+XOR', any(r[:6] == bytes([7, 1, 0, 2, 0, 2]) for r in robot.rx))
        # reconnect after the bridge dies
        br.terminate(); br.wait(); pg.wait_for_function("window.PMSIM.rb.state.msg === 'rbLost'", timeout=8000); check('link loss detected', True)
        br = start_bridge(); pg.wait_for_function("window.PMSIM.rb.state.msg === 'rbOn'", timeout=20000); check('auto reconnect', True)
        ok0 = ev('window.PMSIM.p1.ST.ok'); pg.wait_for_function('window.PMSIM.p1.ST.ok > %d + 30' % ok0, timeout=10000); check('polling resumes after reconnect', True)
        # user disconnect stays disconnected
        jc('rbConn'); time.sleep(0.8); check('manual disconnect does not auto-reconnect', ev("window.PMSIM.rb.state.msg") == 'rbOff')
        check('E-stop works without a link (page-side)', (pg.click('#estop') or True) and ev("window.PMSIM.rb.state.motion") == 'estopNoLink')
        check('no console / page errors', not errs, errs[:3]); b.close()
finally:
    br.terminate(); robot.stop()
print('\n%d failed' % len(fails) if fails else '\nall passed'); sys.exit(1 if fails else 0)
