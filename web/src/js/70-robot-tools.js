// ------------------------------------------------------------------ P1: real-robot digital twin, teach & record, WebSocket bridge, monitor, E-stop
// 0x01 servo query: request [07][01][00][05][pid][0E][xor]; reply [18][01][00][14-byte block][xor] = pid, flag(0x80 valid), encoder LE, trim, command LE, gyro flag x2, gyro gain, speed, stretch
// (layout from the community analysis memo, implemented by github.com/arthurson/premaiduino2 PmaProtocol.h; the 14-byte block is verified against that firmware, not against a factory robot)
var ST = { on: false, twin: false, i: 0, out: {}, win: 3, info: [], fail: [], cells: [], lat: 0, ok: 0, miss: 0, bad: 0, round: 0, cyc: 0, cycT: 0, sumT: 0 };
var MIR = { mode: 'off', out: {}, sent: {}, last: 0 };
var TC = { timer: 0 };
var MN = { lines: [], paused: false, hidePoll: true, dirty: false, timer: 0, tx: 0, rx: 0 };
var BV = [], LK = { want: false, kind: '', wsUrl: '', tries: 0, timer: 0, ws: null, wake: null, info: null };
function h2(x) { return ('0' + (x & 255).toString(16).toUpperCase()).slice(-2); }
function hexStr(u8) { return Array.prototype.map.call(u8, h2).join(' '); }
function s16(lo, hi) { var v = (lo & 255) | ((hi & 255) << 8); return v > 32767 ? v - 65536 : v; }
function sideOf(k) { var j = A.servos[k].joint; return j.indexOf('l_') === 0 ? 'l' : j.indexOf('r_') === 0 ? 'r' : ''; }
function decodeServoInfo(p) {      // null unless a well-formed, checksummed 0x01 servo reply
  if (!p || p.length !== 18 || p[0] !== 18 || p[1] !== 0x01 || p[2] !== 0 || xr(Array.prototype.slice.call(p, 0, 17)) !== p[17]) return null;
  var gy = (p[11] === 0xFF && p[12] === 0xFF) ? '' : (p[11] === 2 && p[12] === 5) ? 'pitch' : (p[11] === 2 && p[12] === 7) ? 'roll' : h2(p[11]) + h2(p[12]);
  return { pid: p[3], valid: (p[4] & 0x80) !== 0, enc: p[5] | (p[6] << 8), trim: s16(p[7], p[8]), cmd: p[9] | (p[10] << 8), gyro: gy, gyroGain: s16(p[13], p[14]), speed: p[15], stretch: p[16], t: 0 };
}
function poseBytesSome(ks, raws, spd) {      // 0x18 pose that carries only the listed servos (0 = release)
  var b = [5 + 3 * ks.length, 0x18, 0, Math.max(1, Math.min(255, spd | 0))];
  ks.forEach(function (k, i) { var v = raws[i]; b.push(A.servos[k].pid, v & 255, (v >> 8) & 255); });
  return mk(b);
}
// ---- servo status polling (3 requests in flight, a new one as each reply arrives, timeout 200 ms or 4x the measured latency)
function stNext() {
  for (var n = 0; n < N; n++) {
    var k = ST.i; ST.i = (ST.i + 1) % N;
    if (!ST.i) { var now = performance.now(); ST.round++; ST.cyc = ST.cycT ? now - ST.cycT : 0; ST.cycT = now; }
    var inf = ST.info[k];
    if (((inf && !inf.valid) || ST.fail[k] >= 3) && ST.round % 8) continue;     // dead / unwired servos are probed once every 8 rounds
    return k;
  }
  return 0;
}
function stPoll() {
  if (!ST.on || !RB.writer || (P.playing && $('rbStream').checked)) return;     // never compete with a streamed motion
  var now = performance.now(), to = Math.max(200, Math.min(1000, ST.lat * 4)), n = 0;
  Object.keys(ST.out).forEach(function (q) { if (now - ST.out[q] >= to) { var kk = PM.pid2k[q]; ST.miss++; ST.fail[kk] = (ST.fail[kk] | 0) + 1; delete ST.out[q]; } else n++; });
  while (n < ST.win) {                 // a few requests in flight: throughput no longer limited by one round trip
    var k = stNext(), pid = A.servos[k].pid; if (ST.out[pid] !== undefined) break;
    ST.out[pid] = now; n++; rbSend(mk([7, 0x01, 0, 5, pid, 14]));
  }
}
setInterval(stPoll, 60);        // watchdog: restarts the chain after a lost reply
function stRx(p) {
  var d = decodeServoInfo(p); if (!d) { ST.bad++; return; }
  var k = PM.pid2k[d.pid]; if (k === undefined) return;
  var now = performance.now(); d.t = now;
  if (ST.out[d.pid] !== undefined) { var dt = now - ST.out[d.pid]; ST.lat = ST.lat ? ST.lat * 0.8 + dt * 0.2 : dt; delete ST.out[d.pid]; }
  ST.info[k] = d; ST.fail[k] = 0; ST.ok++;
  twinApply(k, d); mirDrive(k, d); stCell(k, d);
  setTimeout(stPoll, 0);
}
function stCell(k, d) {
  var c = ST.cells[k]; if (!c) return;
  c.row.classList.toggle('na', !d.valid); c.row.classList.remove('stale');
  c.act.textContent = d.valid ? d.enc : '—'; c.deg.textContent = d.valid ? (A.servos[k].sign * (d.enc - 7500) * K_DEG).toFixed(1) : '';
  c.cmd.textContent = d.cmd ? d.cmd : (d.valid ? 'free' : '—'); c.sp.textContent = d.speed + '/' + d.stretch;
  c.gy.textContent = d.gyro ? d.gyro + (d.gyroGain ? ' ' + d.gyroGain : '') : '';
}
(function stBuild() {
  var tb = $('stBody'); if (!tb) return;
  A.servos.forEach(function (s, k) {
    var tr = document.createElement('tr'); tr.innerHTML = '<td class="mono">' + h2(s.pid) + '</td><td class="mono"></td><td class="mono"></td><td class="mono"></td><td class="mono"></td><td class="mono"></td><td class="mono"></td>';
    tr.children[1].textContent = s.joint.replace('_joint', ''); var c = tr.children;
    ST.cells[k] = { row: tr, act: c[2], deg: c[3], cmd: c[4], sp: c[5], gy: c[6] }; tb.appendChild(tr);
  });
  setInterval(function () { var now = performance.now(); ST.cells.forEach(function (c, k) { var d = ST.info[k]; c.row.classList.toggle('stale', !!d && now - d.t > 3000); }); }, 500);
})();
// ---- digital twin: the robot's encoder positions drive the simulator targets
function twinApply(k, d) {
  if (!ST.twin || !d.valid || !d.enc) return;
  var deg = A.servos[k].sign * (d.enc - 7500) * K_DEG;
  if (Math.abs(deg - target[k]) > 0.15) setTarget(k, deg);
}
// ---- left / right following. Simulator: moving a slider moves its partner (A.mirror: servo k = m x angle of servo p).
// Real robot (twin on, direction L->R or R->L): the leader side is released and moved by hand, the other side is driven to the mirrored position.
function mirFollow(k) {
  if (MIR.mode === 'off') return; var side = sideOf(k);
  if ((MIR.mode === 'lr' && side !== 'l') || (MIR.mode === 'rl' && side !== 'r')) return;
  A.mirror.forEach(function (e) { if (e.p === k && e.k !== k) setTarget(e.k, e.m * target[k]); });
}
function mirDrive(k, d) {
  if ((MIR.mode !== 'lr' && MIR.mode !== 'rl') || !ST.twin || !RB.writer || !d.valid || !d.enc) return;
  if (sideOf(k) !== (MIR.mode === 'lr' ? 'l' : 'r')) return;
  A.mirror.forEach(function (e) {
    if (e.p !== k || e.k === k) return;
    var v = Math.round(7500 + A.servos[e.k].sign * A.servos[k].sign * e.m * (d.enc - 7500));
    MIR.out[e.k] = $('rbSafe').checked ? rawClampSafe(e.k, v) : rcClampRaw(e.k, v);
  });
  var now = performance.now(); if (now - MIR.last < 70) return;
  var ks = [], rs = []; Object.keys(MIR.out).forEach(function (key) { var q = +key, v = MIR.out[q]; if (MIR.sent[q] === undefined || Math.abs(v - MIR.sent[q]) >= 6) { ks.push(q); rs.push(v); } });
  if (!ks.length) return; MIR.last = now; ks.forEach(function (q, i) { MIR.sent[q] = rs[i]; });
  rbSend(poseBytesSome(ks, rs, 2));
}
// ---- teach: release the servos, move the robot by hand, capture keyframes (manual or every N ms) -> the keyframe recorder -> .pma / CSV
function teachStart() {
  if (!RB.writer) return;
  var lead = MIR.mode === 'lr' ? 'l' : MIR.mode === 'rl' ? 'r' : '';
  if (!confirm(t(lead ? 'teachConfirmSide' : 'teachConfirm'))) return;
  $('rbLive').checked = false; $('rbStream').checked = false; touch(); if (typeof WK !== 'undefined') WK.on = false;
  ST.on = true; ST.twin = true; $('stOn').checked = true; $('stTwin').checked = true; MIR.sent = {}; MIR.out = {};
  var ks = [], rs = []; for (var k = 0; k < N; k++) { if (lead && sideOf(k) !== lead) continue; ks.push(k); rs.push(0); }
  RB.retry = 0; RB.lastPkt = poseBytesSome(ks, rs, 2); rbSend(RB.lastPkt, true); RB.motion = ''; renderRb(); stPoll();
}
function teachHold() {          // stiffen every servo at the position it has right now (stops the limp state)
  if (!RB.writer) return; var ks = [], rs = [], now = performance.now();
  for (var k = 0; k < N; k++) { var d = ST.info[k]; if (d && d.valid && d.enc && now - d.t < 3000) { ks.push(k); rs.push(rcClampRaw(k, d.enc)); } }
  if (!ks.length) { $('tcInfo').textContent = t('tcNoData'); return; }
  RB.retry = 0; RB.lastPkt = poseBytesSome(ks, rs, 10); rbSend(RB.lastPkt, true); $('tcInfo').textContent = t('tcHeld', ks.length);
}
function teachCapture(auto) {
  var base = rcRaw(), raw = [], live = 0, now = performance.now();
  for (var k = 0; k < N; k++) { var d = ST.info[k]; if (d && d.valid && d.enc && now - d.t < 3000) { raw[k] = rcClampRaw(k, d.enc); live++; } else raw[k] = base[k]; }
  var spd = auto ? Math.max(1, Math.min(255, Math.round((+$('tcMs').value || 300) / 15))) : (+$('rcSpd').value || 10);
  RC.kf.push({ raw: raw, spd: spd }); rcSave(); rcRender(); $('rcList').scrollTop = 1e6; $('tcInfo').textContent = t('tcCaptured', RC.kf.length, live);
  return live;
}
function teachStopAuto() { clearInterval(TC.timer); TC.timer = 0; tcRender(); }
function teachAuto() {
  if (TC.timer) { teachStopAuto(); return; } if (!RB.writer || !ST.on) { $('tcInfo').textContent = t('tcNeedPoll'); return; }
  TC.timer = setInterval(function () { teachCapture(true); }, Math.max(100, +$('tcMs').value || 300)); teachCapture(true); tcRender();
}
function tcRender() { var b = $('tcAuto'); if (!b) return; b.textContent = t(TC.timer ? 'tcAutoStop' : 'tcAuto'); b.classList.toggle('danger', !!TC.timer); }
function rcCsv() {
  var nm = A.servos.map(function (s) { return s.joint.replace('_joint', ''); }), tt = 0;
  var rows = ['frame,time_ms,spd,' + nm.map(function (n) { return n + '_raw'; }).concat(nm.map(function (n) { return n + '_deg'; })).join(',')];
  RC.kf.forEach(function (f, i) {
    tt += f.spd * 15;
    var deg = f.raw.map(function (v, k) { return v ? (A.servos[k].sign * (v - 7500) * K_DEG).toFixed(2) : ''; });
    rows.push([i + 1, tt, f.spd].concat(f.raw.map(function (v) { return v || 0; }), deg).join(','));
  });
  return rows.join('\r\n') + '\r\n';
}
// ---- serial monitor (hex TX/RX with decode), battery graph, Wake Lock
var CMDN = { 0x01: 'status', 0x02: 'loop-start/MotionId', 0x04: 'SaveMotionId', 0x05: 'stop', 0x07: 'loop-end', 0x18: 'pose', 0x19: 'param', 0x1C: 'ReadFlash', 0x1D: 'WriteMotionData', 0x1E: 'SaveMotionData / LED?', 0x1F: 'motion' };
function pktNote(u8, rx) {
  var c = u8[1], n = CMDN[c] || '?';
  if (c === 0x01 && rx && u8.length === 6) return 'volt ' + ((u8[3] | (u8[4] << 8)) / 216).toFixed(2) + ' V';
  if (c === 0x01 && rx && u8.length === 18) { var d = decodeServoInfo(u8); return d ? 'servo ' + h2(d.pid) + ' act ' + d.enc + ' cmd ' + d.cmd + (d.valid ? '' : ' (n/a)') : 'servo reply (bad checksum)'; }
  if (c === 0x01 && !rx) return u8[3] === 2 ? 'query volt' : u8[3] === 5 ? 'query servo ' + h2(u8[4]) : 'query';
  if (c === 0x18 && rx) return 'pose ' + (u8[2] ? 'ERR' : 'ok');
  if (c === 0x18) return 'pose ' + Math.max(0, ((u8.length - 5) / 3) | 0) + ' servos, spd ' + u8[3];
  if (c === 0x1F && !rx) return u8[2] === 0x08 ? 'STOP motion' : 'play motion ' + h2(u8[3]);
  if (c === 0x05 && !rx) return 'STOP';
  return n;
}
function isPoll(u8) { return u8[1] === 1 && (u8.length === 18 || (u8.length === 7 && u8[3] === 5)); }
function rbLog(dir, u8) {
  if (dir === 'TX') MN.tx++; else MN.rx++;
  if (MN.paused || (MN.hidePoll && isPoll(u8))) return;
  MN.lines.push({ t: Date.now(), d: dir, h: hexStr(u8), n: pktNote(u8, dir === 'RX') }); if (MN.lines.length > 400) MN.lines.splice(0, 100);
  MN.dirty = true; mnDraw();
}
function mnDraw() {
  if (!MN.dirty || MN.timer) return;
  MN.timer = setTimeout(function () {
    MN.timer = 0; MN.dirty = false; var box = $('mnLog'); if (!box) return; var stick = box.scrollTop + box.clientHeight >= box.scrollHeight - 4;
    box.textContent = MN.lines.slice(-150).map(function (l) { var d = new Date(l.t); return d.toTimeString().slice(0, 8) + '.' + ('00' + d.getMilliseconds()).slice(-3) + ' ' + l.d + '  ' + l.h + '   ; ' + l.n; }).join('\n');
    if (stick) box.scrollTop = box.scrollHeight;
  }, 200);
}
function mnSend() {
  var txt = $('mnHex').value.trim(); if (!txt || !RB.writer) return;
  var a = txt.split(/\s+/).map(function (h) { return /^[0-9a-f]{1,2}$/i.test(h) ? parseInt(h, 16) : NaN; });
  if (a.some(isNaN)) { alert(t('mnBad')); return; }
  var pk = $('mnAuto').checked ? mk([a.length + 2].concat(a)) : Uint8Array.from(a);
  if (pk.length > 1 && [0x1C, 0x1D, 0x1E, 0x04].indexOf(pk[1]) >= 0 && !confirm(t('mnFlash'))) return;
  rbSend(pk);
}
function bvPush(v) { BV.push({ t: Date.now(), v: v }); if (BV.length > 2000) BV.splice(0, 200); bvDraw(); }
function bvDraw() {
  var c = $('bvG'), tx = $('bvTxt'); if (!c || !c.getContext) return;
  var g = c.getContext('2d'), w = c.width, h = c.height, cs = getComputedStyle(document.documentElement); g.clearRect(0, 0, w, h);
  if (!BV.length) { if (tx) tx.textContent = ''; return; }
  var vs = BV.map(function (p) { return p.v; }), vmin = Math.min.apply(null, vs), vmax = Math.max.apply(null, vs), lo = Math.min(vmin, 9) - 0.3, hi = Math.max(vmax, 9.6) + 0.3;
  var t0 = BV[0].t, t1 = BV[BV.length - 1].t, X = function (tt) { return t1 > t0 ? 4 + (w - 8) * (tt - t0) / (t1 - t0) : w / 2; }, Y = function (v) { return h - 4 - (h - 8) * (v - lo) / (hi - lo); };
  g.strokeStyle = cs.getPropertyValue('--danger').trim() || '#dc2626'; g.setLineDash([3, 3]); g.beginPath(); g.moveTo(0, Y(9)); g.lineTo(w, Y(9)); g.stroke(); g.setLineDash([]);
  g.strokeStyle = cs.getPropertyValue('--accent').trim() || '#6d28d9'; g.lineWidth = 2; g.beginPath();
  BV.forEach(function (p, i) { if (i) g.lineTo(X(p.t), Y(p.v)); else g.moveTo(X(p.t), Y(p.v)); }); g.stroke(); g.lineWidth = 1;
  g.fillStyle = g.strokeStyle; g.beginPath(); g.arc(X(t1), Y(vs[vs.length - 1]), 3, 0, 6.3); g.fill();
  if (tx) tx.textContent = t('bvTxt', vs[vs.length - 1], vmin, vmax, BV.length, (t1 - t0) / 60000);
}
async function wakeOn() {
  if (!$('mnWake').checked || !navigator.wakeLock || LK.wake) return;
  try { LK.wake = await navigator.wakeLock.request('screen'); LK.wake.addEventListener('release', function () { LK.wake = null; renderRb(); }); } catch (e) { LK.wake = null; } renderRb();
}
function wakeOff() { try { if (LK.wake) LK.wake.release(); } catch (e) {} LK.wake = null; }
document.addEventListener('visibilitychange', function () { if (document.visibilityState === 'visible' && RB.writer) wakeOn(); });
// ---- E-stop: always visible, Esc as well. Stops page-side playback / walking / live control / teach capture and tells the robot to stop its motion.
// It does NOT release the servos (the robot would collapse); both stop forms are sent: official "04 05 00 01" and StopMotion "05 1F 08 01".
function rbStopPackets() { var a = rbSend(hexPkt('04 05 00 01')), b = rbSend(mk([0x05, 0x1F, 0x08, 0x01])); return a && b; }
function estop() {
  $('rbLive').checked = false; $('rbStream').checked = false; teachStopAuto();
  P.playing = false; updatePlayUI(true); if (typeof WK !== 'undefined') WK.on = false; if (SW.on) stopSweep();
  RB.lastF = RB.lastK = null; RB.lastPkt = null; RB.retry = 99;           // no automatic re-send of the last pose after the stop
  var ok = RB.writer ? rbStopPackets() : false;
  if (ok) { setTimeout(rbStopPackets, 60); setTimeout(rbStopPackets, 200); }
  RB.motion = ok ? 'rbMotionStopped' : 'estopNoLink'; renderRb();
  var b = $('estop'); b.classList.remove('hit'); void b.offsetWidth; b.classList.add('hit');
}
$('estop').addEventListener('click', estop);
document.addEventListener('keydown', function (e) { if (e.key === 'Escape') estop(); });
// ---- link: Web Serial (Chrome / Edge) or WebSocket bridge (Safari / iOS, Raspberry Pi, ESP32), auto reconnect
function lkUp(kind) {
  LK.kind = kind; LK.tries = 0; RB.msg = 'rbOn'; RB.err = ''; RB.buf = []; RB.volt = null; RB.motion = ''; ST.out = {};
  clearInterval(RB.bt); RB.bt = setInterval(rbBattery, 10000); setTimeout(rbBattery, 1500);
  wakeOn(); renderRb(); stPoll();
}
function rbLost() {          // the link went away by itself (unplug, reset, Wi-Fi drop): keep trying while the user still wants a connection
  if (!RB.writer && !RB.port) return;
  try { if (RB.writer) RB.writer.releaseLock(); } catch (e) {}
  var port = RB.port; RB.writer = RB.reader = RB.rp = null; RB.port = null; LK.ws = null;
  try { if (port && port.close) Promise.resolve(port.close()).catch(function () {}); } catch (e) {}
  clearInterval(RB.bt); ST.out = {}; teachStopAuto(); wakeOff(); $('rbLive').checked = false; $('rbStream').checked = false;
  RB.msg = LK.want ? 'rbLost' : 'rbOff'; renderRb(); if (LK.want) lkRetry();
}
function lkRetry() {
  clearTimeout(LK.timer); if (!LK.want || RB.writer) return;
  LK.timer = setTimeout(async function () {
    if (!LK.want || RB.writer) return;
    try {
      if (LK.kind === 'ws') await wsOpen(LK.wsUrl);
      else {
        var ports = await navigator.serial.getPorts(), inf = LK.info || {};
        var pick = ports.filter(function (p) { var i = p.getInfo ? p.getInfo() : {}; return i.usbVendorId === inf.usbVendorId && i.usbProductId === inf.usbProductId; })[0] || (ports.length === 1 ? ports[0] : null);
        if (!pick) throw new Error('no port'); await rbOpenSerial(pick);
      }
    } catch (e) { LK.tries++; lkRetry(); }
  }, Math.min(5000, Math.round(1000 * Math.pow(1.5, LK.tries))));
}
function rbLoop() {
  return (async function () {
    var port = RB.port;
    try { while (RB.port === port && !RB.closing && port.readable) { RB.reader = port.readable.getReader(); try { for (;;) { var r = await RB.reader.read(); if (r.done) break; rbRx(r.value); } } finally { RB.reader.releaseLock(); RB.reader = null; } } } catch (e) { console.warn(e); }
    if (RB.port === port && !RB.closing) rbLost();
  })();
}
async function rbOpenSerial(port) {
  await port.open({ baudRate: 115200, dataBits: 8, stopBits: 1, parity: 'none' });
  RB.closing = false; RB.port = port; RB.writer = port.writable.getWriter(); RB.rp = rbLoop(); lkUp('serial');
}
async function rbConnect() {
  if (RB.writer) return;
  if (!navigator.serial) { RB.msg = 'rbNoSerial'; renderRb(); return; }
  try { var port = await navigator.serial.requestPort(); LK.info = port.getInfo ? port.getInfo() : {}; LK.want = true; await rbOpenSerial(port); }
  catch (e) { LK.want = false; RB.port = null; RB.writer = null; RB.msg = 'rbErr'; RB.err = e && e.message || String(e); }
  renderRb();
}
function wsOpen(url) {
  return new Promise(function (res, rej) {
    var ws, opened = false; try { ws = new WebSocket(url); } catch (e) { rej(e); return; }
    ws.binaryType = 'arraybuffer';
    ws.onopen = function () {
      opened = true; LK.ws = ws; RB.closing = false; RB.port = ws;
      RB.writer = { write: function (u) { if (ws.readyState !== 1) return Promise.reject(new Error('WebSocket closed')); ws.send(u); return Promise.resolve(); }, releaseLock: function () {} };
      lkUp('ws'); res();
    };
    ws.onmessage = function (ev) { if (ev.data instanceof ArrayBuffer) rbRx(new Uint8Array(ev.data)); };
    ws.onclose = function () { if (!opened) { rej(new Error('WebSocket connection failed')); return; } if (RB.port === ws && !RB.closing) rbLost(); };
    ws.onerror = function () {};
  });
}
async function wsConnect() {
  if (RB.writer) return; var url = $('wsUrl').value.trim(); if (!url) return;
  if (location.protocol === 'https:' && /^ws:\/\//i.test(url) && !/^ws:\/\/(localhost|127\.0\.0\.1|\[::1\])([:\/]|$)/i.test(url)) { RB.msg = 'rbWsMixed'; renderRb(); return; }
  try { localStorage.setItem('pmsim.ws', url); } catch (e) {}
  LK.wsUrl = url; LK.kind = 'ws'; RB.msg = 'rbWsTry'; renderRb();
  try { LK.want = true; await wsOpen(url); } catch (e) { LK.want = false; RB.msg = 'rbErr'; RB.err = e && e.message || String(e); }
  renderRb();
}
async function rbDisconnect() {
  LK.want = false; clearTimeout(LK.timer); LK.timer = 0; RB.closing = true;
  clearInterval(RB.bt); $('rbLive').checked = false; $('rbStream').checked = false; teachStopAuto(); wakeOff();
  try { if (RB.reader) await RB.reader.cancel(); if (RB.rp) await RB.rp; } catch (e) {}
  try { if (RB.writer) RB.writer.releaseLock(); } catch (e) {}
  try { if (RB.port) await RB.port.close(); } catch (e) {}
  RB.port = RB.writer = RB.reader = RB.rp = null; LK.ws = null; RB.msg = 'rbOff'; RB.motion = ''; ST.out = {}; renderRb();
}
if (navigator.serial && navigator.serial.addEventListener) {
  navigator.serial.addEventListener('disconnect', function (e) { if (e.target === RB.port) rbLost(); });
  navigator.serial.addEventListener('connect', function () { if (LK.want && !RB.writer && LK.kind === 'serial') { LK.tries = 0; lkRetry(); } });
}
(function p1Init() {
  var u = ''; try { u = localStorage.getItem('pmsim.ws') || ''; } catch (e) {}
  $('wsUrl').value = u || (location.protocol === 'http:' || location.protocol === 'https:' ? (location.protocol === 'https:' ? 'wss://' : 'ws://') + location.host + '/ws' : 'ws://raspberrypi.local:8765/ws');
  $('wsConn').addEventListener('click', wsConnect);
  $('stOn').addEventListener('change', function () { ST.on = this.checked; if (!ST.on) { ST.twin = false; $('stTwin').checked = false; teachStopAuto(); } else stPoll(); renderRb(); });
  $('stTwin').addEventListener('change', function () {
    ST.twin = this.checked;
    if (ST.twin) { ST.on = true; $('stOn').checked = true; $('rbLive').checked = false; $('rbStream').checked = false; touch(); if (typeof WK !== 'undefined') WK.on = false; MIR.sent = {}; MIR.out = {}; stPoll(); }
    renderRb();
  });
  $('mrMode').addEventListener('change', function () { MIR.mode = this.value; MIR.sent = {}; MIR.out = {}; });
  $('tcStart').addEventListener('click', teachStart); $('tcHold').addEventListener('click', teachHold);
  $('tcCap').addEventListener('click', function () { teachCapture(false); }); $('tcAuto').addEventListener('click', teachAuto);
  $('rcCsv').addEventListener('click', function () {
    if (!RC.kf.length) return; var a = document.createElement('a'); a.href = URL.createObjectURL(new Blob(['\uFEFF' + rcCsv()], { type: 'text/csv' })); a.download = rcName().replace(/[\\/:*?"<>|]+/g, '_') + '.csv'; a.click(); setTimeout(function () { URL.revokeObjectURL(a.href); }, 1000);
  });
  $('mnPause').addEventListener('change', function () { MN.paused = this.checked; }); $('mnHide').addEventListener('change', function () { MN.hidePoll = this.checked; });
  $('mnClear').addEventListener('click', function () { MN.lines = []; MN.dirty = true; mnDraw(); });
  $('mnCopy').addEventListener('click', function () { var s = MN.lines.map(function (l) { return l.d + ' ' + l.h; }).join('\n'); try { navigator.clipboard.writeText(s); } catch (e) {} });
  $('mnSend').addEventListener('click', mnSend); $('mnHex').addEventListener('keydown', function (e) { if (e.key === 'Enter') mnSend(); });
  $('mnWake').addEventListener('change', function () { if (this.checked) wakeOn(); else wakeOff(); renderRb(); });
  $('ledOk').addEventListener('change', renderRb);
  $('ledSend').addEventListener('click', function () {
    if (!RB.writer || !$('ledOk').checked) return;
    var a = $('ledHex').value.trim().split(/\s+/).filter(Boolean).map(function (h) { return /^[0-9a-f]{1,2}$/i.test(h) ? parseInt(h, 16) : NaN; });
    if (!a.length || a.some(isNaN)) { alert(t('mnBad')); return; }
    var pk = mk([a.length + 3, 0x1E].concat(a)); if (!confirm(t('ledConfirm', hexStr(pk)))) return; rbSend(pk);
  });
})();
function rbRender2(on) {
  $('wsConn').disabled = on || RB.msg === 'rbWsTry';
  if (ST.twin && ($('rbLive').checked || $('rbStream').checked)) { ST.twin = false; $('stTwin').checked = false; }
  $('ledSend').disabled = !on || !$('ledOk').checked;
  ['tcStart', 'tcHold', 'tcCap', 'tcAuto'].forEach(function (id) { $(id).disabled = !on; });
  var now = performance.now();
  if (now - ST.sumT > 250) { ST.sumT = now; $('stSum').textContent = ST.on ? t('stSum', ST.ok, ST.lat, ST.cyc, ST.miss, ST.bad) : ''; }
  $('mnWakeS').textContent = !navigator.wakeLock ? t('wakeNo') : LK.wake ? t('wakeOn') : '';
  $('mnCnt').textContent = t('mnCnt', MN.tx, MN.rx); tcRender();
}

(function () {      // panels, tabs, settings card, robot control mode
  var h = document.documentElement;
  function tab(n) { ['motion', 'robot', 'walk', 'rec'].forEach(function (x) { $('pane_' + x).hidden = x !== n; }); Array.prototype.forEach.call(document.querySelectorAll('[data-tab]'), function (b) { b.classList.toggle('on', b.getAttribute('data-tab') === n); }); h.setAttribute('data-tab', n); }
  Array.prototype.forEach.call(document.querySelectorAll('[data-tab]'), function (b) { b.addEventListener('click', function () { tab(b.getAttribute('data-tab')); }); });
  function drawer(side, n) { var cls = side === 'l' ? 'mobL' : 'mobR', open = h.classList.contains(cls) && (side === 'r' || h.getAttribute('data-tab') === n); h.classList.remove('mobL', 'mobR'); if (!open) { h.classList.add(cls); if (n) tab(n); } }
  $('mobL').addEventListener('click', function () { drawer('l', 'motion'); });
  $('mobRb').addEventListener('click', function () { drawer('l', 'robot'); });
  $('mobR').addEventListener('click', function () { drawer('r'); });
  $('bLang2').addEventListener('click', function () { $('bLang').click(); });
  function rtab(n) { ['servo', 'set'].forEach(function (x) { $('rpane_' + x).hidden = x !== n; }); Array.prototype.forEach.call(document.querySelectorAll('[data-rtab]'), function (b) { b.classList.toggle('on', b.getAttribute('data-rtab') === n); }); $('bSet').classList.toggle('on', n === 'set'); }
  Array.prototype.forEach.call(document.querySelectorAll('[data-rtab]'), function (b) { b.addEventListener('click', function () { rtab(b.getAttribute('data-rtab')); }); });
  $('bSet').addEventListener('click', function () { var toSet = $('rpane_set').hidden; rtab(toSet ? 'set' : 'servo'); if (window.matchMedia('(max-width: 1100px)').matches) { h.classList.remove('mobL'); h.classList.add('mobR'); } });
  Array.prototype.forEach.call(document.querySelectorAll('[data-rbmode]'), function (b) { b.addEventListener('click', function () { var m = b.getAttribute('data-rbmode'); $('rbLive').checked = m === 'live'; $('rbStream').checked = m === 'stream'; renderRb(); }); });
  ['rbRel', 'rbT'].forEach(function (id) { $(id).addEventListener('click', renderRb); });
  $('rbSp').addEventListener('input', function () { $('rbSpV').textContent = this.value; });
  tab('motion');
})();
// ------------------------------------------------------------------ walking generator
// Gait idea and constants: @kazzlog, github.com/kazzlog/pm_sample (PreMaidMathWalkSample.cs, Apache-2.0), re-implemented here.
// One call of step() = one pose every SPD x 15 ms; values are raw servo pulses around 7500 for the 17 servos of WALK.IDS.
var WALK = (function () {
  var SIN1 = [0, 0.59, 0.95, 0.95, 0.59, 0, -0.59, -0.95, -0.95, -0.59], SIN3 = [0, 0.2, 0.86, 0.86, 0.2, 0, -0.2, -0.86, -0.86, -0.2],
      P1 = [0.7, 0.9, 0.6, -0.4, -0.9, -0.8, -0.5, -0.2, 0.1, 0.4], P2 = [-0.8, -0.5, -0.2, 0.1, 0.4, 0.7, 0.9, 0.6, -0.4, -0.9],
      LEN = [2, 4, 2, -1, -2, -4, -2, 1, 0, 0, 0, 0], ROLL = [0, 0, 0, 0, 0, 0, 0, 0, 2, 2, -2, -2], PIT = [-1, 0, 1, 1, 1, 0, -1, -1, 0, 0, 0, 0],
      IDS = [0x10, 0x14, 0x18, 0x02, 0x0e, 0x12, 0x16, 0x04, 0x0a, 0x0c, 0x1a, 0x1c, 0x08, 0x06, 0x05, 0x07, 0x03], tr = Math.trunc;
  function make() {
    var st = { cond: 0, tick: 0, steer: 0 };      // cond: 0 idle, 1 first half step, 2 walking, 3 last half step
    function step(o) {      // o: {go, X1, Y1, X2, Y2, len, roll, pitch}
      var V = [], i, X1 = o.X1, Y1 = o.Y1, X2 = o.X2, Y2 = o.Y2, DL = 140 * o.len, DR = 80 * o.roll, DP = 600 * o.pitch;
      for (i = 0; i < 17; i++) V[i] = 7500;
      if (Math.abs(X2) > 0.1) {      // waist yaw
        V[12] += tr(600 * X2); V[13] += tr(600 * X2); V[14] += tr(600 * X2);
        for (i = 0; i < 3; i++) { V[i] += tr(140 * X2 * PIT[i]); V[i + 4] -= tr(140 * X2 * PIT[i + 4]); }
      }
      if (Math.abs(Y2) > 0.1) {      // lean / crouch
        V[15] += tr(300 * X2); V[16] -= tr(300 * Y2);
        if (Y2 < 0) { for (i = 0; i < 8; i++) V[i] -= tr(400 * Y2 * LEN[i]); }
        else { V[0] += tr(150 * Y2); V[4] -= tr(150 * Y2); V[3] -= tr(450 * Y2); V[7] += tr(450 * Y2); }
      }
      if (st.cond === 0 && o.go) { st.cond = 1; st.tick = 0; }
      if (st.cond > 0) {
        if (++st.tick >= 10) { st.tick = 0; if (st.cond === 1) st.cond = 2; if (!o.go) st.cond = 3; }
        var rl = SIN3[st.tick], rr = SIN1[st.tick], p1 = P1[st.tick], p2 = P2[st.tick], ratio = 2;
        if (st.cond === 1) { if (st.tick < 5) ratio = 1; else st.cond = 2; }
        if (st.cond === 3) { if (st.tick < 5) ratio = 1; else st.cond = 0; }
        for (i = 8; i < 12; i++) V[i] += tr(rr * DR * ROLL[i] * ratio);
        var tmp = 0; if (rl < 0) { tmp = 4; rl = -rl; }
        for (i = tmp; i < tmp + 4; i++) V[i] += tr(rl * DL * LEN[i] * ratio);
        if (st.cond === 2) {
          for (i = 0; i < 4; i++) { V[i] += tr(p1 * DP * PIT[i] * Y1); V[i + 4] += tr(p2 * DP * PIT[i + 4] * Y1); }
          if (st.steer !== 0 || Math.abs(X1) > 0.1) {      // steering is spread over the swing phases
            var sg = -tr(X1 * 250), tk = st.tick;
            if (tk === 2 || tk === 3 || tk === 4) { if (sg > 0) st.steer += sg; else st.steer = (tk === 4) ? 0 : tr(st.steer / 2); }
            if (tk === 7 || tk === 8 || tk === 9) { if (sg < 0) st.steer -= sg; else st.steer = (tk === 9) ? 0 : tr(st.steer / 2); }
          }
          V[12] += st.steer; V[13] -= st.steer;
        }
      }
      return V;
    }
    return { st: st, step: step };
  }
  return { make: make, IDS: IDS };
})();
var WK = { gen: WALK.make(), on: false, go: false, acc: 0, T: 0.135, prev: null, cur: null, ids: WALK.IDS.map(function (pid) { return PM.pid2k[pid]; }), A: { x: 0, y: 0 }, B: { x: 0, y: 0 }, keys: {} };
function wVal(id) { return +$(id).value; }
function wDeg(V) { return V.map(function (v, i) { var k = WK.ids[i]; return clampDeg(k, A.servos[k].sign * (v - 7500) * K_DEG); }); }
function wAxis(neg, pos) { return (WK.keys[pos] ? 1 : 0) - (WK.keys[neg] ? 1 : 0); }
function walkPose() {      // next gait pose (one every SPD x 15 ms)
  var cl = function (v) { return Math.max(-1, Math.min(1, v)); };
  WK.prev = WK.cur;
  WK.cur = wDeg(WK.gen.step({ go: WK.go, X1: cl(WK.A.x + wAxis('ArrowLeft', 'ArrowRight') + WK.gp.x1), Y1: cl(WK.A.y + wAxis('ArrowDown', 'ArrowUp') + WK.gp.y1), X2: cl(WK.B.x + wAxis('a', 'd') + WK.gp.x2), Y2: cl(WK.B.y + wAxis('s', 'w') + WK.gp.y2), len: wVal('wLen'), roll: wVal('wRoll'), pitch: wVal('wPit') }));
  if ($('wRobot').checked && RB.writer) {      // the robot interpolates between poses itself (SPD ticks), like the sample program
    var r = targetsRaw(); WK.ids.forEach(function (k, i) { r[k] = rawOf(k, WK.cur[i]); });
    RB.retry = 0; rbSend(poseBytes(r, Math.round(WK.T / 0.015)), true);
  }
  if (!WK.go && WK.gen.st.cond === 0) { WK.on = false; wRender(); }
}
function walkTick(dt) {
  if (P.playing) { WK.on = false; wRender(); return; }
  WK.acc += dt;
  while (WK.on && WK.acc >= WK.T) { WK.acc -= WK.T; walkPose(); }
  var a = Math.min(1, WK.acc / WK.T); if (!WK.on) a = 1;
  WK.ids.forEach(function (k, i) { setTarget(k, WK.prev[i] + (WK.cur[i] - WK.prev[i]) * a); });
  wTick();
}
function walkStart() {
  pausePlay(); applyPose(PRESETS.zero); WK.gen = WALK.make(); WK.T = wVal('wSpd') * 0.015;
  WK.cur = WK.ids.map(function (k) { return target[k]; }); WK.prev = WK.cur; WK.go = true; WK.on = true; WK.acc = WK.T; wRender();
}
function wTick() { var s = WK.gen.st; $('wTick').textContent = WK.on ? 'cond ' + s.cond + ' · tick ' + s.tick : ''; $('wState').textContent = t(WK.on ? ['walkIdle', 'walkStarting', 'walkWalking', 'walkStopping'][s.cond] : 'walkIdle'); }
function wRender() { $('wGo').textContent = t(WK.on && WK.go ? 'walkStop' : 'walkStart'); $('wGo').classList.toggle('danger', WK.on && WK.go); $('wGo').classList.toggle('primary', !(WK.on && WK.go)); wTick(); }
function makePad(el, v) {
  var knob = el.firstElementChild;
  function draw() { knob.style.left = (50 + v.x * 38) + '%'; knob.style.top = (50 - v.y * 38) + '%'; }
  function set(e) { var r = el.getBoundingClientRect(); v.x = Math.max(-1, Math.min(1, ((e.clientX - r.left) / r.width - 0.5) / 0.38)); v.y = Math.max(-1, Math.min(1, -((e.clientY - r.top) / r.height - 0.5) / 0.38)); draw(); }
  function up() { el.dragging = false; if (!$('wHold').checked) { v.x = 0; v.y = 0; draw(); } }
  el.addEventListener('pointerdown', function (e) { el.setPointerCapture(e.pointerId); el.dragging = true; set(e); });
  el.addEventListener('pointermove', function (e) { if (el.dragging) set(e); });
  el.addEventListener('pointerup', up); el.addEventListener('pointercancel', up); draw();
  return draw;
}
(function () {
  var dA = makePad($('padA'), WK.A), dB = makePad($('padB'), WK.B);
  $('wHold').addEventListener('change', function () { if (!this.checked) { WK.A.x = WK.A.y = WK.B.x = WK.B.y = 0; dA(); dB(); } });
  $('wGo').addEventListener('click', function () { if (!WK.on) walkStart(); else if (WK.go) { WK.go = false; wRender(); } });
  [['wLen', 'wLenV', function (v) { return (+v).toFixed(2); }], ['wRoll', 'wRollV', function (v) { return (+v).toFixed(2); }], ['wPit', 'wPitV', function (v) { return (+v).toFixed(2); }], ['wSpd', 'wSpdV', function (v) { return Math.round(v * 15) + ' ms'; }]].forEach(function (a) {
    $(a[0]).addEventListener('input', function () { $(a[1]).textContent = a[2](this.value); if (a[0] === 'wSpd') WK.T = this.value * 0.015; });
  });
  document.addEventListener('keydown', function (e) { if (!WK.on || /INPUT|SELECT|TEXTAREA/.test(e.target.tagName) && e.target.type !== 'range') return; var k = e.key.length === 1 ? e.key.toLowerCase() : e.key; if ('ArrowUpArrowDownArrowLeftArrowRightwasd'.indexOf(k) >= 0) { WK.keys[k] = true; e.preventDefault(); } });
  document.addEventListener('keyup', function (e) { var k = e.key.length === 1 ? e.key.toLowerCase() : e.key; delete WK.keys[k]; });
  wRender();
})();

// ------------------------------------------------------------------ IK drag (hands / feet), damped least squares on the 4 limb chains
// Forward-kinematics data (A.chains) is exported by mujoco/make_premaid_mjcf.py; the chains hang from base_link, so the target follows the body.
var IK = { on: false, h: {}, drag: null, ray: null, plane: null };
function qmat(q) { var w = q[0], x = q[1], y = q[2], z = q[3]; return [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y), 2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x), 2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]; }
function mmul(a, b) { var r = [], i, j; for (i = 0; i < 3; i++) for (j = 0; j < 3; j++) r.push(a[3 * i] * b[j] + a[3 * i + 1] * b[3 + j] + a[3 * i + 2] * b[6 + j]); return r; }
function mvec(a, v) { return [a[0] * v[0] + a[1] * v[1] + a[2] * v[2], a[3] * v[0] + a[4] * v[1] + a[5] * v[2], a[6] * v[0] + a[7] * v[1] + a[8] * v[2]]; }
function mtvec(a, v) { return [a[0] * v[0] + a[3] * v[1] + a[6] * v[2], a[1] * v[0] + a[4] * v[1] + a[7] * v[2], a[2] * v[0] + a[5] * v[1] + a[8] * v[2]]; }
function mT(a) { return [a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]]; }
function axisRot(ax, th) { var c = Math.cos(th), s = Math.sin(th), n = Math.sqrt(ax[0] * ax[0] + ax[1] * ax[1] + ax[2] * ax[2]), x = ax[0] / n, y = ax[1] / n, z = ax[2] / n, u = 1 - c;
  return [c + x * x * u, x * y * u - z * s, x * z * u + y * s, y * x * u + z * s, c + y * y * u, y * z * u - x * s, z * x * u - y * s, z * y * u + x * s, c + z * z * u]; }
function ikFK(ch, q) {      // joint angles (rad, one per chain joint) -> tip position and orientation in the base_link frame
  var R = [1, 0, 0, 0, 1, 0, 0, 0, 1], p = [0, 0, 0], i, j, o;
  for (i = 0; i < ch.joints.length; i++) { j = ch.joints[i]; o = mvec(R, j.pos); p = [p[0] + o[0], p[1] + o[1], p[2] + o[2]]; R = mmul(mmul(R, j.R0 || (j.R0 = qmat(j.quat))), axisRot(j.axis, q[i])); }
  o = mvec(R, ch.tip); return { p: [p[0] + o[0], p[1] + o[1], p[2] + o[2]], R: R };
}
function rotVec(Ra, Rb) { var E = mmul(Ra, mT(Rb)); return [(E[7] - E[5]) / 2, (E[2] - E[6]) / 2, (E[3] - E[1]) / 2]; }      // small rotation taking Rb to Ra
function solveLin(M, b) {      // Gaussian elimination, M is n x n (array of rows)
  var n = b.length, i, j, k, p, t, f, x = [];
  for (i = 0; i < n; i++) { p = i; for (j = i + 1; j < n; j++) if (Math.abs(M[j][i]) > Math.abs(M[p][i])) p = j; t = M[i]; M[i] = M[p]; M[p] = t; t = b[i]; b[i] = b[p]; b[p] = t;
    for (j = i + 1; j < n; j++) { f = M[j][i] / M[i][i]; for (k = i; k < n; k++) M[j][k] -= f * M[i][k]; b[j] -= f * b[i]; } }
  for (i = n - 1; i >= 0; i--) { t = b[i]; for (j = i + 1; j < n; j++) t -= M[i][j] * x[j]; x[i] = t / M[i][i]; }
  return x;
}
function ikSolve(ch, q0, pt, Rt) {      // returns {q, err(m)}; Rt (optional) keeps the end orientation
  var n = ch.joints.length, q = q0.slice(), m = Rt ? 6 : 3, lam2 = 0.0009, it, i, r, c, f, f2, e, J, JJt, y, dq, step, pe = 0;
  for (it = 0; it < 40; it++) {
    f = ikFK(ch, q); e = [pt[0] - f.p[0], pt[1] - f.p[1], pt[2] - f.p[2]]; pe = Math.sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
    if (Rt) { r = rotVec(Rt, f.R); e.push(0.4 * r[0], 0.4 * r[1], 0.4 * r[2]); }
    if (pe < 3e-4 && (!Rt || Math.abs(e[3]) + Math.abs(e[4]) + Math.abs(e[5]) < 2e-3)) break;
    step = Math.min(1, 0.02 / Math.max(pe, 1e-9)); for (r = 0; r < m; r++) e[r] *= step;
    J = []; for (r = 0; r < m; r++) J.push([]);
    for (c = 0; c < n; c++) {
      q[c] += 1e-4; f2 = ikFK(ch, q); q[c] -= 1e-4;
      for (r = 0; r < 3; r++) J[r].push((f2.p[r] - f.p[r]) / 1e-4);
      if (Rt) { y = rotVec(f2.R, f.R); for (r = 0; r < 3; r++) J[3 + r].push(0.4 * y[r] / 1e-4); }
    }
    JJt = []; for (r = 0; r < m; r++) { JJt.push([]); for (c = 0; c < m; c++) { y = r === c ? lam2 : 0; for (i = 0; i < n; i++) y += J[r][i] * J[c][i]; JJt[r].push(y); } }
    y = solveLin(JJt, e.slice(0, m));
    for (i = 0; i < n; i++) { dq = 0; for (r = 0; r < m; r++) dq += J[r][i] * y[r]; q[i] = Math.min(A.servos[ch.joints[i].k].hi, Math.max(A.servos[ch.joints[i].k].lo, q[i] + dq)); }
  }
  f = ikFK(ch, q); return { q: q, err: Math.sqrt(Math.pow(pt[0] - f.p[0], 2) + Math.pow(pt[1] - f.p[1], 2) + Math.pow(pt[2] - f.p[2], 2)) };
}
function ikChainQ(ch) { return ch.joints.map(function (j) { return target[j.k] * D2R; }); }
function ikBase() { var b = S.base, p = S.d.xpos, q = S.d.xquat; return { p: [p[3 * b], p[3 * b + 1], p[3 * b + 2]], R: qmat([q[4 * b], q[4 * b + 1], q[4 * b + 2], q[4 * b + 3]]) }; }
function ikWorld(B, pb) { var o = mvec(B.R, pb); return [B.p[0] + o[0], B.p[1] + o[1], B.p[2] + o[2]]; }      // base frame -> MuJoCo world
function ikEnsure() {
  if (IK.ray || !S.ready || !A.chains) return IK.ray !== null; IK.ray = new THREE.Raycaster(); IK.plane = new THREE.Plane();
  Object.keys(A.chains).forEach(function (n) {
    var m = new THREE.Mesh(new THREE.SphereGeometry(0.011, 20, 14), new THREE.MeshBasicMaterial({ color: /leg/.test(n) ? 0x22c55e : 0x06b6d4, transparent: true, opacity: 0.85, depthTest: false })); m.renderOrder = 10; m.visible = false; m.userData.chain = n; world.add(m); IK.h[n] = m;
  }); return true;
}
function ikTick() {      // keep the dots on the commanded tip positions
  if (!IK.on || !ikEnsure()) return; var B = ikBase();
  Object.keys(A.chains).forEach(function (n) { var m = IK.h[n], w = ikWorld(B, ikFK(A.chains[n], ikChainQ(A.chains[n])).p); m.visible = true; m.position.set(w[0], w[1], w[2]); m.scale.setScalar(IK.drag && IK.drag.n === n ? 1.35 : 1); });
}
function ikHit(e) {
  var r = canvas.getBoundingClientRect(); IK.ray.setFromCamera({ x: ((e.clientX - r.left) / r.width) * 2 - 1, y: -((e.clientY - r.top) / r.height) * 2 + 1 }, camera);
  var hs = Object.keys(IK.h).map(function (n) { return IK.h[n]; }).filter(function (m) { return m.visible; }), hit = IK.ray.intersectObjects(hs);
  return hit.length ? hit[0] : null;
}
function ikMove(e) {
  var d = IK.drag, r = canvas.getBoundingClientRect(), hit = new THREE.Vector3();
  IK.ray.setFromCamera({ x: ((e.clientX - r.left) / r.width) * 2 - 1, y: -((e.clientY - r.top) / r.height) * 2 + 1 }, camera);
  if (!IK.ray.ray.intersectPlane(IK.plane, hit)) return;
  var w = world.worldToLocal(hit).add(d.off), B = ikBase(), pb = mtvec(B.R, [w.x - B.p[0], w.y - B.p[1], w.z - B.p[2]]), ch = A.chains[d.n];
  var res = ikSolve(ch, ikChainQ(ch), pb, d.Rt); ch.joints.forEach(function (j, i) { setTarget(j.k, res.q[i] / D2R); });
}
canvas.addEventListener('pointerdown', function (e) {
  if (!IK.on || !ikEnsure()) return; var h = ikHit(e); if (!h) return;
  e.stopImmediatePropagation(); e.preventDefault(); canvas.setPointerCapture(e.pointerId); touch(); if (typeof WK !== 'undefined') WK.on = false;
  var n = h.object.userData.chain, ch = A.chains[n], nrm = new THREE.Vector3(); camera.getWorldDirection(nrm); IK.plane.setFromNormalAndCoplanarPoint(nrm, h.object.getWorldPosition(new THREE.Vector3()));
  IK.drag = { n: n, Rt: ch.ori ? ikFK(ch, ikChainQ(ch)).R : null, off: h.object.position.clone().sub(world.worldToLocal(h.point.clone())) }; canvas.style.cursor = 'grabbing';
}, true);
canvas.addEventListener('pointermove', function (e) {
  if (!IK.on || !IK.ray) return; if (IK.drag) { e.stopImmediatePropagation(); ikMove(e); return; } canvas.style.cursor = ikHit(e) ? 'grab' : '';
}, true);
['pointerup', 'pointercancel'].forEach(function (ev) { canvas.addEventListener(ev, function (e) { if (IK.drag) { IK.drag = null; canvas.style.cursor = ''; } }, true); });
$('cIK').addEventListener('change', function () { IK.on = this.checked; if (!IK.on) { Object.keys(IK.h).forEach(function (n) { IK.h[n].visible = false; }); IK.drag = null; } });

// ------------------------------------------------------------------ keyframe recorder -> .pma
// File layout copied from the reference motions: [07 02 00 41 00 01 45] [36 19 10 .. stretch] {0x18 pose frames} [07 02 00 41 00 0A 4E] [04 17 00 13] FF FF
var RC = { kf: [] }, RC_PRE = '07 02 00 41 00 01 45', RC_STRETCH = '36 19 10 10 3C 18 3C 1C 3C 14 3C 0C 3C 08 28 0E 3C 15 1E 13 1E 16 3C 1A 3C 12 3C 0A 3C 06 28 03 14 11 1E 17 1E 0D 1E 0F 1E 09 1E 0B 1E 04 1E 02 1E 05 14 07 14 34', RC_POST = '07 02 00 41 00 0A 4E 04 17 00 13 FF FF';
function rcRaw() { var r = []; for (var k = 0; k < N; k++) { var s = A.servos[k], e = PM.byJoint[s.joint]; r[k] = Math.max(e.rawLo, Math.min(e.rawHi, Math.round(7500 + s.sign * target[k] / K_DEG))); } return r; }
function rcBytes() {
  var parts = [hexPkt(RC_PRE), hexPkt(RC_STRETCH)].concat(RC.kf.map(function (f) { return poseBytes(f.raw, f.spd); }), [hexPkt(RC_POST)]), n = 0, o, i = 0;
  parts.forEach(function (p) { n += p.length; }); o = new Uint8Array(n); parts.forEach(function (p) { o.set(p, i); i += p.length; }); return o;
}
function rcText(name, b) {
  return '\uFEFF[モーション(1)]\n番号=1\n名前=' + name.replace(/[\r\n]+/g, ' ') + '\nデータ=' + Array.prototype.map.call(b, function (x) { return ('0' + x.toString(16).toUpperCase()).slice(-2); }).join(' ') + '\n';
}
function rcName() { return ($('rcName').value || 'My Motion').trim() || 'My Motion'; }
function rcSave() { try { localStorage.setItem('premaid-rc', JSON.stringify({ name: $('rcName').value, kf: RC.kf })); } catch (e) {} }
function rcRender() {
  var box = $('rcList'); box.innerHTML = ''; $('rcName').placeholder = t('recName');
  if (!RC.kf.length) { var d = document.createElement('div'); d.className = 'sub'; d.textContent = t('recEmpty'); box.appendChild(d); }
  RC.kf.forEach(function (f, i) {
    var row = document.createElement('div'); row.className = 'rcrow';
    function btn(txt, tip, fn) { var b = document.createElement('button'); b.className = 'sm'; b.textContent = txt; b.title = tip; b.addEventListener('click', fn); row.appendChild(b); return b; }
    var nb = btn('#' + (i + 1), t('recGo'), function () { rcGo(i); rcSelect(i); }); if (i === RC.sel) row.classList.add('sel');
    var sp = document.createElement('input'); sp.type = 'number'; sp.min = 1; sp.max = 255; sp.value = f.spd; sp.title = t('recSpdRow') + ' (× 15 ms)';
    sp.addEventListener('change', function () { f.spd = Math.max(1, Math.min(255, parseInt(sp.value, 10) || 10)); sp.value = f.spd; rcSave(); rcInfo(); }); row.appendChild(sp);
    btn('⟳', t('recSet'), function () { f.raw = rcRaw(); rcSave(); });
    btn('▲', t('recUp'), function () { if (i > 0) { RC.kf.splice(i - 1, 0, RC.kf.splice(i, 1)[0]); rcSave(); rcRender(); } });
    btn('▼', t('recDn'), function () { if (i < RC.kf.length - 1) { RC.kf.splice(i + 1, 0, RC.kf.splice(i, 1)[0]); rcSave(); rcRender(); } });
    btn('✕', t('recDel'), function () { RC.kf.splice(i, 1); rcSave(); rcRender(); });
    box.appendChild(row);
  });
  rcInfo(); $('rcPlay').disabled = $('rcExp').disabled = $('rcClr').disabled = $('rcCsv').disabled = !RC.kf.length; $('rcAlt').textContent = RC.alt ? t('rcAltTxt', RC.alt.kf.length) : ''; if (typeof onRefresh === 'function') onRefresh();
}
function rcInfo() { var tl = RC.kf.length ? buildTimeline(rcBytes()) : null; $('rcInfo').textContent = tl ? t('recInfo', RC.kf.length, tl.total / 1000) : ''; renderCheck('chkR', tl); }
function rcGo(i) { touch(); if (typeof WK !== 'undefined') WK.on = false; var f = RC.kf[i]; for (var k = 0; k < N; k++) if (f.raw[k]) setTarget(k, A.servos[k].sign * (f.raw[k] - 7500) * K_DEG); }
function rcPreview() {
  if (!RC.kf.length) return; var nm = rcName(); markActive(-1); LIB.cur = -1; audioSet('', '', '');
  loadSections([{ name: nm, bytes: rcBytes() }], nm + '.pma'); $('pPlay').click();
}
function rcExport() {
  if (!RC.kf.length) return; var nm = rcName(), a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([rcText(nm, rcBytes())], { type: 'text/plain' })); a.download = nm.replace(/[\\/:*?"<>|]+/g, '_') + '.pma'; a.click(); setTimeout(function () { URL.revokeObjectURL(a.href); }, 1000);
}
$('rcAdd').addEventListener('click', function () { RC.kf.push({ raw: rcRaw(), spd: +$('rcSpd').value }); rcSave(); rcRender(); $('rcList').scrollTop = 1e6; });
$('rcSpd').addEventListener('input', function () { $('rcSpdV').textContent = this.value * 15 + ' ms'; });
$('rcPlay').addEventListener('click', rcPreview); $('rcExp').addEventListener('click', rcExport);
$('rcClr').addEventListener('click', function () { if (confirm(t('recClr') + '?')) { RC.kf = []; rcSave(); rcRender(); } });
$('rcName').addEventListener('input', rcSave);
function rcClampRaw(k, v) { var e = PM.byJoint[A.servos[k].joint]; return Math.max(e.rawLo, Math.min(e.rawHi, v)); }
function mirrorRaw(raw) {      // A.mirror: {k, p, m}: servo k takes the joint angle of servo p times m (0 = servo released stays released)
  var o = []; A.mirror.forEach(function (e) { var v = raw[e.p]; o[e.k] = v === 0 ? 0 : rcClampRaw(e.k, Math.round(7500 + A.servos[e.k].sign * A.servos[e.p].sign * e.m * (v - 7500))); }); return o;
}
$('rcMir').addEventListener('click', function () { RC.kf.forEach(function (f) { f.raw = mirrorRaw(f.raw); }); rcSave(); rcRender(); });
$('rcImp').addEventListener('click', function () {
  if (!P.frames || !P.frames.length) { alert(t('recNoMotion')); return; } if (RC.kf.length && !confirm(t('recSure'))) return;
  var n = Math.max(1, parseInt($('rcEvery').value, 10) || 1), comp = $('rcComp').checked, last = P.frames.length - 1; RC.kf = [];
  P.frames.forEach(function (f, i) { if (i % n && i !== last) return; var raw = []; for (var k = 0; k < N; k++) raw[k] = f.free[k] ? 0 : f.raw[k]; RC.kf.push({ raw: raw, spd: Math.max(1, Math.min(255, comp ? (f.spd || 1) * n : (f.spd || 1))) }); });
  $('rcName').value = (P.name || P.file || 'My Motion').replace(/\.pma$/i, ''); rcSave(); rcRender(); document.querySelector('[data-tab="rec"]').click();
});
$('rcScaleB').addEventListener('click', function () { var s = Math.max(0.1, +$('rcScale').value || 1); RC.kf.forEach(function (f) { f.spd = Math.max(1, Math.min(255, Math.round(f.spd / s))); }); rcSave(); rcRender(); });
try { var rcS = JSON.parse(localStorage.getItem('premaid-rc') || 'null'); if (rcS && rcS.kf) { RC.kf = rcS.kf.filter(function (f) { return f.raw && f.raw.length === N; }); if (rcS.name) $('rcName').value = rcS.name; } } catch (e) {}
rcRender();

// ------------------------------------------------------------------ motion check: used range / limits / speed per servo
function rawClampSafe(k, v) {      // PMA range ∩ remote-control safe range; 0 (released) stays 0
  if (!v) return v; var e = PM.byJoint[A.servos[k].joint], sf = SAFE[e.pid], lo = e.rawLo, hi = e.rawHi; if (sf) { lo = Math.max(lo, sf[0]); hi = Math.min(hi, sf[1]); } return Math.max(lo, Math.min(hi, v));
}
function chkStats(tl) {
  var fr = tl.frames, S = A.servos.map(function (s, k) { var e = PM.byJoint[s.joint]; return { k: k, e: e, sf: SAFE[e.pid], min: 1e9, max: -1e9, pma: 0, safe: 0, sp: 0, n: 0 }; });
  fr.forEach(function (f, n) { S.forEach(function (o) {
    var k = o.k; if (f.free[k]) return; var v = f.raw[k]; o.n++; if (v < o.min) o.min = v; if (v > o.max) o.max = v;
    if (v < o.e.rawLo || v > o.e.rawHi) o.pma++; if (o.sf && (v < o.sf[0] || v > o.sf[1])) o.safe++;
    if (n && !fr[n - 1].free[k] && tl.D[n] > 0) { var sp = Math.abs(v - fr[n - 1].raw[k]) * K_DEG / (tl.D[n] / 1000); if (sp > o.sp) o.sp = sp; }
  }); });
  return S;
}
function renderCheck(p, tl) {
  var sum = $(p + 'Sum'), tab = $(p + 'Tab'); tab.innerHTML = ''; sum.classList.remove('bad');
  if (!tl || !tl.frames.length) { sum.textContent = t('chkNone'); return; }
  var S = chkStats(tl), a = 0, b = 0, v = 0; S.forEach(function (o) { if (o.pma) a++; if (o.safe) b++; if (o.sp > v) v = o.sp; });
  sum.textContent = t('chkSum', tl.frames.length, tl.total / 1000, a, b, v); if (a) sum.classList.add('bad');
  S.forEach(function (o) {
    var s = A.servos[o.k], e = o.e, span = e.rawHi - e.rawLo, pc = function (x) { return Math.max(0, Math.min(100, (x - e.rawLo) / span * 100)); }, row = document.createElement('div'); row.className = 'chkrow';
    var nm = document.createElement('span'); nm.className = 'nm'; nm.textContent = LANG === 'zh' ? s.zh : s.en; nm.title = s.joint; row.appendChild(nm);
    var bar = document.createElement('div'); bar.className = 'cbar' + (o.pma ? ' bad' : '');
    if (o.sf) { var f = document.createElement('i'); f.className = 'sf'; f.style.left = pc(o.sf[0]) + '%'; f.style.width = Math.max(0, pc(o.sf[1]) - pc(o.sf[0])) + '%'; bar.appendChild(f); }
    var tx = document.createElement('span'); tx.className = 'tx';
    if (o.n) { var u = document.createElement('i'); u.className = 'us'; u.style.left = pc(o.min) + '%'; u.style.width = Math.max(1.5, pc(o.max) - pc(o.min)) + '%'; bar.appendChild(u);
      var d1 = s.sign * (o.min - 7500) * K_DEG, d2 = s.sign * (o.max - 7500) * K_DEG; tx.textContent = Math.round(Math.min(d1, d2)) + '…' + Math.round(Math.max(d1, d2)) + '°'; if (o.pma) { tx.classList.add('bad'); tx.textContent = '⚠ ' + tx.textContent; } }
    else tx.textContent = '–';
    row.appendChild(bar); row.appendChild(tx); tab.appendChild(row);
  });
}
// ------------------------------------------------------------------ push test (external force on the torso) and gamepad
var PUSH = { left: 0, f: [0, 0, 0] };
function pushFire(dir) {
  if (!S.ready || !S.d.xfrc_applied) return;
  var B = ikBase(), v = mvec(B.R, { fwd: [1, 0, 0], back: [-1, 0, 0], left: [0, 1, 0], right: [0, -1, 0] }[dir]), F = +$('pushF').value;
  PUSH.f = [F * v[0], F * v[1], F * v[2]]; PUSH.left = 0.1; pushApply(true);
}
function pushApply(on) { var x = S.d.xfrc_applied, o = 6 * S.base; for (var i = 0; i < 3; i++) x[o + i] = on ? PUSH.f[i] : 0; }
function pushTick(dt) { if (PUSH.left <= 0) return; PUSH.left -= dt; pushApply(PUSH.left > 0); }
Array.prototype.forEach.call(document.querySelectorAll('[data-push]'), function (b) { b.addEventListener('click', function () { pushFire(b.getAttribute('data-push')); }); });
$('pushF').addEventListener('input', function () { $('pushFV').textContent = this.value + ' N'; });
WK.gp = { x1: 0, y1: 0, x2: 0, y2: 0, a: false, name: '' };
function gpLabel() { $('wGp').textContent = WK.gp.name ? t('padGpOn', WK.gp.name) : t('padGpOff'); }
function gpPoll() {
  var gps = navigator.getGamepads ? navigator.getGamepads() : [], g = null, i, G = WK.gp, dz = function (v) { v = v || 0; return Math.abs(v) < 0.15 ? 0 : v; };
  for (i = 0; i < gps.length; i++) if (gps[i] && gps[i].connected) { g = gps[i]; break; }
  if (!g) { G.x1 = G.y1 = G.x2 = G.y2 = 0; G.a = false; G.lt = G.rt = 0; G.lb = G.rb = G.x = G.y = false; if (G.name) { G.name = ''; gpLabel(); } return; }
  if (G.name !== g.id) { G.name = g.id; gpLabel(); }
  G.x1 = dz(g.axes[0]); G.y1 = -dz(g.axes[1]); G.x2 = dz(g.axes[2]); G.y2 = -dz(g.axes[3]);
  var bt = function (n) { return g.buttons[n] || { pressed: false, value: 0 }; }; G.lt = bt(6).value; G.rt = bt(7).value; G.lb = bt(4).pressed; G.rb = bt(5).pressed; G.x = bt(2).pressed; G.y = bt(3).pressed;
  var a = !!(g.buttons[0] && g.buttons[0].pressed); if (a && !G.a) { if (!WK.on) walkStart(); else if (WK.go) { WK.go = false; wRender(); } } G.a = a;
}
gpLabel();

// ------------------------------------------------------------------ HUD wiring
function refreshRun() { $('bRun').classList.toggle('on', !S.run); $('bRun').textContent = t('freeze'); }
$('bRun').addEventListener('click', function () { S.run = !S.run; refreshRun(); });
$('bReset').addEventListener('click', function () { if (S.ready) reset(); });
$('cPin').addEventListener('change', function () { S.pin = this.checked; if (S.ready) reset(); });   // pin -> hang at the weld pose; unpin -> stand on the floor again
$('cGrav').addEventListener('change', function () { if (S.m) S.m.opt.gravity[2] = this.checked ? -9.81 : 0; });
$('rMu').addEventListener('input', function () { setMu(this.value); });
$('rKp').addEventListener('input', function () { setKp(this.value); });
$('sSpeed').addEventListener('change', function () { S.speed = +this.value; });
setMu(S.mu); setKp(S.kp);
