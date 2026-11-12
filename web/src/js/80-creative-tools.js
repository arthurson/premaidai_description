// ================================================================== P2: stand assist, sensor HUD, share / kiosk, recording, playlist, editor, teleop, mocap, shortcuts
function dlBlob(blob, name) { var a = document.createElement('a'); a.href = URL.createObjectURL(blob); a.download = name; document.body.appendChild(a); a.click(); a.remove(); setTimeout(function () { URL.revokeObjectURL(a.href); }, 4000); }
function curTitle() { return (LIB.cur >= 0 && LIB.items[LIB.cur] ? LIB.items[LIB.cur].title : (P.name || 'premaid')).replace(/[\\/:*?"<>|]+/g, '_'); }
function clickId(id) { var e = $(id); if (e) e.click(); }
function toggleId(id) { var e = $(id); if (e) { e.checked = !e.checked; e.dispatchEvent(new Event('change')); } }
// ---- stand assist: ankle strategy (pitch / roll PD on the base IMU) + lateral CoM trim over the support foot (hip roll + opposite ankle roll).
// Measured in this simulator (tests/e2e_p2.py, 0.1 s pushes): the sideways tipping threshold goes from 9.0 N to 9.5 N, forward from 9.0 N to 10 N, i.e. +0.5..1 N (5-10 %). Single-leg stance is NOT solved.
var BAL = { on: false, kp: 1, kd: 0.05, kc: 600, lim: 20, o: [], ak: [], ax: [], hx: [], idx: false, trim: 0, fwd: 0, side: 0 };
function balIndex() { if (BAL.idx) return; BAL.idx = true; A.servos.forEach(function (s, k) { var j = s.joint; if (/^[rl]_aky_joint$/.test(j)) BAL.ak.push(k); else if (/^[rl]_akx_joint$/.test(j)) BAL.ax.push(k); else if (/^[rl]_hpx_joint$/.test(j)) BAL.hx.push(k); }); }
function balStep() {
  balIndex(); var d = S.d, b = S.base, xq = d.xquat, R = qmat([xq[4 * b], xq[4 * b + 1], xq[4 * b + 2], xq[4 * b + 3]]), cl = function (v) { return Math.max(-1, Math.min(1, v)); };
  var fwd = Math.asin(cl(-R[6])), side = Math.asin(cl(R[7])), wx = d.qvel[3], wy = d.qvel[4];
  var rf = bid.r_foot, lf = bid.l_foot, xp = d.xpos, pr = [xp[3 * rf], xp[3 * rf + 1], xp[3 * rf + 2]], pl = [xp[3 * lf], xp[3 * lf + 1], xp[3 * lf + 2]];
  var sup = pl[2] - pr[2] > 0.012 ? pr : pr[2] - pl[2] > 0.012 ? pl : [(pr[0] + pl[0]) / 2, (pr[1] + pl[1]) / 2];
  var sc = d.subtree_com, yaw = Math.atan2(R[3], R[0]), ey = -Math.sin(yaw) * (sc[3 * b] - sup[0]) + Math.cos(yaw) * (sc[3 * b + 1] - sup[1]);
  var trim = Math.max(-BAL.lim, Math.min(BAL.lim, BAL.kc * ey)) * D2R, op = BAL.kp * fwd + BAL.kd * wy, or = BAL.kp * side + BAL.kd * wx;
  BAL.o = []; BAL.ak.forEach(function (k) { BAL.o[k] = op; }); BAL.ax.forEach(function (k) { BAL.o[k] = or - trim; }); BAL.hx.forEach(function (k) { BAL.o[k] = trim; });
  BAL.trim = trim / D2R; BAL.fwd = fwd / D2R; BAL.side = side / D2R;
}
function balRender() { $('balInfo').textContent = BAL.on ? t('balInfo', BAL.fwd, BAL.side, BAL.trim) : ''; }
['balKp', 'balKd', 'balKc'].forEach(function (id) { $(id).addEventListener('input', function () { BAL[id === 'balKp' ? 'kp' : id === 'balKd' ? 'kd' : 'kc'] = +this.value; $(id + 'V').textContent = this.value; }); });
$('balOn').addEventListener('change', function () { BAL.on = this.checked; if (!BAL.on) BAL.o = []; balRender(); });
// ---- sensor HUD (IMU from the base state, foot contact force from MuJoCo contacts, joint torque, CoM + trail)
var SEN = { pv: null, pt: 0, acc: [0, 0, 0], trail: null, n: 0, last: null, tl: 0, cf: new Float64Array(6), noF: false, T: 0 };
function senTrail() {
  if (SEN.trail) return SEN.trail; var g = new THREE.BufferGeometry(); g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3 * 800), 3)); g.setDrawRange(0, 0);
  SEN.trail = new THREE.Line(g, new THREE.LineBasicMaterial({ color: 0xff8a00, depthTest: false })); SEN.trail.renderOrder = 8; SEN.trail.frustumCulled = false; world.add(SEN.trail); return SEN.trail;
}
function senFoot() {      // normal force (N) on each foot: sum of the contact's friction-pyramid facet forces in efc_force (elliptic cone: the first row)
  var d = S.d, m = S.m, rf = bid.r_foot, lf = bid.l_foot, out = { r: 0, l: 0, nr: 0, nl: 0 }, i, j, c, b1, b2, f, ad, nr, fn;
  for (i = 0; i < d.ncon; i++) {
    c = d.contact.get(i); b1 = m.geom_bodyid[c.geom1]; b2 = m.geom_bodyid[c.geom2]; f = b1 === rf || b2 === rf ? 'r' : b1 === lf || b2 === lf ? 'l' : ''; if (!f || c.dist > 0.0005) continue;
    out['n' + f]++; ad = c.efc_address; if (ad < 0) continue; nr = m.opt.cone === 0 ? 2 * (c.dim - 1) : 1; fn = 0; for (j = 0; j < nr; j++) fn += d.efc_force[ad + j]; out[f] += Math.abs(fn);
  } return out;
}
function senTick(now) {
  var on = $('cSen').checked, tr = $('cTrail').checked; $('senHud').style.display = on ? 'block' : 'none';
  var d = S.d, b = S.base, sc = d.subtree_com, cx = sc[3 * b], cy = sc[3 * b + 1], cz = sc[3 * b + 2];
  if (S.simT < SEN.tl - 0.01) SEN.n = 0; SEN.tl = S.simT;
  if (tr) {
    var ln = senTrail(), p = ln.geometry.attributes.position, a = p.array, mv = !SEN.last || Math.hypot(cx - SEN.last[0], cy - SEN.last[1]) > 0.0015;
    if (mv) { if (SEN.n >= 800) { a.copyWithin(0, 3, 2400); SEN.n = 799; } a[3 * SEN.n] = cx; a[3 * SEN.n + 1] = cy; a[3 * SEN.n + 2] = 0.002; SEN.n++; SEN.last = [cx, cy]; p.needsUpdate = true; ln.geometry.setDrawRange(0, SEN.n); }
    ln.visible = true;
  } else if (SEN.trail) { SEN.trail.visible = false; if (SEN.n) { SEN.n = 0; SEN.last = null; } }
  if (!on || now - SEN.T < 100) return; SEN.T = now;
  var xq = d.xquat, R = qmat([xq[4 * b], xq[4 * b + 1], xq[4 * b + 2], xq[4 * b + 3]]), dg = 180 / Math.PI, v = [d.qvel[0], d.qvel[1], d.qvel[2]];
  if (SEN.pv && S.simT - SEN.pt > 0.04) { var dt = S.simT - SEN.pt, aw = [(v[0] - SEN.pv[0]) / dt, (v[1] - SEN.pv[1]) / dt, (v[2] - SEN.pv[2]) / dt + 9.81], ab = mtvec(R, aw); for (var i = 0; i < 3; i++) SEN.acc[i] = SEN.ai ? SEN.acc[i] * 0.5 + ab[i] * 0.5 : ab[i]; SEN.ai = true; SEN.pv = v; SEN.pt = S.simT; } else if (!SEN.pv || S.simT < SEN.pt) { SEN.pv = v; SEN.pt = S.simT; }
  var ft = senFoot(), af = d.actuator_force, fr = S.m.actuator_forcerange, rank = [], k, f1 = function (x) { return (x >= 0 ? '+' : '') + x.toFixed(1); };
  for (k = 0; k < N; k++) rank.push([Math.abs(af[act[k]]) / fr[2 * act[k] + 1], k]); rank.sort(function (a, b2) { return b2[0] - a[0]; });
  $('senHud').textContent =
    'IMU  roll ' + f1(Math.atan2(R[7], R[8]) * dg) + '°  pitch ' + f1(Math.asin(Math.max(-1, Math.min(1, -R[6]))) * dg) + '°  yaw ' + f1(Math.atan2(R[3], R[0]) * dg) + '°\n' +
    'gyro ' + f1(d.qvel[3] * dg) + ' ' + f1(d.qvel[4] * dg) + ' ' + f1(d.qvel[5] * dg) + ' °/s\n' +
    'accel ' + f1(SEN.acc[0]) + ' ' + f1(SEN.acc[1]) + ' ' + f1(SEN.acc[2]) + ' m/s²\n' +
    t('senFoot') + '  R ' + ft.r.toFixed(2) + ' N   L ' + ft.l.toFixed(2) + ' N' + '\n' +
    'CoM  x ' + f1(cx * 100) + '  y ' + f1(cy * 100) + '  z ' + (cz * 100).toFixed(1) + ' cm\n' +
    t('senTq') + '  ' + rank.slice(0, 3).map(function (r) { return '#' + String(r[1] + 1).padStart(2, '0') + ' ' + A.servos[r[1]].joint.replace('_joint', '') + ' ' + (r[0] * fr[2 * act[r[1]] + 1]).toFixed(2) + ' (' + Math.round(r[0] * 100) + '%)'; }).join('\n          ');
}
// ---- share link / kiosk / embed:  #m=<motion title>&t=<ms>&cam=x,y,z,tx,ty,tz&play=0&speed=&loop=0&hang=1&kiosk=1&embed=1&bar=0
function shareHash() {
  var p = [];
  if (LIB.cur >= 0 && LIB.items[LIB.cur]) { p.push('m=' + encodeURIComponent(LIB.items[LIB.cur].title)); p.push('t=' + Math.round(P.t)); if (!P.playing) p.push('play=0'); }
  p.push('cam=' + [camera.position.x, camera.position.y, camera.position.z, controls.target.x, controls.target.y, controls.target.z].map(function (v) { return v.toFixed(3); }).join(','));
  if (S.speed !== 1) p.push('speed=' + S.speed); if (!P.loop) p.push('loop=0'); if ($('cPin').checked) p.push('hang=1');
  return '#' + p.join('&');
}
function shareUrl(extra) { return location.href.split('#')[0] + shareHash() + (extra || ''); }
function hashParse(h) { var o = {}; String(h == null ? location.hash : h).replace(/^#/, '').split('&').forEach(function (kv) { if (!kv) return; var i = kv.indexOf('='), k = i < 0 ? kv : kv.slice(0, i), v = i < 0 ? '1' : kv.slice(i + 1); try { v = decodeURIComponent(v); } catch (e) {} o[k] = v; }); return o; }
function hashApply(o) {
  if (o.kiosk === '1' || o.embed === '1') document.body.classList.add('kiosk'); if (o.embed === '1') document.body.classList.add('embed'); if (o.bar === '0') document.body.classList.add('nobar');
  if (o.speed && +o.speed > 0) { S.speed = +o.speed; var so = $('sSpeed'), has = Array.prototype.some.call(so.options, function (op) { return +op.value === S.speed; }); if (has) so.value = String(S.speed); }
  if (o.loop === '0') { $('pLoop').checked = false; P.loop = false; }
  if (o.hang === '1') { $('cPin').checked = true; $('cPin').dispatchEvent(new Event('change')); }
  var cam = (o.cam || '').split(',').map(Number); if (cam.length === 6 && cam.every(isFinite)) { camera.position.set(cam[0], cam[1], cam[2]); controls.target.set(cam[3], cam[4], cam[5]); controls.update(); }
  if (o.m) {
    var i = -1; LIB.items.forEach(function (it, n) { if (it.title === o.m) i = n; }); if (i < 0 && /^\d+$/.test(o.m) && LIB.items[+o.m]) i = +o.m;
    if (i >= 0) libLoad(i, function () { if (o.t && isFinite(+o.t)) { P.t = Math.max(0, Math.min(P.total, +o.t)); poseAt(P.t); } if (o.play === '0') P.playing = false; updatePlayUI(true); });
  }
}
function hashBoot() { try { hashApply(hashParse()); } catch (e) { console.warn(e); } }
$('shBtn').addEventListener('click', function () { var u = shareUrl(); try { navigator.clipboard.writeText(u); } catch (e) { prompt('URL', u); } flash(this, t('copiedL'), 'shBtn'); });
$('emBtn').addEventListener('click', function () { var u = shareUrl('&embed=1&bar=0'), c = '<iframe src="' + u + '" width="640" height="480" allow="fullscreen; autoplay" style="border:0"></iframe>'; try { navigator.clipboard.writeText(c); } catch (e) { prompt('HTML', c); } flash(this, t('copiedL'), 'emBtn'); });
function fsToggle() { var d = document, el = d.documentElement; if (d.fullscreenElement || d.webkitFullscreenElement) (d.exitFullscreen || d.webkitExitFullscreen).call(d); else (el.requestFullscreen || el.webkitRequestFullscreen || function () {}).call(el); }
$('fsBtn').addEventListener('click', fsToggle);
// ---- recording: WebM (canvas + music) and GIF (own encoder, 3-3-2 palette)
var REC = { mr: null, chunks: [], once: false, gif: null, off: null };
function recUI() { $('recBtn').textContent = t(REC.mr ? 'recStop' : 'recBtn'); $('recBtn').classList.toggle('danger', !!REC.mr); $('gifBtn').disabled = !!REC.gif; }
function recStart() {
  if (REC.mr) { recStop(); return; }
  if (!window.MediaRecorder || !canvas.captureStream) { flash($('recBtn'), t('noRec'), 'recBtn'); return; }
  var stream = canvas.captureStream(30);
  try { var as = AU.el && AU.el.src && !AU.drm ? (AU.el.captureStream ? AU.el.captureStream() : (AU.el.mozCaptureStream ? AU.el.mozCaptureStream() : null)) : null; if (as) as.getAudioTracks().forEach(function (tr) { stream.addTrack(tr); }); } catch (e) {}
  var mt = ['video/webm;codecs=vp9,opus', 'video/webm;codecs=vp8,opus', 'video/webm;codecs=vp8', 'video/webm'].filter(function (m) { return MediaRecorder.isTypeSupported(m); })[0];
  if (!mt) { flash($('recBtn'), t('noRec'), 'recBtn'); return; }
  REC.chunks = []; REC.name = curTitle(); REC.mr = new MediaRecorder(stream, { mimeType: mt, videoBitsPerSecond: 5e6 });
  REC.mr.ondataavailable = function (e) { if (e.data && e.data.size) REC.chunks.push(e.data); };
  REC.mr.onstop = function () { var blob = new Blob(REC.chunks, { type: 'video/webm' }); REC.last = blob; REC.mr = null; REC.once = false; recUI(); dlBlob(blob, REC.name + '.webm'); };
  REC.mr.start(250);
  if ($('recOnce').checked && P.frames) { P.loop = false; $('pLoop').checked = false; P.t = 0; poseAt(0); P.playing = true; REC.once = true; updatePlayUI(true); }
  recUI();
}
function recStop() { if (REC.mr && REC.mr.state !== 'inactive') REC.mr.stop(); }
function lzwEncode(px, minCode) {      // GIF variant of LZW (variable code size, clear / end codes)
  var initBits = minCode + 1, clear = 1 << minCode, eof = clear + 1, nBits = initBits, maxcode = (1 << nBits) - 1, free = clear + 2, out = [], cur = 0, nb = 0, dict = new Map();
  function put(code, isClear) { cur |= code << nb; nb += nBits; while (nb >= 8) { out.push(cur & 255); cur >>>= 8; nb -= 8; } if (isClear) { nBits = initBits; maxcode = (1 << nBits) - 1; } else if (free > maxcode) { nBits++; maxcode = nBits === 12 ? 4096 : (1 << nBits) - 1; } }
  put(clear, true); var ent = px[0], i, c, key, got;
  for (i = 1; i < px.length; i++) {
    c = px[i]; key = (c << 12) + ent; got = dict.get(key);
    if (got !== undefined) ent = got;
    else { put(ent, false); ent = c; if (free < 4096) dict.set(key, free++); else { dict.clear(); free = clear + 2; put(clear, true); } }
  }
  put(ent, false); put(eof, false); if (nb > 0) out.push(cur & 255); return out;
}
function gifEncode(frames, w, h, delayMs) {      // frames: RGBA Uint8ClampedArray each; fixed 3-3-2 palette, endless loop
  var o = [], i, f, k, px, lz, n;
  function w16(v) { o.push(v & 255, (v >> 8) & 255); }
  o.push(71, 73, 70, 56, 57, 97); w16(w); w16(h); o.push(0xF7, 0, 0);
  for (i = 0; i < 256; i++) o.push(Math.round(((i >> 5) & 7) * 255 / 7), Math.round(((i >> 2) & 7) * 255 / 7), Math.round((i & 3) * 255 / 3));
  o.push(0x21, 0xFF, 11, 78, 69, 84, 83, 67, 65, 80, 69, 50, 46, 48, 3, 1, 0, 0, 0);
  for (f = 0; f < frames.length; f++) {
    var src = frames[f]; px = new Uint8Array(w * h); for (k = 0, n = w * h; k < n; k++) px[k] = (src[4 * k] & 0xE0) | ((src[4 * k + 1] >> 3) & 0x1C) | (src[4 * k + 2] >> 6);
    var dc = Math.max(2, Math.round(delayMs / 10)); o.push(0x21, 0xF9, 4, 0, dc & 255, (dc >> 8) & 255, 0, 0);
    o.push(0x2C); w16(0); w16(0); w16(w); w16(h); o.push(0, 8);
    lz = lzwEncode(px, 8); for (k = 0; k < lz.length; k += 255) { n = Math.min(255, lz.length - k); o.push(n); for (var q = 0; q < n; q++) o.push(lz[k + q]); } o.push(0);
  }
  o.push(0x3B); return Uint8Array.from(o);
}
function gifStart() {
  if (REC.gif) return; var secs = Math.max(1, Math.min(10, +$('gifSec').value || 3)), w = 320, h = Math.max(2, Math.round(w * canvas.height / canvas.width));
  REC.gif = { frames: [], w: w, h: h, need: secs * 10, last: 0, name: curTitle() }; recUI();
}
function gifSnap(now) {      // called right after renderer.render, while the WebGL buffer is still valid
  var g = REC.gif; if (!g || now - g.last < 100) return; g.last = now;
  if (!REC.off) REC.off = document.createElement('canvas'); var c = REC.off; c.width = g.w; c.height = g.h; var x = c.getContext('2d', { willReadFrequently: true }); x.drawImage(canvas, 0, 0, g.w, g.h); g.frames.push(x.getImageData(0, 0, g.w, g.h).data);
  $('gifBtn').textContent = 'GIF ' + g.frames.length + '/' + g.need;
  if (g.frames.length >= g.need) { REC.gif = null; setTimeout(function () { var bytes = gifEncode(g.frames, g.w, g.h, 100); REC.lastGif = bytes; $('gifBtn').textContent = t('gifBtn'); recUI(); dlBlob(new Blob([bytes], { type: 'image/gif' }), g.name + '.gif'); }, 20); }
}
$('recBtn').addEventListener('click', recStart); $('gifBtn').addEventListener('click', gifStart);
// ---- playlist (library): auto-next, shuffle, prev / next; screen kept on while a motion plays
var PL = { auto: false, shuf: false, hist: [], wake: null };
function plPick(dir) {
  var n = LIB.items.length; if (!n) return -1; var cur = LIB.cur, i;
  if (PL.shuf && dir > 0) { var c = []; for (i = 0; i < n; i++) if (i !== cur) c.push(i); return c.length ? c[Math.floor(Math.random() * c.length)] : cur; }
  if (PL.shuf && dir < 0 && PL.hist.length) return PL.hist.pop();
  return ((cur < 0 ? (dir > 0 ? -1 : 0) : cur) + dir + n) % n;
}
function plNext(dir) { var i = plPick(dir || 1); if (i < 0) return; if (LIB.cur >= 0 && (dir || 1) > 0) PL.hist.push(LIB.cur); libLoad(i); }
$('plAuto').addEventListener('change', function () { PL.auto = this.checked; }); $('plShuf').addEventListener('change', function () { PL.shuf = this.checked; });
$('plPrev').addEventListener('click', function () { plNext(-1); }); $('plNext').addEventListener('click', function () { plNext(1); });
async function pwSync() {
  if (!$('plWake').checked || !navigator.wakeLock) return;
  if (P.playing && !PL.wake && !PL.wl) { PL.wl = true; try { PL.wake = await navigator.wakeLock.request('screen'); PL.wake.addEventListener('release', function () { PL.wake = null; }); } catch (e) {} PL.wl = false; }
  else if (!P.playing && PL.wake) { try { PL.wake.release(); } catch (e) {} PL.wake = null; }
}
// ---- A-B loop
var AB = { a: -1, b: -1 };
function abRender() {
  var on = AB.a >= 0 && AB.b > AB.a && P.total > 0, s = $('pSeek');
  s.style.background = on ? 'linear-gradient(to right, transparent ' + (AB.a / P.total * 100) + '%, color-mix(in srgb, var(--accent) 35%, transparent) ' + (AB.a / P.total * 100) + '%, color-mix(in srgb, var(--accent) 35%, transparent) ' + (AB.b / P.total * 100) + '%, transparent ' + (AB.b / P.total * 100) + '%)' : '';
  $('abTxt').textContent = AB.a >= 0 || AB.b >= 0 ? t('abTxt', AB.a >= 0 ? fmt(AB.a) : '–', AB.b >= 0 ? fmt(AB.b) : '–') : '';
}
function abSet(w) {
  if (!P.frames) return; var tt = Math.round(P.t);
  if (w === 'a') { AB.a = tt; if (AB.b <= AB.a) AB.b = -1; } else { AB.b = tt; if (AB.a < 0 || AB.a >= AB.b) AB.a = 0; }
  abRender();
}
function abClear() { AB.a = AB.b = -1; abRender(); }
$('abA').addEventListener('click', function () { abSet('a'); }); $('abB').addEventListener('click', function () { abSet('b'); }); $('abX').addEventListener('click', abClear);
// ---- motion editor: selection, insert / duplicate / trim / split / merge, onion skin
RC.sel = -1; RC.alt = null;
function rcSelect(i) { RC.sel = i; rcRender(); }
function rcEdit(fn) { fn(); if (RC.sel >= RC.kf.length) RC.sel = RC.kf.length - 1; rcSave(); rcRender(); }
$('rcIns').addEventListener('click', function () { rcEdit(function () { var at = RC.sel >= 0 ? RC.sel + 1 : RC.kf.length; RC.kf.splice(at, 0, { raw: rcRaw(), spd: +$('rcSpd').value }); RC.sel = at; }); });
$('rcDup').addEventListener('click', function () { if (RC.sel < 0) return; rcEdit(function () { var f = RC.kf[RC.sel]; RC.kf.splice(RC.sel + 1, 0, { raw: f.raw.slice(), spd: f.spd }); RC.sel++; }); });
$('rcTrimB').addEventListener('click', function () { if (RC.sel <= 0) return; rcEdit(function () { RC.kf.splice(0, RC.sel); RC.sel = 0; }); });
$('rcTrimA').addEventListener('click', function () { if (RC.sel < 0) return; rcEdit(function () { RC.kf.splice(RC.sel + 1); }); });
$('rcSplit').addEventListener('click', function () { if (RC.sel <= 0) return; rcEdit(function () { RC.alt = { name: rcName() + ' (2)', kf: RC.kf.splice(RC.sel) }; RC.sel = RC.kf.length - 1; }); });
$('rcMerge').addEventListener('click', function () { if (!RC.alt) return; rcEdit(function () { RC.kf = RC.kf.concat(RC.alt.kf); RC.alt = null; }); });
var ON = { d2: null, g: [null, null], T: 0 };
function onInit() {
  if (ON.d2 || !S.ready) return !!ON.d2; ON.d2 = new S.mj.MjData(S.m);
  [0x4488ff, 0xff8844].forEach(function (col, gi) {
    var grp = new THREE.Group(), mat = new THREE.MeshBasicMaterial({ color: col, transparent: true, opacity: 0.3, depthWrite: false }); grp.userData.m = {}; grp.visible = false;
    for (var nm in bodyMesh) { var ms = new THREE.Mesh(bodyMesh[nm].geometry, mat); ms.frustumCulled = false; grp.add(ms); grp.userData.m[nm] = ms; }
    world.add(grp); ON.g[gi] = grp;
  }); return true;
}
function onPose(grp, f) {
  var d = S.d, d2 = ON.d2, i, k; for (i = 0; i < 7; i++) d2.qpos[i] = d.qpos[i];
  for (k = 0; k < N; k++) { var raw = f.raw[k], deg = raw ? A.servos[k].sign * (raw - 7500) * K_DEG : target[k]; d2.qpos[qadr[k]] = clampDeg(k, deg) * D2R; }
  S.mj.mj_forward(S.m, d2); var xp = d2.xpos, xq = d2.xquat;
  for (var nm in grp.userData.m) { var b = bid[nm], m = grp.userData.m[nm]; m.position.set(xp[3 * b], xp[3 * b + 1], xp[3 * b + 2]); m.quaternion.set(xq[4 * b + 1], xq[4 * b + 2], xq[4 * b + 3], xq[4 * b]); }
}
function onRefresh() {
  if (typeof ON === 'undefined' || !ON) return;      // rcRender runs once before this block has executed
  var show = $('rcOnion') && $('rcOnion').checked && S.ready && RC.sel >= 0 && RC.kf.length > 1; if (!show) { ON.g.forEach(function (g) { if (g) g.visible = false; }); return; }
  if (!onInit()) return; [-1, 1].forEach(function (dir, gi) { var f = RC.kf[RC.sel + dir]; ON.g[gi].visible = !!f; if (f) onPose(ON.g[gi], f); });
}
$('rcOnion').addEventListener('change', onRefresh);
// ---- teleop: gamepad (or keyboard when enabled) drives head + the active arm
var TL = { on: false, arm: 'r', keys: {}, last: { lb: false, rb: false, y: false, x: false }, idx: null };
function tlIdx() { if (TL.idx) return TL.idx; var o = {}; A.servos.forEach(function (s, k) { o[s.joint.replace('_joint', '')] = k; }); return (TL.idx = o); }
function tlRender() { $('teleInfo').textContent = TL.on ? t('teleInfo', TL.arm === 'r' ? t('armR') : t('armL')) : ''; }
function teleTick(dt) {
  if (!TL.on || WK.on || !S.ready) return; var I = tlIdx(), G = WK.gp, K = TL.keys, ax = function (neg, pos) { return (K[pos] ? 1 : 0) - (K[neg] ? 1 : 0); };
  var hx = G.x1 + ax('a', 'd'), hy = G.y1 + ax('s', 'w'), ax2 = G.x2 + ax('j', 'l'), ay2 = G.y2 + ax('k', 'i'), el = (G.rt || 0) - (G.lt || 0) + ax('u', 'o');
  if (G.lb && !TL.last.lb) { TL.arm = 'l'; tlRender(); } if (G.rb && !TL.last.rb) { TL.arm = 'r'; tlRender(); } if (K.q) { TL.arm = 'l'; tlRender(); } if (K.e) { TL.arm = 'r'; tlRender(); }
  if (G.y && !TL.last.y) toggleId('balOn'); if (G.x && !TL.last.x) clickId('pPlay'); TL.last = { lb: !!G.lb, rb: !!G.rb, y: !!G.y, x: !!G.x };
  if (!(hx || hy || ax2 || ay2 || el)) return; touch(); var s = TL.arm, roll = s === 'r' ? -1 : 1;
  function add(name, dd) { var k = I[name]; if (k !== undefined) setTarget(k, target[k] + dd); }
  add('head_nz', -hx * 60 * dt); add('head_ny', -hy * 40 * dt); add(s + '_shx', roll * ax2 * 90 * dt); add(s + '_shy', -ay2 * 90 * dt); add(s + '_ely', el * 90 * dt);
}
$('teleOn').addEventListener('change', function () { TL.on = this.checked; TL.keys = {}; tlRender(); });
// ---- webcam arm mocap (MediaPipe Pose world landmarks -> IK on the arm chains). The runtime (camera, model download) needs network + a camera and is NOT tested here; the landmark -> joint math is (tests/e2e_p2.py).
var MC = { on: false, lm: null, fl: null, face: null, fc: 0, vid: null, stream: null, raf: 0, seen: 0, sm: {}, raw: {}, off: {}, headTg: {} };
function mcNorm(v) { var n = Math.hypot(v[0], v[1], v[2]) || 1; return [v[0] / n, v[1] / n, v[2] / n]; }
function mcCross(a, b) { return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]; }
function mcDot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
function mcSub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
function mcPoint(l) { return [l.x, l.y, l.z]; }
function mcPoseToTargets(lm, cur, opt) {      // lm: 33 MediaPipe world landmarks (right-handed, any axes); returns {k: deg} for both arms
  var Ls = mcPoint(lm[11]), Rs = mcPoint(lm[12]), Lh = mcPoint(lm[23]), Rh = mcPoint(lm[24]);
  var ex = mcNorm(mcSub(Ls, Rs)), up = mcNorm(mcSub([(Ls[0] + Rs[0]) / 2, (Ls[1] + Rs[1]) / 2, (Ls[2] + Rs[2]) / 2], [(Lh[0] + Rh[0]) / 2, (Lh[1] + Rh[1]) / 2, (Lh[2] + Rh[2]) / 2]));
  var fw = mcNorm(mcCross(ex, up)); ex = mcNorm(mcCross(up, fw));      // person frame: forward, left, up  (= the robot base frame)
  var mir = opt && opt.mirror ? -1 : 1, b = function (v) { return [mcDot(v, fw), mir * mcDot(v, ex), mcDot(v, up)]; }, out = {};      // mirror: lateral axis flipped and left/right swapped below
  (mir < 0 ? [['l', 12, 14, 16], ['r', 11, 13, 15]] : [['l', 11, 13, 15], ['r', 12, 14, 16]]).forEach(function (c) {
    var ch = A.chains[c[0] + '_arm']; if (!ch) return; var S0 = mcPoint(lm[c[1]]), E = mcPoint(lm[c[2]]), W = mcPoint(lm[c[3]]), sh = ch.joints[0].pos;
    var robotLen = Math.hypot.apply(null, ikFK(ch, ch.joints.map(function () { return 0; })).p.map(function (v, i) { return v - sh[i]; })), pl = Math.hypot.apply(null, mcSub(E, S0)) + Math.hypot.apply(null, mcSub(W, E)), sc = 0.88 * robotLen / (pl || 1);      // a fully stretched human arm maps to 88 % of the robot's reach (the IK cannot hit the exact limit)
    var d = b(mcSub(W, S0)), pt = [sh[0] + sc * d[0], sh[1] + sc * d[1], sh[2] + sc * d[2]], q0 = ch.joints.map(function (j) { return (cur ? cur[j.k] : target[j.k]) * D2R; }), res = ikSolve(ch, q0, pt);
    ch.joints.forEach(function (j, i) { out[j.k] = res.q[i] / D2R; }); out['err_' + c[0]] = res.err;
  }); if (opt && opt.legs) mcLegs(lm, b, out, mir < 0); return out;
}
// ---- head + legs (added): pure joint maths, person frame = [forward, left, up] = the robot base frame; joint axes from A.chains (hpz=z, hpx=x, hpy=y, kny=y, aky=y, akx=x)
function mcKey(joint) { if (!mcKey.c) { mcKey.c = {}; A.servos.forEach(function (sv, k) { mcKey.c[sv.joint] = k; }); } return mcKey.c[joint]; }
function mcAngle(a, b) { return Math.acos(Math.max(-1, Math.min(1, mcDot(mcNorm(a), mcNorm(b))))) / D2R; }
function mcLegAngles(hip, knee, ankle, heel, toe, yawOff) {      // person-frame points -> degrees {hpz, hpx, hpy, kny, aky, akx, yawRaw}
  var d = mcNorm(mcSub(knee, hip)), f = mcSub(toe, heel), yawRaw = Math.atan2(f[1], f[0]) / D2R;      // toe turned to the left = +
  var a = Math.max(-40, Math.min(40, yawRaw - (yawOff || 0))) * D2R, c = Math.cos(a), sn = Math.sin(a);
  var x = d[0] * c + d[1] * sn, y = -d[0] * sn + d[1] * c, z = d[2];      // undo the hip yaw (hpz is the first joint of the chain)
  var roll = Math.atan2(y, -z) / D2R, pitch = Math.atan2(-x, Math.hypot(y, z)) / D2R, kn = mcAngle(d, mcSub(ankle, knee));      // thigh forward = negative pitch; knee bends the shin backwards
  return { hpz: a / D2R, hpx: roll, hpy: pitch, kny: kn, aky: -(pitch + kn), akx: -roll, yawRaw: yawRaw };      // ankle cancels hip + knee so the foot stays level
}
function mcLegs(lm, b, out, mirror) {
  (mirror ? [['l', 24, 26, 28, 30, 32], ['r', 23, 25, 27, 29, 31]] : [['l', 23, 25, 27, 29, 31], ['r', 24, 26, 28, 30, 32]]).forEach(function (c) {
    for (var i = 1; i < 6; i++) if (lm[c[i]].visibility !== undefined && lm[c[i]].visibility < 0.5) return;      // leg hidden -> keep the last pose
    var P = function (n) { return b(mcPoint(lm[n])); }, g = mcLegAngles(P(c[1]), P(c[2]), P(c[3]), P(c[4]), P(c[5]), MC.off['hz_' + c[0]]);
    MC.raw['hz_' + c[0]] = g.yawRaw;
    ['hpz', 'hpx', 'hpy', 'kny', 'aky', 'akx'].forEach(function (n) { out[mcKey(c[0] + '_' + n + '_joint')] = g[n]; });
  });
}
function mcFaceAngles(m) {      // 4x4 facial transformation matrix (canonical face -> camera) -> degrees; yaw + = face turns to the person's left, pitch + = nod down, roll + = tilt to the person's right
  if (Math.abs(m[12]) + Math.abs(m[13]) + Math.abs(m[14]) < 1e-6 && Math.abs(m[3]) + Math.abs(m[7]) + Math.abs(m[11]) > 1e-6) m = [m[0], m[4], m[8], m[12], m[1], m[5], m[9], m[13], m[2], m[6], m[10], m[14], m[3], m[7], m[11], m[15]];      // row-major -> column-major
  var n = Math.hypot(m[8], m[9], m[10]) || 1;
  return { yaw: Math.atan2(m[8], m[10]) / D2R, pitch: Math.asin(Math.max(-1, Math.min(1, -m[9] / n))) / D2R, roll: Math.atan2(m[1], m[5]) / D2R };
}
async function mcStart() {
  if (MC.on) { mcStop(); return; } $('mcInfo').textContent = t('mcLoading');
  try {
    MC.stream = await navigator.mediaDevices.getUserMedia({ video: { width: 640, height: 480 } }); MC.vid = $('mcVid'); MC.vid.srcObject = MC.stream; await MC.vid.play(); $('mcPrev').hidden = false; mcPrevMirror();
    var V = await import('https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.14/vision_bundle.mjs'), fs = await V.FilesetResolver.forVisionTasks('https://cdn.jsdelivr.net/npm/@mediapipe/tasks-vision@0.10.14/wasm');
    MC.lm = await V.PoseLandmarker.createFromOptions(fs, { baseOptions: { modelAssetPath: 'https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_lite/float16/1/pose_landmarker_lite.task' }, runningMode: 'VIDEO', numPoses: 1 });
    MC.fl = null; MC.face = null; MC.headTg = {}; var warn = '';
    { try { MC.fl = await V.FaceLandmarker.createFromOptions(fs, { baseOptions: { modelAssetPath: 'https://storage.googleapis.com/mediapipe-models/face_landmarker/face_landmarker/float16/1/face_landmarker.task' }, runningMode: 'VIDEO', numFaces: 1, outputFacialTransformationMatrixes: true }); } catch (e) { warn = t('mcFaceErr', e && e.message || String(e)); } }
    MC.on = true; $('mcBtn').textContent = t('mcStop'); $('mcInfo').textContent = warn || t('mcOn'); touch(); mcLoop();
  } catch (e) { $('mcInfo').textContent = t('mcErr', e && e.message || String(e)); mcStop(true); }
}
function mcPrevMirror() { $('mcPrev').classList.toggle('mir', $('mcMirror').checked); }
var MC_BONES = [[11, 13], [13, 15], [12, 14], [14, 16], [11, 12], [11, 23], [12, 24], [23, 24], [23, 25], [25, 27], [27, 29], [27, 31], [24, 26], [26, 28], [28, 30], [28, 32]];
function mcDraw(lm) {      // skeleton over the preview; lm = normalised image landmarks
  var cv = $('mcCv'), v = MC.vid; if (!cv || !v || !v.videoWidth) return; if (cv.width !== v.videoWidth) { cv.width = v.videoWidth; cv.height = v.videoHeight; }
  var g = cv.getContext('2d'), W = cv.width, H = cv.height, ok = function (p) { return p.visibility === undefined || p.visibility >= 0.5; }; g.clearRect(0, 0, W, H); if (!lm) return;
  g.lineWidth = 3; g.strokeStyle = '#a78bfa'; g.fillStyle = '#fb923c';
  MC_BONES.forEach(function (e) { var p = lm[e[0]], q = lm[e[1]]; if (!ok(p) || !ok(q)) return; g.beginPath(); g.moveTo(p.x * W, p.y * H); g.lineTo(q.x * W, q.y * H); g.stroke(); });
  [0, 11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28].forEach(function (i) { var p = lm[i]; if (!ok(p)) return; g.beginPath(); g.arc(p.x * W, p.y * H, 4, 0, 7); g.fill(); });
}
function mcLoop() {
  if (!MC.on) return; MC.raf = requestAnimationFrame(mcLoop); var now = performance.now(); if (now - MC.seen < 40 || MC.vid.readyState < 2) return; MC.seen = now;
  var mir = $('mcMirror').checked, ms = mir ? -1 : 1, r = MC.lm.detectForVideo(MC.vid, now), tg = {};
  mcDraw(r.landmarks && r.landmarks[0]);
  if (r.worldLandmarks && r.worldLandmarks[0]) tg = mcPoseToTargets(r.worldLandmarks[0], null, { legs: true, mirror: mir });
  if (MC.fl) {      // the face model runs every second frame to save CPU; mirror flips yaw and roll (not pitch)
    if ((MC.fc++ & 1) === 0) {
      try { var fr = MC.fl.detectForVideo(MC.vid, now), m = fr.facialTransformationMatrixes && fr.facialTransformationMatrixes[0];
        if (m) { var h = mcFaceAngles(Array.prototype.slice.call(m.data)); MC.raw.nz = h.yaw; MC.raw.ny = h.pitch; MC.raw.nx = h.roll;
          MC.headTg = {}; MC.headTg[mcKey('head_nz_joint')] = ms * (h.yaw - (MC.off.nz || 0)); MC.headTg[mcKey('head_ny_joint')] = h.pitch - (MC.off.ny || 0); MC.headTg[mcKey('head_nx_joint')] = ms * (h.roll - (MC.off.nx || 0)); } } catch (e) { }
    }
    Object.keys(MC.headTg).forEach(function (k) { tg[k] = MC.headTg[k]; });
  }
  Object.keys(tg).forEach(function (k) { if (/^err/.test(k)) return; k = +k; var v = MC.sm[k] === undefined ? tg[k] : MC.sm[k] * 0.6 + tg[k] * 0.4; MC.sm[k] = v; setTarget(k, v); });
}
function mcStop(keepMsg) { MC.on = false; if (MC.fl) { try { MC.fl.close(); } catch (e) { } MC.fl = null; } MC.headTg = {}; MC.sm = {}; $('mcPrev').hidden = true; var pc = $('mcCv'); if (pc) pc.getContext('2d').clearRect(0, 0, pc.width, pc.height); cancelAnimationFrame(MC.raf); if (MC.stream) MC.stream.getTracks().forEach(function (tr) { tr.stop(); }); MC.stream = null; $('mcBtn').textContent = t('mcStart'); if (!keepMsg) $('mcInfo').textContent = ''; }
$('mcBtn').addEventListener('click', mcStart);
$('mcCal').addEventListener('click', function () { MC.off = {}; Object.keys(MC.raw).forEach(function (k) { MC.off[k] = MC.raw[k]; }); MC.sm = {}; });
$('mcMirror').addEventListener('change', function () { MC.off = {}; MC.sm = {}; mcPrevMirror(); });      // calibration offsets belong to one mapping -> reset
// ---- AI policy: a trained MLP (rl/train.py -> rl/export_policy.py -> policy.json) runs inside the simulator, 50 Hz, same observation layout as the Gymnasium env
var POL = { p: null, on: false, cnt: 0, last: [], k: [], t0: 0, uiT: 0, cmd: 0 };
function polLoad(j) {
  if (!j || j.format !== 'premaid-policy-v1' || j.obs_dim !== 82 || j.act_dim !== 25) throw new Error('not a premaid-policy-v1 file (82 -> 25)');
  var idx = {}; A.servos.forEach(function (sv, k) { idx[sv.joint] = k; }); var ks = j.joint_names.map(function (n) { if (idx[n] === undefined) throw new Error('unknown joint ' + n); return idx[n]; });
  POL.p = j; POL.k = ks; POL.cnt = 0; POL.last = new Array(25).fill(0); POL.t0 = S.simT; $('polInfo').textContent = t('polInfo', j.task, j.layers.length);
}
function polObs() {
  var d = S.d, b = S.base, xq = d.xquat, R = qmat([xq[4 * b], xq[4 * b + 1], xq[4 * b + 2], xq[4 * b + 3]]), p = POL.p, o = new Array(82), i;
  o[0] = d.qvel[3] * 0.25; o[1] = d.qvel[4] * 0.25; o[2] = d.qvel[5] * 0.25; o[3] = -R[6]; o[4] = -R[7]; o[5] = -R[8];      // gravity direction in the base frame
  for (i = 0; i < 25; i++) { var k = POL.k[i]; o[6 + i] = (d.qpos[qadr[k]] - p.pose0[i]) / 0.5; o[31 + i] = d.qvel[qadr[k] - 1] * 0.05; o[56 + i] = POL.last[i]; }
  o[81] = POL.cmd; return o;
}
function polForward(o) {
  var p = POL.p, x = o.map(function (v, i) { return Math.max(-p.obs_clip, Math.min(p.obs_clip, (v - p.obs_mean[i]) / Math.sqrt(p.obs_var[i] + 1e-8))); });
  p.layers.forEach(function (L, li) {
    var y = L.b.slice(); for (var i = 0; i < x.length; i++) { var xi = x[i], row = L.W[i]; for (var j = 0; j < y.length; j++) y[j] += xi * row[j]; }
    x = li < p.layers.length - 1 ? y.map(Math.tanh) : y;
  });
  return x.map(function (v) { return Math.max(-1, Math.min(1, v)); });
}
function polStep() {
  if (S.simT < POL.t0 - 0.01) { POL.cnt = 0; POL.last = new Array(25).fill(0); } POL.t0 = S.simT;
  if (POL.cnt++ % 10) return;                                      // 10 physics steps (0.02 s) per control step
  var a = polForward(polObs()), p = POL.p; POL.last = a;
  for (var i = 0; i < 25; i++) target[POL.k[i]] = (p.pose0[i] + p.action_scale * a[i]) / D2R;
}
function polPose() {       // start from the pose the policy was trained from (keyframe arms_down) with the robot standing on the floor
  var m = S.m, d = S.d, key = 1; reset(); for (var i = 0; i < m.nq; i++) d.qpos[i] = m.key_qpos[key * m.nq + i]; d.qpos[2] += 0.002; for (var k = 0; k < N; k++) target[k] = m.key_ctrl[key * m.nu + k] / D2R;
  for (var v = 0; v < m.nv; v++) d.qvel[v] = 0; S.d.ctrl.set(m.key_ctrl.subarray(key * m.nu, (key + 1) * m.nu)); S.simT = 0; POL.cnt = 0; POL.last = new Array(25).fill(0); S.mj.mj_forward(m, d);
}
$('polLoad').addEventListener('click', function () { fetch('policies/stand.json').then(function (r) { if (!r.ok) throw new Error('policies/stand.json: HTTP ' + r.status); return r.json(); }).then(polLoad).catch(function (e) { $('polInfo').textContent = t('polErr', e.message); }); });
$('polFileBtn').addEventListener('click', function () { $('polFile').click(); });
$('polFile').addEventListener('change', function () { var f = this.files[0]; if (!f) return; f.text().then(function (x) { polLoad(JSON.parse(x)); }).catch(function (e) { $('polInfo').textContent = t('polErr', e.message); }); });
$('polOn').addEventListener('change', function () { if (this.checked && !POL.p) { this.checked = false; $('polInfo').textContent = t('polNone'); return; } POL.on = this.checked; if (POL.on) { BAL.on = false; $('balOn').checked = false; BAL.o = []; } });
$('polPose').addEventListener('click', polPose);
// ---- keyboard shortcuts + help
var VIEW_KEYS = ['front', 'back', 'left', 'right', 'top', 'iso'];
function seekBy(ms) { if (!P.frames) return; P.t = Math.max(0, Math.min(P.total, P.t + ms)); poseAt(P.t); updatePlayUI(true); }
function stepFrame(dir) { if (!P.frames) return; pausePlay(); var k = findK(P.t) + dir; k = Math.max(0, Math.min(P.frames.length - 1, k)); P.t = P.T[k]; poseAt(P.t); updatePlayUI(true); }
function kbHelp(on) { var e = $('kbHelp'); e.hidden = on === undefined ? !e.hidden : !on; }
document.addEventListener('keydown', function (e) {
  var tag = e.target && e.target.tagName; if (e.ctrlKey || e.metaKey || e.altKey) return;
  if (e.key === 'Escape') { kbHelp(false); return; }
  if (/INPUT|TEXTAREA|SELECT/.test(tag) || (e.target && e.target.isContentEditable)) return;
  var k = e.key.length === 1 ? e.key.toLowerCase() : e.key;
  if (k === 'k') { toggleId('teleOn'); e.preventDefault(); return; }
  if (TL.on) { if ('wasdjliuoqe'.indexOf(k) >= 0 && k.length === 1) { TL.keys[k] = true; e.preventDefault(); return; } }
  if (WK.on) return;
  if (tag === 'BUTTON' && (k === ' ' || k === 'Enter')) return;
  var done = true;
  if (k === ' ') clickId('pPlay'); else if (k === 'ArrowLeft') seekBy(e.shiftKey ? -5000 : -1000); else if (k === 'ArrowRight') seekBy(e.shiftKey ? 5000 : 1000);
  else if (k === ',') stepFrame(-1); else if (k === '.') stepFrame(1); else if (k === 'r') clickId('bReset'); else if (k === 'h') toggleId('cPin'); else if (k === 'i') toggleId('cIK');
  else if (k === 'c') toggleId('cCom'); else if (k === 'v') toggleId('cSen'); else if (k === 't') toggleId('cTrail'); else if (k === 'l') toggleId('pLoop'); else if (k === 'm') toggleId('aOn');
  else if (k === 'b') toggleId('balOn'); else if (/^[1-6]$/.test(k)) view(VIEW_KEYS[+k - 1]); else if (k === 'f') fsToggle(); else if (k === '?' || k === '/') kbHelp();
  else if (k === '[') abSet('a'); else if (k === ']') abSet('b'); else if (k === '\\') abClear(); else if (k === 'n') plNext(1); else if (k === 'p') plNext(-1); else if (k === 's') clickId('shBtn'); else if (k === 'g') clickId('gifBtn'); else if (k === 'x') clickId('recBtn');
  else done = false;
  if (done) e.preventDefault();
});
document.addEventListener('keyup', function (e) { var k = e.key.length === 1 ? e.key.toLowerCase() : e.key; delete TL.keys[k]; });
$('kbHelp').addEventListener('click', function () { kbHelp(false); });
// ---- PWA: manifest + service worker (offline), registered only over http(s)
if ('serviceWorker' in navigator && /^https?:$/.test(location.protocol)) navigator.serviceWorker.register('sw.js').then(function () { $('pwaInfo').textContent = t('pwaOn'); }, function () { $('pwaInfo').textContent = t('pwaOff'); });
else $('pwaInfo').textContent = t('pwaOff');
