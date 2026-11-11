// ------------------------------------------------------------------ real robot over Web Serial (packets: neon-izm/PremaindAI_TechVerification)
// packet = [LEN][CMD]...[XOR of all previous bytes]; 0x18 pose = [LEN][18][00][SPD]{pid, lo, hi}*n ; 0x01 = battery (V = u16/216)
var SAFE = {2:[2500,7500],3:[7300,8000],4:[7500,12500],5:[6600,8400],6:[7000,7900],7:[7200,7800],8:[7000,8000],9:[7500,9700],10:[7000,7700],11:[5200,7300],12:[7300,8000],13:[5000,8000],14:[5000,9000],15:[7000,10000],16:[6000,10000],17:[7400,11000],18:[5000,7550],19:[4000,7600],20:[7450,9000],21:[6000,12000],22:[5800,8200],23:[3000,9000],24:[6800,9200],26:[6700,8700],28:[6300,8300]};
var RB = { port: null, writer: null, reader: null, rp: null, bt: 0, buf: [], volt: null, last: 0, sig: '', lastPkt: null, retry: 0, lastK: -1, lastF: null, paramKey: '', motion: '', msg: 'rbOff', err: '', perr: false };
var PIDK = A.servos.map(function (s, k) { return k; }).sort(function (a, b) { return A.servos[a].pid - A.servos[b].pid; });
function xr(a) { var x = 0; for (var i = 0; i < a.length; i++) x ^= a[i]; return x & 255; }
function mk(body) { var b = body.slice(); b.push(xr(b)); return Uint8Array.from(b); }
function hexPkt(s) { return Uint8Array.from(s.trim().split(/\s+/).map(function (h) { return parseInt(h, 16); })); }
function rbSend(u8, keep) { if (!RB.writer) return false; if (keep) RB.lastPkt = u8; rbLog('TX', u8); RB.writer.write(u8).catch(function (e) { console.warn(e); }); return true; }
function rawOf(k, deg) {
  var s = A.servos[k], e = PM.byJoint[s.joint], r = Math.round(7500 + s.sign * deg / K_DEG), lo = e.rawLo, hi = e.rawHi, sf = SAFE[s.pid];
  if (sf && $('rbSafe').checked) { lo = Math.max(lo, sf[0]); hi = Math.min(hi, sf[1]); }
  return Math.max(lo, Math.min(hi, r));
}
function poseBytes(raw, spd) {
  var b = [5 + 3 * N, 0x18, 0, Math.max(1, Math.min(255, spd | 0))];
  PIDK.forEach(function (k) { var v = raw[k]; b.push(A.servos[k].pid, v & 255, (v >> 8) & 255); });
  return mk(b);
}
function targetsRaw() { var r = []; for (var k = 0; k < N; k++) r[k] = rawOf(k, target[k]); return r; }
function rbSpd() { return parseInt($('rbSp').value, 10) || 10; }
function rbPose() { var r = targetsRaw(); RB.sig = r.join(','); RB.retry = 0; rbSend(poseBytes(r, rbSpd()), true); }
function rbParamPacket(type, values) {   // [LEN][19][00 speed | 10 stretch]{pid, value}...; only 1..127 are valid
  var pairs = [];
  for (var k = 0; k < N; k++) {
    var v = values[k] | 0;
    if (v >= 1 && v <= 127) pairs.push(A.servos[k].pid, v);
  }
  if (!pairs.length) return false;
  return rbSend(mk([4 + pairs.length, 0x19, type].concat(pairs)));
}
function rbParam(stretch) {
  var v = Math.max(1, Math.min(127, parseInt($(stretch ? 'rbStr' : 'rbSpd').value, 10) || 60)), vals = [];
  for (var k = 0; k < N; k++) vals[k] = v;
  rbParamPacket(stretch ? 0x10 : 0x00, vals);
}
function rbBattery() { rbSend(hexPkt('07 01 00 02 00 02 06')); }
var BUILTIN = [
  { id: 0x3D, jp: 'ホーム' }, { id: 0x3E, jp: 'どうぞ' }, { id: 0x3F, jp: 'ワクワク' }, { id: 0x40, jp: 'コシニテ' },
  { id: 0x41, jp: 'ようこそ' }, { id: 0x42, jp: 'おねがい' }, { id: 0x43, jp: 'バイバイ' }, { id: 0x44, jp: '右にキス' },
  { id: 0x45, jp: '左にキス' }, { id: 0x46, jp: 'ピストル' }, { id: 0x47, jp: '敬礼' }, { id: 0x48, jp: 'えーん' },
  { id: 0x49, jp: 'ハート' }, { id: 0x4A, jp: 'キラッ' }, { id: 0x4B, jp: 'あなたへ' }, { id: 0x4C, jp: 'もんきー' },
  { id: 0x4D, jp: 'GO×2' }, { id: 0x4E, jp: 'エアギター' }, { id: 0x4F, jp: '右ターン' }, { id: 0x50, jp: '左ターン' }
];
function renderBuiltin() {       // 20 buttons, one tap = play that built-in motion (0x1F)
  var box = $('rbBuiltinGrid'); if (!box) return; var names = t('rbBuiltinNames'); box.innerHTML = '';
  BUILTIN.forEach(function (it, i) {
    var b = document.createElement('button'); b.className = 'rbc'; b.textContent = names[i] || it.jp; b.title = (i + 1) + '. ' + it.jp + '  (0x' + h2(it.id) + ')'; b.disabled = !RB.writer;
    b.addEventListener('click', function () { if (!RB.writer) { renderRb(); return; } rbMotionSend(it.id); }); box.appendChild(b);
  });
}
function rbMotionSend(id) {
  $('rbLive').checked = false; $('rbStream').checked = false;
  P.playing = false; updatePlayUI(true);
  RB.lastF = RB.lastK = null;
  RB.motion = 'rbMotionSent'; renderRb();
  rbSend(mk([0x05, 0x1F, 0x00, id & 0xFF]));
}
function rbStopMotion() {
  $('rbLive').checked = false; $('rbStream').checked = false;
  P.playing = false; updatePlayUI(true);
  RB.lastF = RB.lastK = null;
  RB.motion = 'rbMotionStopped'; renderRb();
  rbStopPackets();
}
function rbRx(u8) {
  for (var i = 0; i < u8.length; i++) RB.buf.push(u8[i]);
  while (RB.buf.length) {
    var L = RB.buf[0]; if (L === 0 || L < 4) { RB.buf.shift(); continue; }
    if (RB.buf.length < L) { if (RB.buf.length > 200) RB.buf.length = 0; break; }
    var p = RB.buf.splice(0, L); rbLog('RX', p);
    if (p[1] === 0x01 && L === 18) stRx(p);
    else if (p[1] === 0x01 && L === 6) { RB.volt = (p[3] | (p[4] << 8)) / 216; bvPush(RB.volt); }
    else if (p[1] === 0x1F && L >= 4) { RB.motion = (p[2] & 0x80) ? 'rbMotionErr' : 'rbMotionOk'; }
    else if (p[1] === 0x18) {
      if (p[2] === 0x08 && RB.lastPkt && RB.retry < 20) { RB.retry++; RB.perr = true; setTimeout(function () { rbSend(RB.lastPkt); }, 5); }   // reference code re-sends the last pose on "04 18 08 14"
      else if (p[2] === 0) { RB.retry = 0; RB.perr = false; }
    }
  }
  renderRb();
}
function renderRb() {
  var on = !!RB.writer;
  Array.prototype.forEach.call(document.querySelectorAll('[data-rbmode]'), function (b) { var m = b.getAttribute('data-rbmode'); b.classList.toggle('on', m === 'live' ? $('rbLive').checked : m === 'stream' ? $('rbStream').checked : !$('rbLive').checked && !$('rbStream').checked); });
  $('rbState').textContent = RB.msg === 'rbErr' ? t('rbErr') + RB.err : t(RB.msg) + (RB.perr ? ' · ' + t('rbPoseErr') : '');
  $('rbMotion').textContent = RB.motion ? t(RB.motion) : '';
  $('rbMotion').classList.toggle('warn', RB.motion === 'rbMotionErr');
  $('rbConn').textContent = t(on ? 'rbDisc' : 'rbConn');
  $('rbVolt').textContent = RB.volt == null ? '' : t('rbVolt', RB.volt) + (RB.volt < 9 ? ' ' + t('rbLow') : '');
  Array.prototype.forEach.call(document.querySelectorAll('.rbc'), function (el) { el.disabled = !on; });
  rbRender2(on);
}
function rbTick(now) {       // live mirror of the slider targets (not while a PMA is streamed), max ~12 packets/s
  if (!RB.writer || !$('rbLive').checked || (P.playing && $('rbStream').checked) || now - RB.last < 80) return;
  var s = targetsRaw().join(','); if (s === RB.sig) return; RB.last = now; rbPose();
}
function rbStream() {        // send each PMA frame to the robot when it starts; the robot interpolates with the frame's own SPD
  if (!RB.writer || !$('rbStream').checked || !P.frames) return;
  if (RB.lastF !== P.frames) { RB.lastF = P.frames; RB.lastK = -1; RB.paramKey = ''; }
  var k = findK(P.t); if (k === RB.lastK) return; RB.lastK = k;
  var f = P.frames[k], raw = []; for (var s = 0; s < N; s++) raw[s] = f.free[s] ? 0 : ($('rbSafeStream').checked ? rawClampSafe(s, f.raw[s]) : f.raw[s]);
  var key = f.speed.join(',') + '|' + f.stretch.join(',');
  if (key !== RB.paramKey) {           // 0x19 parameters changed at this keyframe: send them before the pose
    RB.paramKey = key;
    rbParamPacket(0x10, f.stretch);
    rbParamPacket(0x00, f.speed);
  }
  RB.retry = 0; rbSend(poseBytes(raw, f.spd || 1), true);
}
function rbExport() {
  var sec = P.secs[+$('pSec').value || 0]; if (!sec) return;
  var n = Math.max(2, parseInt($('rbThin').value, 10) || 2), b = sec.bytes, pk = [], i = 0, ix = [], keep = {}, out = [];
  while (i < b.length) { var L = b[i]; if (!L || L === 255 || i + L > b.length) break; pk.push(b.subarray(i, i + L)); i += L; }
  pk.forEach(function (p, j) { if (p[1] === 0x18 && p.length >= 7) ix.push(j); });
  ix.forEach(function (j, q) { if (q % n === 0 || q === ix.length - 1) keep[j] = 1; });
  pk.forEach(function (p, j) { if (p[1] !== 0x18 || keep[j]) out.push(p); });
  var hx = out.map(function (p) { return Array.prototype.map.call(p, function (x) { return ('0' + x.toString(16).toUpperCase()).slice(-2); }).join(' '); }).join(' ');
  var txt = '\uFEFF[モーション(' + (sec.idx || 1) + ')]\r\n番号=' + (sec.num || 1) + '\r\n名前=' + (sec.name || '') + '\r\nデータ=' + hx + '\r\n';
  var a = document.createElement('a'); a.href = URL.createObjectURL(new Blob([txt], { type: 'text/plain' })); a.download = (P.file || 'motion').replace(/\.pma$/i, '') + '_thin' + n + '.pma'; a.click(); setTimeout(function () { URL.revokeObjectURL(a.href); }, 1000);
}
$('rbConn').addEventListener('click', function () { RB.writer ? rbDisconnect() : rbConnect(); });
$('rbOnce').addEventListener('click', rbPose);
$('rbRel').addEventListener('click', function () { $('rbLive').checked = false; $('rbStream').checked = false; RB.motion = ''; var z = []; for (var k = 0; k < N; k++) z[k] = 0; RB.retry = 0; rbSend(poseBytes(z, 2), true); });
$('rbT').addEventListener('click', rbStopMotion);
$('rbStopMotion').addEventListener('click', function () { if (!RB.writer) { renderRb(); return; } rbStopMotion(); });
$('rbStrB').addEventListener('click', function () { rbParam(true); });
$('rbSpdB').addEventListener('click', function () { rbParam(false); });
$('rbBat').addEventListener('click', rbBattery);
$('rbExp').addEventListener('click', rbExport);
