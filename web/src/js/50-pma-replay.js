// ------------------------------------------------------------------ .pma replay (PreMaid AI motion files, see premaiduino2/PmaProtocol.h)
// file = text, "[モーション(n)] / 番号= / 名前= / データ=<hex bytes>"; bytes = packets [LEN][CMD]...[XOR]; 0x18 = {pma_id, raw lo, raw hi} per servo
// (raw 0 = release), payload[1] = SPD, 1 tick = 15 ms = time the firmware waits before the next packet (and never less than the servo's full-speed travel).
var K_DEG = 270 / 8000, PULSE_PER_MS = (60 / 140) * (8000 / 270);
var PM = { byJoint: {}, pid2k: {} };
A.pma.forEach(function (e) { PM.byJoint[e.joint] = e; });
A.servos.forEach(function (s, k) { var e = PM.byJoint[s.joint]; s.pid = e.pid; s.sign = e.sign; s.flip = false; PM.pid2k[e.pid] = k; });
function parsePmaText(text) {
  if (text.charCodeAt(0) === 0xFEFF) text = text.slice(1);
  var re = /\[モーション\((\d+)\)\][\r\n]+番号=(\d+)[\r\n]+名前=([^\r\n]*)[\r\n]+データ=([0-9A-Fa-f ]+)/g, out = [], m;
  while ((m = re.exec(text))) {
    var hx = m[4].trim().split(/\s+/), b = new Uint8Array(hx.length);
    for (var i = 0; i < hx.length; i++) b[i] = parseInt(hx[i], 16);
    out.push({ name: m[3].trim(), bytes: b, idx: m[1], num: m[2] });
  }
  return out;
}
function decodePma(buf) {     // UTF-8 (with BOM) first, Shift-JIS as fallback for older files
  var r = parsePmaText(new TextDecoder('utf-8').decode(buf));
  if (!r.length) { try { r = parsePmaText(new TextDecoder('shift_jis').decode(buf)); } catch (e) {} }
  return r;
}
var P = { name: '', frames: null, T: [], D: [], total: 0, t: 0, playing: false, loop: true, loopMode: 'orig', interp: true, freeNow: new Uint8Array(N), dirty: false, uiT: 0, secs: [], secIdx: 0, state: null, infoD: null, bpm: 0 };

function cloneFrame(f) { return { raw: f.raw.slice(), free: f.free.slice(), spd: f.spd, speed: f.speed.slice(), stretch: f.stretch.slice() }; }
function buildTimeline(bytes, once) {
  var cur = new Int32Array(N).fill(7500), free = new Uint8Array(N), speed = new Uint8Array(N), stretch = new Uint8Array(N).fill(60), frames = [], loops = [], stack = [], bad = 0, other = 0, rawN = 0, i = 0;
  while (i < bytes.length) {
    var L = bytes[i]; if (L === 0 || L === 0xFF || i + L > bytes.length) break;
    var x = 0; for (var j = 0; j < L - 1; j++) x ^= bytes[i + j];
    if (x !== bytes[i + L - 1]) bad++;
    if (bytes[i + 1] === 0x18 && L >= 7) {
      for (var q = 4; q + 2 < L - 1; q += 3) {
        var k = PM.pid2k[bytes[i + q]]; if (k === undefined) continue;
        var v = bytes[i + q + 1] | (bytes[i + q + 2] << 8);
        if (v === 0) free[k] = 1; else { free[k] = 0; cur[k] = v; }
      }
      frames.push({ raw: Array.prototype.slice.call(cur), free: free.slice(), spd: bytes[i + 3], speed: speed.slice(), stretch: stretch.slice() });
      rawN++;
    } else if (bytes[i + 1] === 0x19 && L >= 6) {
      // [LEN][19][00 speed | 10 stretch][pid, value]...  (smaller = slower/softer, larger = faster/harder)
      var typ = bytes[i + 2];
      for (var p = 3; p + 1 < L - 1; p += 2) {
        var pk = PM.pid2k[bytes[i + p]]; if (pk === undefined) continue;
        if (typ === 0x00) speed[pk] = bytes[i + p + 1];
        else if (typ === 0x10) stretch[pk] = bytes[i + p + 1];
      }
    } else if (bytes[i + 1] === 0x02 && L === 8) {
      // loop start: start = previous keyframe (TechVerification MotionDataLoader.cs), iterations = payload[5]
      stack.push({ start: Math.max(0, frames.length - 1), iterations: Math.max(1, bytes[i + 5] || 1), depth: stack.length + 1 });
    } else if (bytes[i + 1] === 0x07 && L === 8) {
      // loop end: duplicate the frames from start..end (iterations - 1) times, nested loops included
      if (stack.length) {
        var lp = stack.pop(); lp.end = frames.length - 1; loops.push(lp);
        if (!once && lp.iterations > 1 && lp.end >= lp.start) {
          var base = frames.slice(lp.start, lp.end + 1);
          for (var r = 1; r < lp.iterations; r++) for (var b = 0; b < base.length; b++) frames.push(cloneFrame(base[b]));
        }
      }
    } else other++;
    i += L;
  }
  var T = [], D = [], t = 0, repeats = 0, maxDepth = 0;
  loops.forEach(function (lp) { repeats += Math.max(0, lp.iterations - 1); maxDepth = Math.max(maxDepth, lp.depth); });
  frames.forEach(function (f, n) {
    var pf = n ? frames[n - 1] : f, md = 0;
    for (var s = 0; s < N; s++) if (!f.free[s]) md = Math.max(md, Math.abs(f.raw[s] - pf.raw[s]));
    var d = Math.max(Math.max(f.spd, 1) * 15, md / PULSE_PER_MS);     // same rule as motionPlaybackUpdate()
    T.push(t); D.push(d); t += d;
  });
  return { frames: frames, T: T, D: D, total: t, bad: bad, other: other, rawN: rawN, loops: loops, loopRepeats: repeats, maxLoopDepth: maxDepth };
}
function findK(t) { var lo = 0, hi = P.frames.length - 1; while (lo < hi) { var mid = (lo + hi + 1) >> 1; if (P.T[mid] <= t) lo = mid; else hi = mid - 1; } return lo; }
function poseAt(t) {
  var fs = P.frames, k = findK(t), f = fs[k], pf = k ? fs[k - 1] : f, a = 1, pd = false;
  if (P.interp && P.D[k] > 0) a = Math.min(1, Math.max(0, (t - P.T[k]) / P.D[k]));
  for (var s = 0; s < N; s++) {
    var raw = pf.raw[s] + (f.raw[s] - pf.raw[s]) * a, fr = f.free[s];
    if (P.freeNow[s] !== fr) { P.freeNow[s] = fr; pd = true; }
    if (S.svSpeed[s] !== f.speed[s]) { S.svSpeed[s] = f.speed[s]; }
    if (S.svStretch[s] !== f.stretch[s]) { S.svStretch[s] = f.stretch[s]; pd = true; }
    setTarget(s, A.servos[s].sign * (raw - 7500) * K_DEG);          // clamped to the servo's travel
  }
  if (pd) applyGains();
}
function rangeOfFrames(frames, field, ignoreZero) {
  var lo = 0, hi = 0, any = false;
  frames.forEach(function (f) { for (var s = 0; s < N; s++) { var v = f[field][s]; if (ignoreZero && !v) continue; if (!any) { lo = hi = v; any = true; } else { if (v < lo) lo = v; if (v > hi) hi = v; } } });
  return any ? [lo, hi] : null;
}
function renderPma() {
  var st = P.state, d = P.infoD; renderCheck('chk', P.frames && P.frames.length ? P : null);
  $('pmaState').textContent = !st ? t('notLoaded') : st.k ? t(st.k) : st.raw;
  if (!d) { $('pInfo').textContent = ''; return; }
  if (d.k) { $('pInfo').textContent = t(d.k); return; }
  var txt = t('pmaInfo', d.n, d.rawN, d.sec, d.loops, d.loopRepeats, d.bpm) + ' · ' + d.bytes + ' bytes' + (d.bad ? ' · ' + t('csBad', d.bad) : ' · checksum OK') + (d.other ? ' · ' + t('skipped', d.other) : '');
  if (d.stretch || d.speed) txt += ' · ' + t('pmaParams', d.stretch, d.speed);
  $('pInfo').textContent = txt;
}
function fmt(t) { t /= 1000; var m = Math.floor(t / 60); return m + ':' + (t - m * 60).toFixed(1).padStart(4, '0'); }
function updatePlayUI(force) {
  var now = performance.now(); if (!force && now - P.uiT < 100) return; P.uiT = now;
  $('pPlay').textContent = P.playing ? t('pause') : t('play'); $('pPlay').classList.toggle('on', P.playing);
  if (!P.frames) return;
  $('pTime').textContent = fmt(P.t) + ' / ' + fmt(P.total) + ' · ' + t('frame') + ' ' + (findK(P.t) + 1) + '/' + P.frames.length;
  if (document.activeElement !== $('pSeek')) $('pSeek').value = P.total > 0 ? Math.round(P.t / P.total * 1000) : 0;
}
function pausePlay() { if (P.playing) { P.playing = false; updatePlayUI(true); } }
function playTick(dt) {          // dt in seconds (sim time); the timeline is in ms
  P.t += dt * 1000;
  if (AB.a >= 0 && AB.b > AB.a && P.t >= AB.b) P.t = AB.a + (P.t - AB.b);
  if (P.t >= P.total) { if (PL.auto && LIB.cur >= 0 && !REC.once) { P.t = P.total; P.playing = false; plNext(1); } else if (P.loop) P.t -= P.total; else { P.t = P.total; P.playing = false; } }
  poseAt(P.t); updatePlayUI(!P.playing); rbStream();
}
function useSection(i, keepT) {
  var sec = P.secs[i]; if (!sec) return;
  var oldT = P.t, tl = buildTimeline(sec.bytes, P.loopMode === 'once');
  if (!tl.frames.length) { P.state = { k: 'errNo18' }; P.infoD = null; renderPma(); return; }
  P.secIdx = i; P.frames = tl.frames; P.T = tl.T; P.D = tl.D; P.total = tl.total; P.t = keepT ? Math.min(oldT, tl.total) : 0; P.name = sec.name || P.file || '';
  P.state = { raw: P.name };
  P.infoD = {
    n: tl.frames.length, rawN: tl.rawN, sec: tl.total / 1000, bytes: sec.bytes.length, bad: tl.bad, other: tl.other,
    loops: tl.loops.length, loopRepeats: tl.loopRepeats, maxLoopDepth: tl.maxLoopDepth, bpm: P.bpm,
    stretch: rangeOfFrames(tl.frames, 'stretch'), speed: rangeOfFrames(tl.frames, 'speed', true)
  };
  renderPma();
  ['pPlay', 'pStop', 'pSeek'].forEach(function (id) { $(id).disabled = false; });
  $('pLoopRow').style.display = ''; $('pLoopMode').value = P.loopMode;
  poseAt(P.t); updatePlayUI(true);
}
function loadSections(secs, file, bpm) {
  if (!secs.length) { P.state = { k: 'errFmt' }; P.infoD = { k: 'errFmtInfo' }; renderPma(); return; }
  P.playing = false; P.secs = secs; P.file = file || ''; P.bpm = +bpm || 0; var sel = $('pSec'); sel.innerHTML = '';
  secs.forEach(function (c, i) { var o = document.createElement('option'); o.value = i; o.textContent = (i + 1) + '. ' + (c.name || '(no name)'); sel.appendChild(o); });
  sel.style.display = secs.length > 1 ? '' : 'none'; useSection(0); if (typeof AB !== 'undefined') { AB.a = AB.b = -1; abRender(); }
}
function loadFile(f, af) { if (!f) return; var r = new FileReader(); r.onload = function () { markActive(-1); LIB.cur = -1; audioSet('', '', ''); loadSections(decodePma(r.result), f.name, 0); if (af) audioFile(af); }; r.readAsArrayBuffer(f); }
function loadFiles(fs) {      // any mix of .pma + audio (+ encrypted .wsdcf, which only triggers the explanation)
  var pm = null, au = null, dr = '', AUD = /\.(mp3|m4a|aac|ogg|opus|wav|flac)$/i; Array.prototype.forEach.call(fs, function (f) { if (/\.wsdcf$/i.test(f.name)) dr = f.name; else if (AUD.test(f.name) || /^audio\//.test(f.type)) au = au || f; else pm = pm || f; });
  if (pm) loadFile(pm, au); else if (au) audioFile(au); else if (dr) { audioSet('', '', dr); }
}
function b64bytes(b) { var bin = atob(b), u = new Uint8Array(bin.length); for (var i = 0; i < bin.length; i++) u[i] = bin.charCodeAt(i); return u; }
$('fPma').addEventListener('change', function () { loadFiles(this.files); this.value = ''; });
function refreshList() {}      // the library box shows names only
function markActive(i) { Array.prototype.forEach.call($('pList').children, function (b, n) { b.classList.toggle('on', n === i); }); }
var LIB = { base: '../pma/', items: [], cur: -1 };      // window.PMA_LIBRARY from pma/library.js (written by pma/make_index.py, a plain <script>, so it also works from file://): { title, pma, audio?, drm?, data (base64 .pma) }
var AU = { el: new Audio(), name: '', drm: '', err: false, obj: '' };
AU.el.preload = 'auto'; AU.el.volume = 0.8;
function audioSet(src, name, drm) {
  var a = AU.el; a.pause(); if (AU.obj && AU.obj !== src) { URL.revokeObjectURL(AU.obj); AU.obj = ''; }
  AU.name = name || ''; AU.drm = drm || ''; AU.err = false;
  if (src) { a.src = src; a.load(); } else { a.removeAttribute('src'); a.load(); }
  renderAudio();
}
function audioFile(f) { if (!f) return; var u = URL.createObjectURL(f); audioSet(u, f.name, ''); AU.obj = u; }
function renderAudio() {
  var has = !!AU.el.getAttribute('src');
  $('audName').textContent = has ? '🎵 ' + AU.name : t('audNone');
  var note = AU.err ? t('audErr') : (!has && AU.drm ? t('audDrm') : '');
  $('aOn').disabled = !has;
}
AU.el.addEventListener('error', function () { if (AU.el.getAttribute('src')) { AU.err = true; renderAudio(); } });
function audioSync() {      // music follows the motion clock: plays only while the motion plays; re-seeks when it drifts (>0.25 s), loops, seeks or changes speed
  var a = AU.el; if (!a.getAttribute('src') || AU.err) return;
  if (!(P.playing && S.run && P.frames)) { if (!a.paused) a.pause(); return; }
  var want = P.t / 1000 + (+$('aOff').value) / 1000;
  if (want < 0 || (a.duration && want >= a.duration)) { if (!a.paused) a.pause(); return; }
  if (a.playbackRate !== S.speed) a.playbackRate = S.speed;
  if (a.paused) { a.currentTime = want; a.play().catch(function () {}); }
  else if (Math.abs(a.currentTime - want) > 0.25) a.currentTime = want;
}
$('fAud').addEventListener('change', function () { audioFile(this.files[0]); this.value = ''; });
$('aVol').addEventListener('input', function () { AU.el.volume = +this.value; $('aVolV').textContent = Math.round(this.value * 100); });
$('aOff').addEventListener('input', function () { $('aOffV').textContent = this.value + ' ms'; });
$('aOn').addEventListener('change', function () { AU.el.muted = !this.checked; });
function libFetch(it) {      // the .pma comes base64-encoded inside pma/library.js, so no fetch() (blocked on file://) is needed
  if (it.secs) return Promise.resolve(it);
  try { var bin = atob(it.data), u = new Uint8Array(bin.length); for (var i = 0; i < bin.length; i++) u[i] = bin.charCodeAt(i); it.secs = decodePma(u.buffer); } catch (e) { return Promise.reject(e); }
  return it.secs.length ? Promise.resolve(it) : Promise.reject(new Error('format'));
}
function libLoad(i, after) {
  var it = LIB.items[i]; if (!it) return;
  libFetch(it).then(function () {
    loadSections(it.secs, it.pma, it.bpm); markActive(i); LIB.cur = i;
    audioSet(it.audio ? LIB.base + encodeURIComponent(it.audio) : '', it.audio || '', it.drm || '');
    $('pPlay').click(); if (after) after();
  }, function () { P.state = { k: 'errFmt' }; P.infoD = { k: 'errFmtInfo' }; renderPma(); });
}
(function () {
  var box = $('pList');
  function msg(key) { box.innerHTML = ''; var d = document.createElement('div'); d.className = 'sub'; d.style.padding = '8px'; d.setAttribute('data-i18n', key); d.textContent = t(key); box.appendChild(d); }
  var list = window.PMA_LIBRARY; if (!list) { msg('libErr'); return; } if (!list.length) { msg('libEmpty'); return; }
  LIB.items = list;
  list.forEach(function (it, i) {
    var b = document.createElement('button'); b.innerHTML = '<span></span>'; b.firstChild.textContent = it.title;
    b.addEventListener('click', function () { libLoad(i); }); box.appendChild(b);
  });
})();
$('pSec').addEventListener('change', function () { P.playing = false; useSection(+this.value); });
$('pPlay').addEventListener('click', function () {
  if (!P.frames) return;
  if (P.playing) P.playing = false;
  else { if (SW.on) stopSweep(); if (P.t <= 0.001 || P.t >= P.total - 0.001) { P.t = 0; poseAt(0); if (S.ready) reset(); } P.playing = true; }
  updatePlayUI(true);
});
$('pStop').addEventListener('click', function () { if (!P.frames) return; P.playing = false; P.t = 0; poseAt(0); updatePlayUI(true); });
$('pSeek').addEventListener('input', function () { if (!P.frames) return; P.t = this.value / 1000 * P.total; poseAt(P.t); updatePlayUI(true); });
$('pLoop').addEventListener('change', function () {
  P.loop = this.checked;
  if (P.loop && P.loopMode === 'once') { P.loopMode = 'orig'; $('pLoopMode').value = 'orig'; if (P.frames && P.secs.length) useSection(P.secIdx, true); }
});
$('pLoopMode').addEventListener('change', function () {
  P.loopMode = this.value;
  if (P.loopMode === 'once') { P.loop = false; $('pLoop').checked = false; }
  if (P.frames && P.secs.length) useSection(P.secIdx, true);
  updatePlayUI(true);
});
$('pInterp').addEventListener('change', function () { P.interp = this.value === '1'; if (P.frames) poseAt(P.t); });
['dragenter', 'dragover'].forEach(function (ev) { $('stage').addEventListener(ev, function (e) { e.preventDefault(); $('stage').classList.add('drag'); }); });
['dragleave', 'drop'].forEach(function (ev) { $('stage').addEventListener(ev, function (e) { e.preventDefault(); $('stage').classList.remove('drag'); if (ev === 'drop' && e.dataTransfer.files.length) loadFiles(e.dataTransfer.files); }); });
function setFlip(k, on) {      // flip one servo's direction: sign, travel (and the live MuJoCo joint/ctrl range) are mirrored
  var s = A.servos[k]; on = !!on; if (!!s.flip === on) return;
  var lo = s.lo, hi = s.hi; s.lo = -hi; s.hi = -lo; s.flip = on; s.sign = -s.sign;
  if (S.m) { var m = S.m; m.jnt_range[2 * jid[k]] = s.lo; m.jnt_range[2 * jid[k] + 1] = s.hi; m.actuator_ctrlrange[2 * act[k]] = s.lo; m.actuator_ctrlrange[2 * act[k] + 1] = s.hi; }
  var u = ui[k]; u.range.min = u.num.min = (s.lo / D2R).toFixed(2); u.range.max = u.num.max = (s.hi / D2R).toFixed(2);
  setTarget(k, -target[k]); if (P.frames) poseAt(P.t);
}

