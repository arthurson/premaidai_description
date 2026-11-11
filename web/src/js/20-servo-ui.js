// ------------------------------------------------------------------ servo UI
function clampDeg(k, v) { var s = A.servos[k], lo = s.lo / D2R, hi = s.hi / D2R; v = +v; if (!isFinite(v)) v = 0; return Math.min(hi, Math.max(lo, v)); }
function setTarget(k, v, fromUi) {
  v = clampDeg(k, v); target[k] = v; var u = ui[k];
  if (fromUi !== 'range') u.range.value = v; if (fromUi !== 'num') u.num.value = v.toFixed(1);
}
(function buildServoUI() {
  var host = $('servos'), idx = 0;
  GROUPS.forEach(function (g) {
    var h = document.createElement('h2'); h.innerHTML = '<span></span> <small></small>'; host.appendChild(h);
    var n = 0, ge = { key: g, h: h, n: 0 }; groupEls.push(ge);
    A.servos.forEach(function (s, k) {
      if (s.group !== g) return; n++;
      var lo = s.lo / D2R, hi = s.hi / D2R, el = document.createElement('div'); el.className = 'servo';
      el.title = s.joint + '  axis ' + s.axis.join(' ') + '  range ' + lo.toFixed(1) + '…' + hi.toFixed(1) + '°  effort ' + s.effort + ' N·m';
      el.innerHTML = '<div class="name"><span class="lbl"></span><span class="mono">' + s.joint.replace('_joint', '') + '</span></div>' +
        '<input type="range" min="' + lo.toFixed(2) + '" max="' + hi.toFixed(2) + '" step="0.1" value="0">' +
        '<input type="number" min="' + lo.toFixed(2) + '" max="' + hi.toFixed(2) + '" step="0.5" value="0.0">' +
        '<button class="sm" data-i18n-title="tipZero">0</button><div class="tq"><i></i></div>';
      host.appendChild(el);
      var r = el.querySelector('input[type=range]'), nu = el.querySelector('input[type=number]'), z = el.querySelector('button'), bar = el.querySelector('.tq i');
      ui[k] = { range: r, num: nu, bar: bar, lbl: el.querySelector('.lbl') };
      r.addEventListener('input', function () { touch(); setTarget(k, r.value, 'range'); mirFollow(k); });
      nu.addEventListener('change', function () { touch(); setTarget(k, nu.value, 'num'); mirFollow(k); });
      z.addEventListener('click', function () { touch(); setTarget(k, 0); mirFollow(k); });
    });
    ge.n = n;
  });
})();

function servoLabel(s, k) {
  var no = '<b class="mono">#' + String(k + 1).padStart(2, '0') + '</b> ';
  return LANG === 'zh' ? no + s.zh + ' <span style="color:var(--mute)">' + s.en + '</span>' : no + s.en.charAt(0).toUpperCase() + s.en.slice(1);
}
function applyPose(p, base) {       // p: {joint: deg}; joints not listed -> 0
  touch();
  A.servos.forEach(function (s, k) { setTarget(k, p[s.joint] != null ? p[s.joint] : 0); });
}
Array.prototype.forEach.call(document.querySelectorAll('[data-pose]'), function (b) { b.addEventListener('click', function () { applyPose(PRESETS[b.getAttribute('data-pose')]); }); });
$('bCopy').addEventListener('click', function () {
  var o = {}; A.servos.forEach(function (s, k) { o[s.joint] = +target[k].toFixed(2); });
  var t = JSON.stringify(o, null, 1);
  if (navigator.clipboard && navigator.clipboard.writeText) navigator.clipboard.writeText(t).then(function () { flash($('bCopy'), t('copied'), 'copy'); }, function () { window.prompt('Ctrl+C', t); });
  else window.prompt('Ctrl+C', t);
});
$('bPaste').addEventListener('click', function () {
  var t = window.prompt(t('pastePrompt')); if (!t) return;
  try { var o = JSON.parse(t); applyPose(o); } catch (e) { alert(t('jsonErr') + e.message); }
});
function flash(b, txt, key) { b.textContent = txt; setTimeout(function () { b.textContent = t(key); }, 1200); }

// sweep test: one servo at a time swings 45% of its free travel each way around its current value
function sweepTick(dt) {
  var period = 3; SW.t += dt;
  if (SW.t >= period) { setTarget(SW.k, SW.base); SW.t = 0; SW.k = (SW.k + 1) % N; SW.base = target[SW.k]; }
  var s = A.servos[SW.k], lo = s.lo / D2R, hi = s.hi / D2R, w = Math.sin(2 * Math.PI * SW.t / period);
  setTarget(SW.k, SW.base + 0.45 * w * (w >= 0 ? hi - SW.base : SW.base - lo));
}
function startSweep() { pausePlay(); SW.on = true; SW.k = 0; SW.t = 0; SW.base = target[0]; $('bSweep').classList.add('on'); refreshSweep(); }
function stopSweep() { if (!SW.on) return; setTarget(SW.k, SW.base); SW.on = false; $('bSweep').classList.remove('on'); refreshSweep(); }
function refreshSweep() { $('bSweep').textContent = t(SW.on ? 'sweepStop' : 'sweep'); }
$('bSweep').addEventListener('click', function () { SW.on ? stopSweep() : startSweep(); });

