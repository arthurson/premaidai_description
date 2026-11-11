// ---- per-frame hook (called from frame())
function p2Frame(now, dt) {
  teleTick(dt); if (BAL.on) balRender(); senTick(now); pwSync();
  if (REC.once && REC.mr && !P.playing && P.t >= P.total - 1) recStop();
  if (ON.d2 && now - ON.T > 250) { ON.T = now; onRefresh(); }
}

function applyLang() {
  document.documentElement.lang = LANG === 'zh' ? 'zh-Hant' : LANG;
  Array.prototype.forEach.call(document.querySelectorAll('[data-i18n]'), function (el) { var v = t(el.getAttribute('data-i18n')); if (el.hasAttribute('data-i18n-html')) el.innerHTML = v; else el.textContent = v; });
  Array.prototype.forEach.call(document.querySelectorAll('[data-i18n-title]'), function (el) { el.title = t(el.getAttribute('data-i18n-title')); });
  $('stage').setAttribute('data-drop', t('drop')); $('bLang').textContent = LANG === 'zh' ? 'EN' : LANG === 'en' ? '日本語' : '中文'; $('bLang2').textContent = $('bLang').textContent;
  groupEls.forEach(function (g) { g.h.firstChild.textContent = t('g_' + g.key); g.h.querySelector('small').textContent = t('nServos', g.n); });
  A.servos.forEach(function (s, k) { ui[k].lbl.innerHTML = servoLabel(s, k); });
  recUI(); tlRender(); balRender(); abRender(); if (!MC.on) $('mcBtn').textContent = t('mcStart'); refreshRun(); refreshSweep(); refreshList(); renderPma(); renderRb(); renderAudio(); renderBuiltin(); wRender(); rcRender(); gpLabel(); renderInfo(); updatePlayUI(true); S.statT = 0; if (S.ready) status(performance.now());
}
$('bLang').addEventListener('click', function () { LANG = LANG === 'zh' ? 'en' : LANG === 'en' ? 'ja' : 'zh'; try { localStorage.setItem('pmsim.lang', LANG); } catch (e) {} applyLang(); });
applyLang();

loadMujoco().then(function (mj) {
  build(mj); buildMeshes(); reset(); S.ready = true; $('loader').style.display = 'none'; hashBoot();
  window.PMSIM = { qadr: qadr, render: function () { updateScene(); controls.update(); renderer.render(scene, camera); return canvas.toDataURL('image/png'); }, p2: { POL: POL, polLoad: polLoad, polObs: polObs, polForward: polForward, polPose: polPose, BAL: BAL, SEN: SEN, REC: REC, PL: PL, AB: AB, ON: ON, TL: TL, MC: MC, shareHash: shareHash, hashParse: hashParse, hashApply: hashApply, gifEncode: gifEncode, lzwEncode: lzwEncode, mcPoseToTargets: mcPoseToTargets, plPick: plPick, plNext: plNext, teleTick: teleTick, abSet: abSet, rcSelect: rcSelect, onRefresh: onRefresh, senTick: senTick, balStep: balStep, kbHelp: kbHelp, playTick: playTick, WK: WK, gpPoll: gpPoll, RC: RC, fsToggle: fsToggle, recUI: recUI }, S: S, target: target, setTarget: setTarget, reset: reset, advance: advance, applyPose: applyPose, PRESETS: PRESETS, view: view, camera: camera, pma: { P: P, decode: decodePma, timeline: buildTimeline, load: loadSections, poseAt: poseAt, setFlip: setFlip }, rb: { send: rbMotionSend, stop: rbStopMotion, rx: rbRx, state: RB }, p1: { ST: ST, MIR: MIR, TC: TC, BV: BV, LK: LK, MN: MN, estop: estop, decodeServoInfo: decodeServoInfo, mirFollow: mirFollow, teachCapture: teachCapture, rcCsv: rcCsv, RC: RC, poseBytesSome: poseBytesSome } };   // for tests / scripting
}).catch(function (e) { console.error(e); $('loadMsg').textContent = t('failed'); $('loadErr').textContent = String(e && e.message || e) + (location.protocol === 'file:' ? '' : t('httpHint')); });
requestAnimationFrame(frame);
})();
