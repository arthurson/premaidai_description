// ------------------------------------------------------------------ MuJoCo
var LOCAL = { js: null, wasm: null };
function rd(f, buf) { return new Promise(function (res, rej) { var r = new FileReader(); r.onload = function () { res(r.result); }; r.onerror = function () { rej(r.error); }; buf ? r.readAsArrayBuffer(f) : r.readAsText(f); }); }
async function loadMujoco() {
  var mod, opts = {};
  if (location.protocol === 'file:') {
    if (!(LOCAL.js && LOCAL.wasm)) { $('loadMsg').textContent = t('waitVendor'); $('loadFile').style.display = 'block'; await new Promise(function (r) { $('fVendor').addEventListener('change', function () { var fs = $('fVendor').files; for (var i = 0; i < fs.length; i++) { if (/mujoco\.js$/.test(fs[i].name)) LOCAL.js = fs[i]; if (/mujoco\.wasm$/.test(fs[i].name)) LOCAL.wasm = fs[i]; } if (LOCAL.js && LOCAL.wasm) r(); }); }); }
    $('loadFile').style.display = 'none'; $('loadMsg').textContent = t('loading2');
    var js = (await rd(LOCAL.js)).split('import.meta.url').join(JSON.stringify('https://mujoco.invalid/mujoco.js'));   // a blob: URL cannot be a URL base
    mod = await import(URL.createObjectURL(new Blob([js], { type: 'text/javascript' })));
    opts.wasmBinary = await rd(LOCAL.wasm, true);
  } else mod = await import(new URL('vendor/mujoco/mujoco.js', location.href).href);
  return mod.default(opts);
}
function build(mj) {
  var m = mj.MjModel.from_xml_string(A.xml), d = new mj.MjData(m), O = mj.mjtObj;
  S.mj = mj; S.m = m; S.d = d; S.ts = m.opt.timestep;
  A.links.forEach(function (L) { var id = mj.mj_name2id(m, O.mjOBJ_BODY.value, L.name); if (id < 0) throw new Error('body ' + L.name + ' missing'); bid[L.name] = id; });
  A.servos.forEach(function (s, k) {
    var j = mj.mj_name2id(m, O.mjOBJ_JOINT.value, s.joint), a = mj.mj_name2id(m, O.mjOBJ_ACTUATOR.value, 'act_' + s.joint);
    if (j < 0 || a < 0) throw new Error('servo ' + (k + 1) + ' ' + s.joint + ' joint/actuator missing');
    act[k] = a; jid[k] = j; qadr[k] = m.jnt_qposadr[j];
  });
  S.base = bid.base_link;
  var mass = 0; for (var i = 0; i < m.nbody; i++) mass += m.body_mass[i];
  S.infoData = { nb: m.nbody - 1, nu: m.nu, mass: mass, dt: S.ts * 1000, ver: mj.mj_versionString ? mj.mj_versionString() : '' }; renderInfo();
  setMu(S.mu); setKp(S.kp);
}
function renderInfo() { if (S.infoData) $('info').textContent = t('infoLine', S.infoData); }
function setMu(v) { S.mu = +v; $('rMu').value = v; $('vMu').textContent = (+v).toFixed(2); if (!S.m) return; var f = S.m.geom_friction; for (var g = 0; g < S.m.ngeom; g++) f[3 * g] = S.mu; }
function setKp(v) { S.kp = +v; $('rKp').value = v; $('vKp').textContent = String(Math.round(v)); applyGains(); }
function applyGains() {   // position servo: gain = kp * stretch factor, bias = [0, -kp, -kv]; a released (free, raw 0) servo gets all three = 0
  if (!S.m) return; var gp = S.m.actuator_gainprm, bp = S.m.actuator_biasprm, kv = A.params.kv;
  for (var k = 0; k < N; k++) { var a = act[k], on = !P.freeNow[k], kp = S.kp * stretchFactor(S.svStretch[k]); gp[10 * a] = on ? kp : 0; bp[10 * a + 1] = on ? -kp : 0; bp[10 * a + 2] = on ? -kv : 0; }
}
function clearServoParams() {   // back to the normal servo settings when the user takes over from a PMA
  var dirty = false;
  for (var k = 0; k < N; k++) {
    if (S.svSpeed[k] || S.svStretch[k] !== 60) { S.svSpeed[k] = 0; S.svStretch[k] = 60; dirty = true; }
  }
  if (dirty) applyGains();
}
function ctrlOf(k) { return clampDeg(k, target[k]) * D2R; }
function setCtrl() {   // 0x19 speed -> maximum angular velocity of the position command (0 = unlimited)
  var ct = S.d.ctrl, ts = S.ts; if (POL.on && S.run && !S.pin) polStep(); if (BAL.on && !S.pin && S.run) balStep(); else if (BAL.o.length) BAL.o = [];
  for (var k = 0; k < N; k++) {
    var want = ctrlOf(k) + (BAL.o[k] || 0), sp = S.svSpeed[k] | 0;
    if (sp >= 1) {
      var maxStep = (SV_SPEED_LO + (Math.min(127, sp) / 127) * (SV_SPEED_HI - SV_SPEED_LO)) * D2R * ts, d = want - S.cmd[k];
      if (d > maxStep) want = S.cmd[k] + maxStep; else if (d < -maxStep) want = S.cmd[k] - maxStep;
    }
    S.cmd[k] = want; ct[act[k]] = want;
  }
}
function lowestZ() {   // lowest collision-hull vertex in the world (m.mesh_vert is in each mesh's own frame)
  var m = S.m, d = S.d, gt = m.geom_type, gi = m.geom_dataid, gp = d.geom_xpos, gm = d.geom_xmat, mv = m.mesh_vert, va = m.mesh_vertadr, vn = m.mesh_vertnum, lo = 1e9;
  for (var g = 0; g < m.ngeom; g++) {
    if (gt[g] !== 7) continue;   // mjGEOM_MESH
    var id = gi[g], a0 = va[id], n = vn[id], pz = gp[3 * g + 2], r0 = gm[9 * g + 6], r1 = gm[9 * g + 7], r2 = gm[9 * g + 8];
    for (var i = 0; i < n; i++) { var o = 3 * (a0 + i), z = pz + r0 * mv[o] + r1 * mv[o + 1] + r2 * mv[o + 2]; if (z < lo) lo = z; }
  }
  return lo;
}
function reset() {   // upright, joints at the commanded pose, lowest point 1 mm above the floor (or hanging at the pin height), at rest
  var mj = S.mj, m = S.m, d = S.d, qp;
  mj.mj_resetData(m, d); qp = d.qpos;
  qp[0] = 0; qp[1] = 0; qp[2] = S.pin ? A.z0 : 1; qp[3] = 1; qp[4] = 0; qp[5] = 0; qp[6] = 0;
  for (var k = 0; k < N; k++) { qp[qadr[k]] = ctrlOf(k); S.cmd[k] = qp[qadr[k]]; }
  setCtrl(); mj.mj_forward(m, d);
  if (!S.pin) { d.qpos[2] = 1 - lowestZ() + 0.001; mj.mj_forward(m, d); }
  d.eq_active[0] = S.pin ? 1 : 0; S.acc = 0; S.simT = 0;
}
function advance(dt) {
  var mj = S.mj, m = S.m, d = S.d, n = 0;
  setCtrl(); S.acc += dt;
  while (S.acc >= S.ts && n < 120) { setCtrl(); mj.mj_step(m, d); S.acc -= S.ts; n++; }
  if (S.acc > S.ts) S.acc = 0;      // device too slow: drop the backlog instead of spiralling
  S.simT += n * S.ts;
}
function updateScene() {
  var d = S.d, xp = d.xpos, xq = d.xquat;
  for (var name in bodyMesh) { var i = bid[name], m = bodyMesh[name]; m.position.set(xp[3 * i], xp[3 * i + 1], xp[3 * i + 2]); m.quaternion.set(xq[4 * i + 1], xq[4 * i + 2], xq[4 * i + 3], xq[4 * i]); }
  var sc = d.subtree_com, b = S.base, cx = sc[3 * b], cy = sc[3 * b + 1], cz = sc[3 * b + 2];
  com.visible = $('cCom').checked; com.position.set(cx, cy, cz);
  com.userData.line.scale.set(1, 1, Math.max(cz, 0.001)); com.userData.ring.position.set(0, 0, -cz + 0.001);
}
function status(now) {
  if (now - S.statT < 100) return; S.statT = now;
  var d = S.d, m = S.m, b = S.base, qx = d.xquat[4 * b + 1], qy = d.xquat[4 * b + 2], z = d.xpos[3 * b + 2], tilt = Math.acos(Math.max(-1, Math.min(1, 1 - 2 * (qx * qx + qy * qy)))) * 180 / Math.PI;
  var af = d.actuator_force, fr = m.actuator_forcerange, sat = 0, k;
  for (k = 0; k < N; k++) { var f = Math.abs(af[act[k]]) / fr[2 * act[k] + 1]; ui[k].bar.style.width = Math.min(100, f * 100).toFixed(0) + '%'; var s = f >= 0.98; ui[k].bar.classList.toggle('sat', s); if (s) sat++; }
  var fallen = !S.pin && (z < 0.15 || tilt > 50), st = $('stat');
  st.textContent = 't ' + S.simT.toFixed(1) + 's · z ' + (z * 100).toFixed(1) + 'cm · ' + t('sTilt') + ' ' + tilt.toFixed(0) + '° · ' + t('sSat') + ' ' + sat + '/' + N + (SW.on ? ' · ' + t('sSweep') + ' #' + String(SW.k + 1).padStart(2, '0') + ' ' + A.servos[SW.k].joint.replace('_joint', '') : '') + (fallen ? ' · ' + t('sFell') : '') + (S.run ? '' : ' · ' + t('sPaused'));
  st.classList.toggle('warn', fallen);
}
function frame(now) {
  requestAnimationFrame(frame);
  var dt = S.last ? Math.min(0.05, (now - S.last) / 1000) : 0; S.last = now;
  if (S.ready) {
    if (SW.on) sweepTick(dt);
    if (P.playing && S.run && P.total > 0) playTick(dt * S.speed);
    gpPoll(); if (WK.on && S.run) walkTick(dt * S.speed);
    if (S.run) pushTick(dt * S.speed);
    if (S.run) advance(dt * S.speed); else setCtrl();
    updateScene(); status(now); rbTick(now); audioSync(); ikTick(); p2Frame(now, dt);
  }
  controls.update(); renderer.render(scene, camera); if (REC.gif) gifSnap(now);
}

