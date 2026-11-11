// ------------------------------------------------------------------ three.js scene
var canvas = $('view'), renderer = new THREE.WebGLRenderer({ canvas: canvas, antialias: true });
renderer.shadowMap.enabled = true; renderer.shadowMap.type = THREE.PCFSoftShadowMap; renderer.setPixelRatio(Math.min(2, window.devicePixelRatio || 1));
var scene = new THREE.Scene(), camera = new THREE.PerspectiveCamera(40, 1, 0.01, 20), controls = new THREE.OrbitControls(camera, canvas);
controls.target.set(0, 0.19, 0); controls.enableDamping = true; controls.dampingFactor = 0.12; controls.maxPolarAngle = Math.PI * 0.499; controls.minDistance = 0.2; controls.maxDistance = 3;
scene.add(new THREE.HemisphereLight(0xffffff, 0x666677, 0.85));
var sun = new THREE.DirectionalLight(0xffffff, 0.8); sun.position.set(0.6, 1.2, 0.5); sun.castShadow = true; sun.shadow.mapSize.set(2048, 2048);
Object.assign(sun.shadow.camera, { left: -0.6, right: 0.6, top: 0.6, bottom: -0.6, near: 0.1, far: 4 }); sun.shadow.camera.updateProjectionMatrix(); sun.shadow.bias = -0.0004; scene.add(sun);
var floorMat = new THREE.MeshPhongMaterial({ color: 0xd9dbe3, shininess: 5 }), floor = new THREE.Mesh(new THREE.CircleGeometry(3, 64), floorMat);
floor.rotation.x = -Math.PI / 2; floor.receiveShadow = true; scene.add(floor);
var grid = null;
function theme() {
  var cs = getComputedStyle(document.documentElement), bg = cs.getPropertyValue('--bg').trim(), fl = cs.getPropertyValue('--floor').trim(), gr = cs.getPropertyValue('--grid').trim();
  renderer.setClearColor(new THREE.Color(bg)); floorMat.color.set(fl);
  if (grid) { scene.remove(grid); grid.geometry.dispose(); grid.material.dispose(); }
  grid = new THREE.GridHelper(2, 40, new THREE.Color(gr), new THREE.Color(gr)); grid.position.y = 0.0006; scene.add(grid);   // GridHelper colours are vertex colours -> rebuild on theme change
}
theme(); if (window.matchMedia) window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', theme);
function resize() { var st = $('stage'), w = st.clientWidth, h = st.clientHeight; renderer.setSize(w, h, false); camera.aspect = w / h; camera.updateProjectionMatrix(); }
new ResizeObserver(resize).observe($('stage')); resize();
var VIEWS = { front: [1.15, 0.26, 0], back: [-1.15, 0.26, 0], left: [0, 0.26, -1.15], right: [0, 0.26, 1.15], top: [0.001, 1.5, 0], iso: [0.8, 0.5, 0.8] };   // robot faces +x (three x); its left (+y in MuJoCo) is three -z
function view(n) {   // portrait screens: pull back and push the robot below the HUD
  var v = VIEWS[n], f = camera.aspect < 0.9 ? 1.45 : 1, ty = camera.aspect < 0.9 ? 0.27 : 0.19;
  camera.position.set(v[0] * f, (v[1] - 0.19) * f + ty, v[2] * f); controls.target.set(0, ty, 0); controls.update();
}
Array.prototype.forEach.call(document.querySelectorAll('[data-view]'), function (b) { b.addEventListener('click', function () { view(b.getAttribute('data-view')); }); });
view('iso');

function dec(b64, T) { var bin = atob(b64), u = new Uint8Array(bin.length); for (var i = 0; i < bin.length; i++) u[i] = bin.charCodeAt(i); return new T(u.buffer); }
function buildMeshes() {
  world = new THREE.Group(); world.rotation.x = -Math.PI / 2; scene.add(world);   // MuJoCo z-up -> three y-up
  var mat = new THREE.MeshPhongMaterial({ vertexColors: true, flatShading: true, shininess: 25, specular: 0x222222 });
  A.links.forEach(function (L) {
    var g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.BufferAttribute(dec(L.pos, Float32Array), 3));
    g.setAttribute('color', new THREE.BufferAttribute(dec(L.col, Uint8Array), 3, true));
    g.setIndex(new THREE.BufferAttribute(dec(L.idx, Uint16Array), 1));
    var m = new THREE.Mesh(g, mat); m.castShadow = true; m.frustumCulled = false; world.add(m); bodyMesh[L.name] = m;
  });
  com = new THREE.Group();
  var dot = new THREE.Mesh(new THREE.SphereGeometry(0.006, 16, 12), new THREE.MeshBasicMaterial({ color: 0xff8a00, depthTest: false })); dot.renderOrder = 9;
  var lg = new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0, 0, 0), new THREE.Vector3(0, 0, -1)]);
  var line = new THREE.Line(lg, new THREE.LineBasicMaterial({ color: 0xff8a00, depthTest: false })); line.renderOrder = 9;
  var ring = new THREE.Mesh(new THREE.RingGeometry(0.008, 0.012, 24), new THREE.MeshBasicMaterial({ color: 0xff8a00, depthTest: false, side: THREE.DoubleSide })); ring.renderOrder = 9;
  com.add(dot, line, ring); com.userData = { line: line, ring: ring }; world.add(com);
}

