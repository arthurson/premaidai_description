#!/usr/bin/env python3
"""premaidai_description -> MuJoCo WASM web simulator (Apache-2.0, same idea as ubtech-alpha2-urdf/mujoco/make_mjcf.py).

Reads the ROS xacro model (robots/premaidai.urdf.xacro), expands it, and writes
  mujoco/premaidai.xml        ONE self-contained MJCF (free base, floor, 25 position servos, collision hulls inlined)
  web/premaid-sim.html        single-file simulator = web/premaid-sim.src.html + the MJCF + the coloured visual meshes

Run (repo root):   pip install mujoco numpy scipy trimesh pycollada xacro
                   python mujoco/make_premaid_mjcf.py            # writes both files
                   python mujoco/make_premaid_mjcf.py --check    # also settles the model for 3 s and prints the result
Needs no ROS: $(find premaidai_description) is resolved to this checkout.

Two fixes versus the raw package (both measured, see README_PHYSICS in the generated html footer):
 * The .dae meshes are in centimetres / Y-up; the scene-node matrix (x0.01, axis swap) MUST be applied. With it the mesh
   centre of mass equals the URDF <inertial><origin> to 0.1 mm for every watertight link.
 * The URDF inertia tensors are exactly 100x too large (e.g. r_uarm ixx 3.57e-3 vs 3.57e-5 kg m^2 from the mesh at
   uniform density). INERTIA_SCALE = 0.01 corrects it.
"""
import os, sys, re, json, math, base64, shutil, tempfile
import numpy as np
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__)); PKG = os.path.dirname(HERE)
XML_OUT = os.path.join(HERE, 'premaidai.xml')
SRC_HTML = os.path.join(PKG, 'web', 'premaid-sim.src.html')
HTML_OUT = os.path.join(PKG, 'web', 'premaid-sim.html')

INERTIA_SCALE = 0.01        # URDF tensors are 100x too large (see docstring)
TIMESTEP = 0.002
KP, KV = 20.0, 0.5          # position servo: torque = KP*(q_target-q) - KV*qdot, clipped to the URDF effort
ARMATURE = 0.005            # kg m^2 reflected rotor inertia: keeps the light links stable under servo stiffness (same as alpha2)
DAMPING = 0.05              # N m s/rad passive joint damping (URDF says 0.4 = Gazebo default; too slow for position control)
MU = 0.8

LABEL = {  # joint suffix -> (zh, en)
    'hpz': ('髖旋轉', 'hip yaw'), 'hpx': ('髖側擺', 'hip roll'), 'hpy': ('髖前後', 'hip pitch'), 'kny': ('膝', 'knee'),
    'aky': ('踝前後', 'ankle pitch'), 'akx': ('踝側擺', 'ankle roll'),
    'shy': ('肩前後', 'shoulder pitch'), 'shx': ('肩側舉', 'shoulder roll'), 'ely': ('上臂旋轉', 'upper-arm twist'),
    'elx': ('手肘', 'elbow'), 'wry': ('手腕旋轉', 'wrist twist'),
    'ny': ('頭前後', 'neck pitch'), 'nz': ('轉頭', 'head yaw'), 'nx': ('頭側傾', 'head roll')}

# ---- .pma replay calibration -------------------------------------------------------------------------------------
# .pma packets carry raw ICS positions (3500..11500 = 270 deg, 7500 = centre).  Mapping used by the web replay:
#     theta_urdf[deg] = sign * (raw - 7500) * 270/8000
# Ranges come from premaiduino2/ServoConfig.h: the firmware clamps the PHYSICAL position  home + (raw - baseline)  to
# [min, max], so the raw range a servo can reach is [min-home+baseline, max-home+baseline].  This matters for the shoulder
# pitch (home 10200 / 4700) and wrist twist (home 5000 / 10000), whose real travel is far from the URDF's +-135.
# sign: verified on the 10 sample .pma (3089 frames): knee/hip-pitch/ankle-pitch signs make hip+knee+ankle ~ 0 (RMS 2.7 deg,
# all other combinations >= 21 deg); elbow/shoulder-roll/head-pitch signs keep the data inside the URDF limits (0 violations);
# hip/ankle roll: same raw sign on both legs and ankle = -hip in pulses (LegIK.h, corr -0.62 in the data) => hip and ankle share
# the URDF sign.  The absolute direction (+1) was settled by a dynamics test: replaying the 10 sample dances open-loop on the floor,
# +1/+1 gives a median survival of >=40 s (5/10 never fall) versus 6.6 s (1/10) for -1/-1, and flipping only hip or only ankle is no better.
# inferred=True: absolute direction could NOT be derived from the data (mirror pairs are correct, the pair as a
# whole may be flipped) -> the web UI has a per-servo flip checkbox.
#            joint      pma_id sign  fw_min fw_max  home  baseline inferred
PMA_CAL = {
    'r_shy_joint': (0x02, +1, 4300, 11500, 10200, 7500, False), 'l_shy_joint': (0x04, -1, 3500, 10700, 4700, 7500, False),
    'r_hpz_joint': (0x06, +1, 6530, 9030, 7780, 7500, True),    'l_hpz_joint': (0x08, +1, 6250, 8750, 7500, 7500, True),
    'r_hpx_joint': (0x0A, +1, 6700, 8300, 7400, 7500, False),    'l_hpx_joint': (0x0C, +1, 6700, 8300, 7600, 7500, False),
    'r_hpy_joint': (0x0E, +1, 4700, 10200, 7500, 7500, False),  'l_hpy_joint': (0x10, -1, 4700, 10200, 7500, 7500, False),
    'r_kny_joint': (0x12, -1, 3950, 7600, 7500, 7500, False),   'l_kny_joint': (0x14, +1, 7400, 11050, 7500, 7500, False),
    'r_aky_joint': (0x16, +1, 5700, 8300, 7500, 7500, False),   'l_aky_joint': (0x18, -1, 6750, 9350, 7550, 7500, False),
    'r_akx_joint': (0x1A, +1, 6800, 9150, 7825, 7500, False),    'l_akx_joint': (0x1C, +1, 6200, 8450, 7450, 7500, False),
    'head_ny_joint': (0x03, +1, 7200, 8400, 7500, 7500, False), 'head_nz_joint': (0x05, +1, 5000, 10000, 7500, 7500, True),
    'head_nx_joint': (0x07, +1, 6900, 8100, 7500, 7500, True),
    'r_shx_joint': (0x09, +1, 7450, 10350, 9500, 9500, False),  'l_shx_joint': (0x0B, +1, 4550, 7550, 5500, 5500, False),
    'r_ely_joint': (0x0D, +1, 4000, 11000, 7500, 7500, True),   'l_ely_joint': (0x0F, -1, 4000, 11000, 7500, 7500, True),
    'r_elx_joint': (0x11, +1, 7100, 11000, 7500, 7500, False),  'l_elx_joint': (0x13, +1, 4000, 7900, 7500, 7500, False),
    'r_wry_joint': (0x15, +1, 3500, 11500, 5000, 7500, True),   'l_wry_joint': (0x17, -1, 3500, 11500, 10000, 7500, True),
}
PULSE_DEG = 270.0 / 8000.0
def pma_range(joint):
    pid, sg, mn, mx, home, base, _ = PMA_CAL[joint]
    a, b = sg * (mn - home + base - 7500) * PULSE_DEG, sg * (mx - home + base - 7500) * PULSE_DEG
    return math.radians(min(a, b)), math.radians(max(a, b))

GROUPS = [('head', '頭 Head'), ('r_arm', '右手 Right arm'), ('l_arm', '左手 Left arm'), ('r_leg', '右腳 Right leg'), ('l_leg', '左腳 Left leg')]

# ------------------------------------------------------------------ xacro -> URDF
def expand_xacro():
    import xacro
    tmp = tempfile.mkdtemp(prefix='premaid_')
    try:
        for d in ('robots', 'urdf'):
            shutil.copytree(os.path.join(PKG, d), os.path.join(tmp, d))
        for root, _, files in os.walk(tmp):
            for f in files:
                p = os.path.join(root, f); s = open(p, encoding='utf-8').read()
                open(p, 'w', encoding='utf-8').write(s.replace('$(find premaidai_description)', tmp))
        return xacro.process_file(os.path.join(tmp, 'robots', 'premaidai.urdf.xacro')).toxml()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

def fl(s, default=0.0): return [float(x) for x in s.split()] if s else [default] * 3

def rpy_quat(r, p, y):   # URDF rpy (fixed axes x,y,z) -> quaternion w,x,y,z
    cr, sr, cp, sp, cy, sy = math.cos(r/2), math.sin(r/2), math.cos(p/2), math.sin(p/2), math.cos(y/2), math.sin(y/2)
    return [cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy]

class Link: pass

def parse_urdf(text):
    root = ET.fromstring(text)
    links = {}
    for l in root.findall('link'):
        o = Link(); o.name = l.get('name'); o.mesh = None; o.inertial = None
        m = l.find('visual/geometry/mesh')
        if m is not None: o.mesh = m.get('filename').split('/')[-1]
        i = l.find('inertial')
        if i is not None:
            a = i.find('inertia')
            o.inertial = dict(mass=float(i.find('mass').get('value')), pos=fl(i.find('origin').get('xyz')),
                              I=[float(a.get(k)) for k in ('ixx', 'iyy', 'izz', 'ixy', 'ixz', 'iyz')])
        links[o.name] = o
    joints = []
    for j in root.findall('joint'):
        o = Link(); o.name = j.get('name'); o.type = j.get('type'); o.parent = j.find('parent').get('link'); o.child = j.find('child').get('link')
        org = j.find('origin'); o.xyz = fl(org.get('xyz')) if org is not None else [0, 0, 0]; o.rpy = fl(org.get('rpy')) if org is not None else [0, 0, 0]
        ax = j.find('axis'); o.axis = fl(ax.get('xyz')) if ax is not None else [1, 0, 0]
        lim = j.find('limit'); o.lo = float(lim.get('lower')) if lim is not None and lim.get('lower') else 0; o.hi = float(lim.get('upper')) if lim is not None and lim.get('upper') else 0
        o.effort = float(lim.get('effort')) if lim is not None else 1; o.vel = float(lim.get('velocity')) if lim is not None else 5
        joints.append(o)
    return links, joints

# ------------------------------------------------------------------ meshes (scene transforms applied!)
def load_dae(path):
    import collada
    c = collada.Collada(path)
    V, F, C, off = [], [], [], 0
    for bg in c.scene.objects('geometry'):
        for bp in bg.primitives():
            if hasattr(bp, 'triangleset'): bp = bp.triangleset()
            idx = np.array(bp.vertex_index).reshape(-1, 3); used, inv = np.unique(idx, return_inverse=True)
            v = np.array(bp.vertex)[used]; col = (0.5, 0.5, 0.5)
            try: col = tuple(bp.material.effect.diffuse[:3])
            except Exception: pass
            V.append(v); F.append(inv.reshape(-1, 3) + off); C.append(np.tile(col, (len(v), 1))); off += len(v)
    return np.vstack(V), np.vstack(F), np.vstack(C)

def b64(a): return base64.b64encode(np.ascontiguousarray(a).tobytes()).decode()

# ------------------------------------------------------------------ build
def build():
    import trimesh
    links, joints = parse_urdf(expand_xacro())
    for j in joints:
        if j.type == 'revolute' and j.name in PMA_CAL: j.lo, j.hi = pma_range(j.name)     # real servo travel (see PMA_CAL)
    kids = {}
    for j in joints: kids.setdefault(j.parent, []).append(j)
    # base_link (massless) + fixed identity child 'body' -> one MuJoCo root body called base_link
    fixed = [j for j in kids['base_link'] if j.type == 'fixed']; assert len(fixed) == 1 and fixed[0].child == 'body' and not any(fixed[0].xyz) and not any(fixed[0].rpy)
    merged = {'base_link': 'body'}
    mesh_dir = os.path.join(PKG, 'meshes')
    visual, hull_xml, servos = [], [], []
    def src_link(n): return merged.get(n, n)

    def link_assets(name):
        L = links[src_link(name)]
        V, F, C = load_dae(os.path.join(mesh_dir, L.mesh))
        tm = trimesh.Trimesh(V, F, process=False)
        H = tm.convex_hull.vertices
        hull_xml.append('    <mesh name="hull_%s" vertex="%s"/>' % (name, ' '.join('%.7g' % x for x in np.asarray(H).ravel())))
        disp = 0.12 + 0.88 * C            # lift the near-black Blender materials so the shading stays readable
        visual.append(dict(name=name, nv=int(len(V)), nf=int(len(F)), pos=b64(V.astype('<f4')), col=b64((np.clip(disp, 0, 1) * 255).round().astype('u1')),
                           idx=b64(F.astype('<u2'))))

    body_lines = []
    def emit(name, ind, parent_joint=None):
        pad = '  ' * ind; L = links[src_link(name)]
        if parent_joint is None:
            body_lines.append('%s<body name="%s" pos="0 0 @ZSTART@">' % (pad, name)); body_lines.append(pad + '  <freejoint name="root"/>')
        else:
            j = parent_joint; q = rpy_quat(*j.rpy); att = ' pos="%.7g %.7g %.7g"' % tuple(j.xyz)
            if any(abs(x) > 1e-12 for x in j.rpy): att += ' quat="%.9g %.9g %.9g %.9g"' % tuple(q)
            body_lines.append('%s<body name="%s"%s>' % (pad, name, att))
            body_lines.append('%s  <joint name="%s" type="hinge" axis="%.6g %.6g %.6g" range="%.7g %.7g" limited="true"/>' % ((pad, j.name) + tuple(j.axis) + (j.lo, j.hi)))
        I = L.inertial; s = INERTIA_SCALE
        body_lines.append('%s  <inertial pos="%.7g %.7g %.7g" mass="%.7g" fullinertia="%.9g %.9g %.9g %.9g %.9g %.9g"/>' % ((pad,) + tuple(I['pos']) + (I['mass'],) + tuple(x * s for x in I['I'])))
        link_assets(name)
        body_lines.append('%s  <geom type="mesh" mesh="hull_%s"/>' % (pad, name))
        for c in kids.get(name, []) + (kids.get('body', []) if name == 'base_link' else []):
            if c.type == 'fixed':
                continue            # base_link->body (merged) and *_hand_ee (collision stub only)
            emit(c.child, ind + 1, c)
        body_lines.append('%s</body>' % pad)
    emit('base_link', 2)

    rev = [j for j in joints if j.type == 'revolute']; assert len(rev) == 25, len(rev)
    def group_of(j):
        c = j.child
        if j.name.startswith('head_'): return 'head'
        side = j.name[0]; return side + ('_arm' if j.name[2:4] in ('sh', 'el', 'wr') else '_leg')
    for j in rev:
        key = j.name.replace('head_', '').replace('_joint', '') if j.name.startswith('head_') else j.name.split('_')[1]
        zh, en = LABEL[key]
        servos.append(dict(joint=j.name, key=key, group=group_of(j), zh=zh, en=en, lo=j.lo, hi=j.hi, effort=j.effort, vel=j.vel, axis=j.axis, link=j.child))
    order = {g: i for i, (g, _) in enumerate(GROUPS)}; servos.sort(key=lambda s: order[s['group']])   # stable: URDF order inside each group
    act = ['    <position name="act_%s" joint="%s" kp="%g" kv="%g" ctrllimited="true" ctrlrange="%.7g %.7g" forcelimited="true" forcerange="%.7g %.7g"/>' %
           (s['joint'], s['joint'], KP, KV, s['lo'], s['hi'], -s['effort'], s['effort']) for s in servos]
    xml = '''<mujoco model="premaidai">
  <compiler angle="radian" balanceinertia="true"/>
  <option timestep="%g" integrator="implicitfast"/>
  <default>
    <joint armature="%g" damping="%g"/>
    <geom contype="1" conaffinity="0" friction="%g 0.005 0.0001" rgba="0.7 0.7 0.75 1"/>
  </default>
  <asset>
%s
  </asset>
  <worldbody>
    <geom name="floor" type="plane" size="3 3 0.1" contype="0" conaffinity="1" friction="%g 0.005 0.0001" rgba="0.8 0.8 0.8 1"/>
%s
  </worldbody>
  <equality>
    <weld name="pin" body1="base_link" active="false"/>
  </equality>
  <actuator>
%s
  </actuator>
</mujoco>
''' % (TIMESTEP, ARMATURE, DAMPING, MU, '\n'.join(hull_xml), MU, '\n'.join(body_lines), '\n'.join(act))
    return xml, visual, servos

def zstart(xml):
    """base height so the lowest collision vertex of the zero pose sits 4 cm above the floor (= where 'Fix base' hangs it)"""
    import mujoco
    m = mujoco.MjModel.from_xml_string(xml.replace('@ZSTART@', '1')); d = mujoco.MjData(m); mujoco.mj_forward(m, d)
    lo = 1e9
    for g in range(m.ngeom):
        if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH: continue
        mid = m.geom_dataid[g]; a = m.mesh_vertadr[mid]; n = m.mesh_vertnum[mid]; v = m.mesh_vert[a:a + n]
        lo = min(lo, float(np.min(d.geom_xpos[g][2] + v @ d.geom_xmat[g].reshape(3, 3)[2])))
    return 1 - lo

def main():
    xml, visual, servos = build()
    z0 = zstart(xml) + 0.04
    xml = xml.replace('@ZSTART@', '%.5f' % z0)
    open(XML_OUT, 'w', encoding='utf-8').write(xml); print('wrote', XML_OUT, len(xml) // 1024, 'KB, base z %.4f' % z0)
    pma = [dict(joint=j, pid=v[0], sign=v[1], rawLo=v[2] - v[4] + v[5], rawHi=v[3] - v[4] + v[5], inferred=v[6]) for j, v in PMA_CAL.items()]
    samples = []
    sdir = os.path.join(PKG, 'web', 'samples')
    for fn in (sorted(os.listdir(sdir)) if os.path.isdir(sdir) else []):
        if fn.lower().endswith('.pma'):
            t = open(os.path.join(sdir, fn), encoding='utf-8-sig', errors='replace').read()
            m = re.search(r'\[\u30e2\u30fc\u30b7\u30e7\u30f3\(\d+\)\]\s*\u756a\u53f7=\d+\s*\u540d\u524d=([^\r\n]*)\s*\u30c7\u30fc\u30bf=([0-9A-Fa-f ]+)', t)
            if m: samples.append(dict(file=fn, name=m.group(1).strip(), b64=base64.b64encode(bytes(int(x, 16) for x in m.group(2).split())).decode()))
    assets = dict(xml=xml, links=visual, servos=servos, pma=pma, samples=samples, z0=z0, params=dict(kp=KP, kv=KV, armature=ARMATURE, damping=DAMPING, mu=MU, dt=TIMESTEP))
    if os.path.exists(SRC_HTML):
        html = open(SRC_HTML, encoding='utf-8').read(); assert '/*__ASSETS__*/' in html
        js = 'window.PREMAID=' + json.dumps(assets, separators=(',', ':'), ensure_ascii=False).replace('</', '<\\/') + ';'
        open(HTML_OUT, 'w', encoding='utf-8').write(html.replace('/*__ASSETS__*/', js)); print('wrote', HTML_OUT, os.path.getsize(HTML_OUT) // 1024, 'KB')
    if '--check' in sys.argv:
        import mujoco
        m = mujoco.MjModel.from_xml_string(xml); d = mujoco.MjData(m)
        print('nq', m.nq, 'nu', m.nu, 'mass %.4f kg' % m.body_mass.sum(), 'ngeom', m.ngeom)

if __name__ == '__main__': main()
