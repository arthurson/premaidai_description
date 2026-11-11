#!/usr/bin/env python3
"""premaidai_description -> MuJoCo WASM web simulator (Apache-2.0, same idea as ubtech-alpha2-urdf/mujoco/make_mjcf.py).

Reads the ROS xacro model (robots/premaidai.urdf.xacro), expands it, and writes
  mujoco/premaidai.xml        ONE self-contained MJCF (free base, floor, 25 position servos, collision hulls inlined)
  web/premaid-sim.html        single-file simulator = web/src/index.template.html + web/src/js/*.js (concatenated, in file-name order) + the MJCF + the coloured visual meshes

Run (repo root):   pip install mujoco numpy scipy trimesh pycollada xacro
                   python mujoco/make_premaid_mjcf.py            # writes both files
                   python mujoco/make_premaid_mjcf.py --check    # also settles the model for 3 s and prints the result
                   python mujoco/make_premaid_mjcf.py --verify   # writes nothing; exit 1 when the committed premaidai.xml / premaid-sim.html are stale (CI)
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
SRC_DIR = os.path.join(PKG, 'web', 'src')
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
# head yaw/roll (0x05/0x07): -1, taken from chikuta/premaidai_controller config/joint_config.yaml (an independent ros_control bridge for the real robot);
# with these two flips all 25 signs agree with that file.  They stay inferred=True: the data alone cannot settle the absolute direction.
# elbow roll (0x11/0x13) and r_wry (0x15) limits widened to what the 10 official motions actually use (elbow up to 11122 / down to 3897 raw, r_wry down to 5916):
# the nominal limits were 3.5-4 deg too tight, so the simulator clamped those motions (found by the motion check in web/premaid-sim.html).
#            joint      pma_id sign  fw_min fw_max  home  baseline inferred
PMA_CAL = {
    'r_shy_joint': (0x02, +1, 4300, 11500, 10200, 7500, False), 'l_shy_joint': (0x04, -1, 3500, 10700, 4700, 7500, False),
    'r_hpz_joint': (0x06, +1, 6530, 9030, 7780, 7500, True),    'l_hpz_joint': (0x08, +1, 6250, 8750, 7500, 7500, True),
    'r_hpx_joint': (0x0A, +1, 6700, 8300, 7400, 7500, False),    'l_hpx_joint': (0x0C, +1, 6700, 8300, 7600, 7500, False),
    'r_hpy_joint': (0x0E, +1, 4700, 10200, 7500, 7500, False),  'l_hpy_joint': (0x10, -1, 4700, 10200, 7500, 7500, False),
    'r_kny_joint': (0x12, -1, 3950, 7600, 7500, 7500, False),   'l_kny_joint': (0x14, +1, 7400, 11050, 7500, 7500, False),
    'r_aky_joint': (0x16, +1, 5700, 8300, 7500, 7500, False),   'l_aky_joint': (0x18, -1, 6750, 9350, 7550, 7500, False),
    'r_akx_joint': (0x1A, +1, 6800, 9150, 7825, 7500, False),    'l_akx_joint': (0x1C, +1, 6200, 8450, 7450, 7500, False),
    'head_ny_joint': (0x03, +1, 7200, 8400, 7500, 7500, False), 'head_nz_joint': (0x05, -1, 5000, 10000, 7500, 7500, True),
    'head_nx_joint': (0x07, -1, 6900, 8100, 7500, 7500, True),
    'r_shx_joint': (0x09, +1, 7450, 10350, 9500, 9500, False),  'l_shx_joint': (0x0B, +1, 4550, 7550, 5500, 5500, False),
    'r_ely_joint': (0x0D, +1, 4000, 11000, 7500, 7500, True),   'l_ely_joint': (0x0F, -1, 4000, 11000, 7500, 7500, True),
    'r_elx_joint': (0x11, +1, 7100, 11130, 7500, 7500, False),  'l_elx_joint': (0x13, +1, 3870, 7900, 7500, 7500, False),
    'r_wry_joint': (0x15, +1, 3400, 11500, 5000, 7500, True),   'l_wry_joint': (0x17, -1, 3500, 11500, 10000, 7500, True),
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
    visual, hull_xml, servos, jorder, HULL = [], [], [], [], {}
    def src_link(n): return merged.get(n, n)

    def link_assets(name):
        L = links[src_link(name)]
        V, F, C = load_dae(os.path.join(mesh_dir, L.mesh))
        tm = trimesh.Trimesh(V, F, process=False)
        H = np.unique(np.round(np.asarray(tm.convex_hull.vertices, dtype=float), 6), axis=0)      # sorted + rounded to 1 um: identical output on every scipy/qhull version
        HULL[name] = H
        hull_xml.append('    <mesh name="hull_%s" vertex="%s"/>' % (name, ' '.join('%.7g' % x for x in np.asarray(H).ravel())))
        disp = 0.12 + 0.88 * C            # lift the near-black Blender materials so the shading stays readable
        visual.append(dict(name=name, nv=int(len(V)), nf=int(len(F)), pos=b64(V.astype('<f4')), col=b64((np.clip(disp, 0, 1) * 255).round().astype('u1')),
                           idx=b64(F.astype('<u2'))))

    body_lines = []
    def link_sites(name, I, H):      # sites: IMU at the base inertial centre; sole centre + touch box on each foot; hand centre
        out = []
        if name == 'base_link': out.append('<site name="imu" pos="%.7g %.7g %.7g" size="0.004" rgba="1 0.2 0.2 1" group="3"/>' % tuple(I['pos']))
        if name in ('r_foot', 'l_foot'):
            lo = H[:, 2].min(); sole = H[H[:, 2] < lo + 0.003]; cx, cy = sole[:, 0].mean(), sole[:, 1].mean(); hx, hy = (H[:, 0].max() - H[:, 0].min()) / 2, (H[:, 1].max() - H[:, 1].min()) / 2
            out.append('<site name="%s_sole" pos="%.6g %.6g %.6g" size="0.004" rgba="0.2 0.6 1 1" group="3"/>' % (name[0], cx, cy, lo))
            out.append('<site name="%s_foot_touch" type="box" pos="%.6g %.6g %.6g" size="%.6g %.6g 0.008" rgba="0.2 0.8 0.3 0.3" group="3"/>' % (name[0], (H[:, 0].max() + H[:, 0].min()) / 2, (H[:, 1].max() + H[:, 1].min()) / 2, lo + 0.002, hx, hy))
        if name in ('r_hand', 'l_hand'):
            c = H.mean(axis=0); out.append('<site name="%s_hand_tip" pos="%.6g %.6g %.6g" size="0.004" rgba="1 0.7 0.1 1" group="3"/>' % (name[0], c[0], c[1], c[2]))
        return out
    def emit(name, ind, parent_joint=None):
        pad = '  ' * ind; L = links[src_link(name)]
        if parent_joint is None:
            body_lines.append('%s<body name="%s" pos="0 0 @ZSTART@">' % (pad, name)); body_lines.append(pad + '  <freejoint name="root"/>')
        else:
            j = parent_joint; q = rpy_quat(*j.rpy); att = ' pos="%.7g %.7g %.7g"' % tuple(j.xyz)
            if any(abs(x) > 1e-12 for x in j.rpy): att += ' quat="%.9g %.9g %.9g %.9g"' % tuple(q)
            body_lines.append('%s<body name="%s"%s>' % (pad, name, att))
            body_lines.append('%s  <joint name="%s" type="hinge" axis="%.6g %.6g %.6g" range="%.7g %.7g" limited="true"/>' % ((pad, j.name) + tuple(j.axis) + (j.lo, j.hi))); jorder.append(j.name)
        I = L.inertial; s = INERTIA_SCALE
        body_lines.append('%s  <inertial pos="%.7g %.7g %.7g" mass="%.7g" fullinertia="%.9g %.9g %.9g %.9g %.9g %.9g"/>' % ((pad,) + tuple(I['pos']) + (I['mass'],) + tuple(x * s for x in I['I'])))
        link_assets(name)
        body_lines.append('%s  <geom type="mesh" mesh="hull_%s"/>' % (pad, name))
        body_lines.extend(pad + '  ' + x for x in link_sites(name, I, HULL[name]))
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
    @CAMERAS@
  </worldbody>
  <equality>
    <weld name="pin" body1="base_link" active="false"/>
  </equality>
  <actuator>
%s
  </actuator>
  <sensor>
%s
  </sensor>
@KEYS@</mujoco>
''' % (TIMESTEP, ARMATURE, DAMPING, MU, '\n'.join(hull_xml), MU, '\n'.join(body_lines), '\n'.join(act), '\n'.join(sensors(servos)))
    xml = xml.replace('@CAMERAS@', '\n    '.join(cameras()))
    return xml, visual, servos, jorder

def source_html():
    """template + the JS parts (web/src/js/*.js, concatenated in file-name order: they are slices of ONE script, so no part is valid on its own)"""
    import glob
    tpl = open(os.path.join(SRC_DIR, 'index.template.html'), encoding='utf-8', newline='').read(); assert tpl.count('<!--@MAIN_SCRIPT-->') == 1
    js = ''.join(open(f, encoding='utf-8', newline='').read() for f in sorted(glob.glob(os.path.join(SRC_DIR, 'js', '*.js'))))
    return tpl.replace('<!--@MAIN_SCRIPT-->', js)

def sensors(servos):
    """IMU (gyro, accelerometer, orientation), foot touch (normal force on the sole), per-joint angle and actuator torque, centre of mass"""
    out = ['    <gyro name="imu_gyro" site="imu"/>', '    <accelerometer name="imu_acc" site="imu"/>', '    <framequat name="imu_quat" objtype="site" objname="imu"/>',
           '    <touch name="r_foot_touch" site="r_foot_touch"/>', '    <touch name="l_foot_touch" site="l_foot_touch"/>', '    <subtreecom name="com" body="base_link"/>']
    out += ['    <jointpos name="pos_%s" joint="%s"/>' % (s['joint'], s['joint']) for s in servos]
    out += ['    <actuatorfrc name="tau_%s" actuator="act_%s"/>' % (s['joint'], s['joint']) for s in servos]
    return out

def cameras():
    """fixed viewpoints that look at the base_link (robot faces +x)"""
    spec = [('front', (0.9, 0, 0.3)), ('back', (-0.9, 0, 0.3)), ('left', (0, 0.9, 0.3)), ('right', (0, -0.9, 0.3)), ('top', (0.001, 0, 1.1)), ('iso', (0.7, -0.7, 0.55))]
    return ['<camera name="%s" pos="%g %g %g" mode="targetbody" target="base_link" fovy="45"/>' % ((n,) + p) for n, p in spec]

POSES = {      # keyframes = the presets of the web simulator (joint angles in degrees)
    't_pose': {},
    'arms_down': {'r_shx_joint': 80, 'l_shx_joint': -80},
    'squat': {'r_hpy_joint': -20, 'l_hpy_joint': -20, 'r_kny_joint': 40, 'l_kny_joint': 40, 'r_aky_joint': -20, 'l_aky_joint': -20, 'r_shx_joint': 80, 'l_shx_joint': -80},
}

def keyframes(xml, servos, jorder):
    """<keyframe> block: base standing 1 mm above the floor for each pose (lowest collision vertex), joints and position-servo targets set"""
    import mujoco
    m = mujoco.MjModel.from_xml_string(xml.replace('@ZSTART@', '1').replace('@KEYS@', '')); d = mujoco.MjData(m)
    out = ['  <keyframe>']
    for name, pose in POSES.items():
        mujoco.mj_resetData(m, d); d.qpos[:3] = (0, 0, 1)
        for j, deg in pose.items(): d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)]] = math.radians(deg)
        mujoco.mj_forward(m, d); lo = 1e9
        for g in range(m.ngeom):
            if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH: continue
            mid = m.geom_dataid[g]; v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            lo = min(lo, float(np.min(d.geom_xpos[g][2] + v @ d.geom_xmat[g].reshape(3, 3)[2])))
        q = [0, 0, 1 - lo + 0.001, 1, 0, 0, 0] + [math.radians(pose.get(j, 0.0)) for j in jorder]
        c = [math.radians(pose.get(s['joint'], 0.0)) for s in servos]
        out.append('    <key name="%s" qpos="%s" ctrl="%s"/>' % (name, ' '.join('%.7g' % x for x in q), ' '.join('%.7g' % x for x in c)))
    out.append('  </keyframe>\n')
    return '\n'.join(out)

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

def kin_chains(xml, servos):
    """forward-kinematics data of the 4 limb chains (relative to base_link) for the in-browser IK: per joint the body offset/rotation and joint axis,
    plus a tip point in the last body (hand centre / sole centre).  Everything else (angles, limits) the page already has."""
    import mujoco
    m = mujoco.MjModel.from_xml_string(xml); idx = {s['joint']: k for k, s in enumerate(servos)}
    bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n); jn = lambda j: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
    spec = {'r_leg': ('r_hip_joint', 'r_foot', True), 'l_leg': ('l_hip_joint', 'l_foot', True), 'r_arm': ('r_uarm_joint', 'r_hand', False), 'l_arm': ('l_uarm_joint', 'l_hand', False)}
    r6 = lambda a: [round(float(x), 6) for x in a]; out = {}
    for name, (first, last, ori) in spec.items():
        chain, b = [], bid(last)
        while b != bid('base_link'): chain.append(b); b = int(m.body_parentid[b])
        chain.reverse(); assert chain[0] == bid(first), name
        js = []
        for b in chain:
            j = int(m.body_jntadr[b]); assert m.body_jntnum[b] == 1 and not np.any(m.jnt_pos[j])
            js.append(dict(k=idx[jn(j)], pos=r6(m.body_pos[b]), quat=r6(m.body_quat[b]), axis=r6(m.jnt_axis[j])))
        pts = []
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != chain[-1] or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH: continue
            mid = m.geom_dataid[g]; v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
            R = np.zeros(9); mujoco.mju_quat2Mat(R, m.geom_quat[g]); pts.append(v @ R.reshape(3, 3).T + m.geom_pos[g])
        pts = np.vstack(pts)
        if ori:   # foot: centre of the sole
            lo = pts[:, 2].min(); sole = pts[pts[:, 2] < lo + 0.003]; tip = [sole[:, 0].mean(), sole[:, 1].mean(), lo]
        else:     # hand: centre of the mesh
            tip = pts.mean(axis=0)
        out[name] = dict(joints=js, tip=r6(tip), ori=ori)
    return out

def mirror_table(xml, servos):
    """left/right mirror of a pose: servo k takes the joint angle of servo p times m.  m follows from the joint axes at the symmetric zero pose
    (a reflection reverses the sense of rotation: theta_left = -theta_right if axis_left == M axis_right, +theta_right if axis_left == -M axis_right)."""
    import mujoco
    m = mujoco.MjModel.from_xml_string(xml); d = mujoco.MjData(m); mujoco.mj_resetData(m, d); mujoco.mj_kinematics(m, d)
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n); M = np.diag([1.0, -1.0, 1.0]); idx = {s['joint']: k for k, s in enumerate(servos)}
    Rb = d.xmat[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'base_link')].reshape(3, 3); out = []
    for s in servos:
        n = s['joint']; pn = 'l_' + n[2:] if n.startswith('r_') else 'r_' + n[2:] if n.startswith('l_') else n
        a, ap = Rb.T @ d.xaxis[jid(n)], Rb.T @ d.xaxis[jid(pn)]; dp = float(ap @ (M @ a))
        assert abs(dp) > 0.9, (n, pn, dp)
        out.append(dict(k=idx[n], p=idx[pn], m=-1 if dp > 0 else 1))
    return out

def main():
    xml, visual, servos, jorder = build()
    xml = xml.replace('@KEYS@', keyframes(xml, servos, jorder))
    z0 = zstart(xml) + 0.04
    xml = xml.replace('@ZSTART@', '%.5f' % z0)
    if '--verify' not in sys.argv: open(XML_OUT, 'w', encoding='utf-8', newline='\n').write(xml); print('wrote', XML_OUT, len(xml) // 1024, 'KB, base z %.4f' % z0)
    pma = [dict(joint=j, pid=v[0], sign=v[1], rawLo=v[2] - v[4] + v[5], rawHi=v[3] - v[4] + v[5], inferred=v[6]) for j, v in PMA_CAL.items()]
    assets = dict(xml=xml, links=visual, servos=servos, pma=pma, chains=kin_chains(xml, servos), mirror=mirror_table(xml, servos), z0=z0, params=dict(kp=KP, kv=KV, armature=ARMATURE, damping=DAMPING, mu=MU, dt=TIMESTEP))
    if os.path.isdir(SRC_DIR):
        html = source_html(); assert '/*__ASSETS__*/' in html
        js = 'window.PREMAID=' + json.dumps(assets, separators=(',', ':'), ensure_ascii=False).replace('</', '<\\/') + ';'
        out = html.replace('/*__ASSETS__*/', js)
        if '--verify' in sys.argv:
            stale = [n for n, p, new in (('mujoco/premaidai.xml', XML_OUT, xml), ('web/premaid-sim.html', HTML_OUT, out)) if open(p, encoding='utf-8', newline='').read() != new]
            print('STALE: ' + ', '.join(stale) + '  (run python mujoco/make_premaid_mjcf.py and commit)' if stale else 'OK: premaidai.xml and premaid-sim.html are up to date'); sys.exit(1 if stale else 0)
        open(HTML_OUT, 'w', encoding='utf-8', newline='\n').write(out); print('wrote', HTML_OUT, os.path.getsize(HTML_OUT) // 1024, 'KB')
    if '--check' in sys.argv:
        import mujoco
        m = mujoco.MjModel.from_xml_string(xml); d = mujoco.MjData(m)
        print('nq', m.nq, 'nu', m.nu, 'mass %.4f kg' % m.body_mass.sum(), 'ngeom', m.ngeom)

if __name__ == '__main__': main()
