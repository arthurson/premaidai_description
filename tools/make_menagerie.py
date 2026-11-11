#!/usr/bin/env python3
"""Package the robot the way MuJoCo Menagerie (github.com/google-deepmind/mujoco_menagerie) expects it:
    menagerie/premaid_ai/{premaid_ai.xml, scene.xml, assets/*.obj|*.stl, README.md, LICENSE, CHANGELOG.md, premaid_ai.png}
Derived from mujoco/premaidai.xml (same masses, inertias, joints, actuators, sensors, keyframes); visual meshes are the coloured DAE links split by material
into OBJ files, collision meshes are the convex hulls as STL.
    pip install mujoco numpy trimesh pycollada xacro
    python tools/make_menagerie.py            # (re)write menagerie/premaid_ai
    python tools/make_menagerie.py --verify   # exit 1 when the committed folder is stale
    MUJOCO_GL=egl python tools/make_menagerie.py --png   # also render premaid_ai.png"""
import filecmp, hashlib, importlib.util, os, shutil, sys, tempfile
import numpy as np, xml.etree.ElementTree as ET
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE); OUT = os.path.join(ROOT, 'menagerie', 'premaid_ai')
spec = importlib.util.spec_from_file_location('gen', os.path.join(ROOT, 'mujoco', 'make_premaid_mjcf.py')); gen = importlib.util.module_from_spec(spec); spec.loader.exec_module(gen)

SCENE = '''<mujoco model="premaid_ai scene">
  <include file="premaid_ai.xml"/>

  <statistic center="0 0 0.15" extent="0.55"/>

  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="140" elevation="-20"/>
  </visual>

  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
  </asset>

  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane" contype="0" conaffinity="1" friction="0.8 0.005 0.0001"/>
  </worldbody>
</mujoco>
'''

def dae_parts(path):
    import collada
    c = collada.Collada(path); parts = {}
    for bg in c.scene.objects('geometry'):
        for bp in bg.primitives():
            if hasattr(bp, 'triangleset'): bp = bp.triangleset()
            idx = np.array(bp.vertex_index).reshape(-1, 3); used, inv = np.unique(idx, return_inverse=True); col = (0.5, 0.5, 0.5)
            try: col = tuple(float(x) for x in bp.material.effect.diffuse[:3])
            except Exception: pass
            parts.setdefault(tuple(round(x, 3) for x in col), []).append((np.array(bp.vertex)[used], inv.reshape(-1, 3)))
    out = {}
    for col, lst in parts.items():
        V, F, off = [], [], 0
        for v, f in lst: V.append(v); F.append(f + off); off += len(v)
        out[col] = (np.vstack(V), np.vstack(F))
    return out

def build(out):
    import trimesh
    shutil.rmtree(out, ignore_errors=True); os.makedirs(os.path.join(out, 'assets'))
    links, joints = gen.parse_urdf(gen.expand_xacro()); merged = {'base_link': 'body'}
    tree = ET.parse(os.path.join(ROOT, 'mujoco', 'premaidai.xml')); root = tree.getroot(); root.set('model', 'premaid_ai')
    root.find('compiler').set('meshdir', 'assets'); root.find('compiler').set('autolimits', 'true')
    d = root.find('default'); g = d.find('geom'); g.attrib.pop('rgba', None)
    for cls, attrs in (('visual', dict(contype='0', conaffinity='0', group='2')), ('collision', dict(group='3'))):
        e = ET.SubElement(d, 'default', {'class': cls}); ET.SubElement(e, 'geom', attrs)
    asset = root.find('asset')
    for m in list(asset): asset.remove(m)
    # hull vertices of the old inline meshes -> STL
    old = ET.parse(os.path.join(ROOT, 'mujoco', 'premaidai.xml')).getroot().find('asset')
    for m in old:
        name = m.get('name')[len('hull_'):]; V = np.array([float(x) for x in m.get('vertex').split()]).reshape(-1, 3)
        trimesh.convex.convex_hull(V).export(os.path.join(out, 'assets', name + '_hull.stl'))
        ET.SubElement(asset, 'mesh', {'name': name + '_hull', 'file': name + '_hull.stl'})
    wb = root.find('worldbody')
    for e in list(wb):
        if e.tag == 'geom' and e.get('name') == 'floor': wb.remove(e)
    eq = root.find('equality')
    if eq is not None: root.remove(eq)
    def walk(b):
        name = b.get('name'); src = links[merged.get(name, name)].mesh; parts = dae_parts(os.path.join(ROOT, 'meshes', src)); new = []
        for e in list(b):
            if e.tag == 'geom' and e.get('mesh') == 'hull_' + name:
                i = list(b).index(e); b.remove(e); vis = []
                for k, (col, (V, F)) in enumerate(sorted(parts.items())):
                    mn = '%s_%d' % (name, k); trimesh.Trimesh(V, F, process=False).export(os.path.join(out, 'assets', mn + '.obj'))
                    ET.SubElement(asset, 'mesh', {'name': mn, 'file': mn + '.obj'})
                    vis.append(ET.Element('geom', {'class': 'visual', 'mesh': mn, 'rgba': '%g %g %g 1' % col}))
                col_geom = ET.Element('geom', {'class': 'collision', 'type': 'mesh', 'mesh': name + '_hull'})
                for j, x in enumerate(vis + [col_geom]): b.insert(i + j, x)
        for c in b.findall('body'): walk(c)
    for b in wb.findall('body'): walk(b)
    ET.indent(tree, space='  '); tree.write(os.path.join(out, 'premaid_ai.xml'), encoding='unicode', xml_declaration=False, short_empty_elements=True)
    with open(os.path.join(out, 'premaid_ai.xml'), 'a', encoding='utf-8') as f: f.write('\n')
    open(os.path.join(out, 'scene.xml'), 'w', encoding='utf-8', newline='\n').write(SCENE)
    for f in ('README.md', 'CHANGELOG.md', 'LICENSE', 'test_load.py'):
        shutil.copy(os.path.join(HERE, 'menagerie_files', f), os.path.join(out, f))
    shutil.copy(os.path.join(ROOT, 'meshes', 'LICENSE.txt'), os.path.join(out, 'assets', 'LICENSE.txt'))

def tree_hash(p):
    h = hashlib.sha256()
    for r, _, fs in sorted(os.walk(p)):
        for f in sorted(fs):
            if f.endswith('.png'): continue
            h.update(os.path.relpath(os.path.join(r, f), p).encode()); h.update(open(os.path.join(r, f), 'rb').read().replace(b'\r\n', b'\n'))
    return h.hexdigest()

def render_png(out):
    import mujoco
    m = mujoco.MjModel.from_xml_path(os.path.join(out, 'scene.xml')); d = mujoco.MjData(m); mujoco.mj_resetDataKeyframe(m, d, 1); mujoco.mj_forward(m, d)
    r = mujoco.Renderer(m, 480, 640); cam = mujoco.MjvCamera(); cam.lookat[:] = (0, 0, 0.16); cam.distance = 0.9; cam.azimuth = 135; cam.elevation = -12
    r.update_scene(d, cam); from PIL import Image; Image.fromarray(r.render()).save(os.path.join(out, 'premaid_ai.png'))

if __name__ == '__main__':
    if '--verify' in sys.argv:
        tmp = tempfile.mkdtemp(prefix='menagerie_'); build(tmp); a, b = tree_hash(tmp), tree_hash(OUT) if os.path.isdir(OUT) else ''
        print('OK: menagerie/premaid_ai is up to date' if a == b else 'STALE: menagerie/premaid_ai (run python tools/make_menagerie.py and commit)'); sys.exit(0 if a == b else 1)
    png = os.path.join(OUT, 'premaid_ai.png'); keep = open(png, 'rb').read() if os.path.exists(png) else None
    build(OUT)
    if keep: open(png, 'wb').write(keep)
    if '--png' in sys.argv: render_png(OUT)
    n = sum(len(f) for _, _, f in os.walk(os.path.join(OUT, 'assets'))); size = sum(os.path.getsize(os.path.join(r, f)) for r, _, fs in os.walk(OUT) for f in fs)
    print('wrote', OUT, n, 'asset files', size // 1024, 'KB')
