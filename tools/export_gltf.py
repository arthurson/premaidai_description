#!/usr/bin/env python3
"""Export the robot as a glTF 2.0 binary (export/premaidai.glb) for Blender / Unity / three.js: one node per URDF link (zero pose, kinematic
hierarchy), coloured meshes, glTF's Y-up axes (root node rotates Z-up -> Y-up, metres). Joint axes / limits / servo ids go to export/premaidai_joints.json
and into the asset extras, so a rig can be rebuilt from the file.
    pip install trimesh pycollada numpy xacro
    python tools/export_gltf.py            # write
    python tools/export_gltf.py --verify   # exit 1 when export/ is stale (CI)"""
import hashlib, importlib.util, json, os, sys
import numpy as np, trimesh
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE); OUT = os.path.join(ROOT, 'export')
spec = importlib.util.spec_from_file_location('gen', os.path.join(ROOT, 'mujoco', 'make_premaid_mjcf.py')); gen = importlib.util.module_from_spec(spec); spec.loader.exec_module(gen)
ZUP_TO_YUP = trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0])          # (x, y, z) -> (x, z, -y)

def joint_matrix(j):
    T = trimesh.transformations.euler_matrix(*j.rpy, axes='sxyz'); T[:3, 3] = j.xyz; return T

def build():
    links, joints = gen.parse_urdf(gen.expand_xacro()); scene = trimesh.Scene(); rows = []
    scene.graph.update(frame_to='premaidai_root', frame_from='world', matrix=ZUP_TO_YUP)
    parent_of = {j.child: j for j in joints}
    def attach(name, parent, T):
        l = links[name]; geom = None
        if l.mesh:
            V, F, C = gen.load_dae(os.path.join(ROOT, 'meshes', l.mesh))
            geom = trimesh.Trimesh(V, F, vertex_colors=np.clip(np.hstack([C, np.ones((len(C), 1))]) * 255, 0, 255).astype(np.uint8), process=False)
        scene.add_geometry(geom, node_name=name, geom_name=name + '_mesh', parent_node_name=parent, transform=T) if geom is not None else scene.graph.update(frame_to=name, frame_from=parent, matrix=T)
    done = {'base_link'}; attach('base_link', 'premaidai_root', np.eye(4)); pending = list(joints)
    while pending:                                               # parents before children, whatever the order of the URDF
        rest = []
        for j in pending:
            if j.parent in done: attach(j.child, j.parent, joint_matrix(j)); done.add(j.child)
            else: rest.append(j)
        assert len(rest) < len(pending), 'URDF is not a tree'; pending = rest
    for i, j in enumerate(joints):
        rows.append({'name': j.name, 'type': j.type, 'parent': j.parent, 'child': j.child, 'origin_xyz': [float(x) for x in j.xyz], 'origin_rpy': [float(x) for x in j.rpy],
                     'axis': [float(x) for x in j.axis], 'lower': float(j.lo) if j.type == 'revolute' else None, 'upper': float(j.hi) if j.type == 'revolute' else None,
                     'effort': float(j.effort) if j.type == 'revolute' else None, 'velocity': float(j.vel) if j.type == 'revolute' else None})
    scene.metadata['joints'] = rows; scene.metadata['up_axis'] = 'Y (root node rotates the robot\'s Z-up frame)'; scene.metadata['source'] = 'https://github.com/arthurson/premaidai_description'
    return scene, rows

def write(out):
    os.makedirs(out, exist_ok=True); scene, rows = build()
    open(os.path.join(out, 'premaidai.glb'), 'wb').write(scene.export(file_type='glb'))
    open(os.path.join(out, 'premaidai_joints.json'), 'w', encoding='utf-8', newline='\n').write(json.dumps(rows, indent=1) + '\n')

def digest(p):
    h = hashlib.sha256()
    for f in ('premaidai.glb', 'premaidai_joints.json'): h.update(open(os.path.join(p, f), 'rb').read())
    return h.hexdigest()

if __name__ == '__main__':
    if '--verify' in sys.argv:
        import tempfile; tmp = tempfile.mkdtemp(); write(tmp); ok = os.path.exists(os.path.join(OUT, 'premaidai.glb')) and digest(tmp) == digest(OUT)
        print('OK: export/premaidai.glb is up to date' if ok else 'STALE: export/ (run python tools/export_gltf.py and commit)'); sys.exit(0 if ok else 1)
    write(OUT); print('wrote', OUT, os.path.getsize(os.path.join(OUT, 'premaidai.glb')) // 1024, 'KB')
