import json, os, struct, subprocess, sys, unittest
try:
    import numpy as np, trimesh
except ImportError:
    trimesh = None
try:
    import mujoco
except ImportError:
    mujoco = None
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); GLB = os.path.join(ROOT, 'export', 'premaidai.glb')


def glb_json(path):
    b = open(path, 'rb').read(); assert b[:4] == b'glTF'; n = struct.unpack('<I', b[12:16])[0]; return json.loads(b[20:20 + n])


@unittest.skipIf(trimesh is None, 'trimesh not installed')
class GltfTest(unittest.TestCase):
    def test_structure(self):
        j = glb_json(GLB); self.assertEqual(j['asset']['version'], '2.0')
        names = {n.get('name') for n in j['nodes']}
        for link in ('base_link', 'r_foot', 'l_foot', 'r_hand', 'l_hand', 'r_uleg', 'l_uleg'): self.assertIn(link, names)
        self.assertGreaterEqual(len(j['meshes']), 25); self.assertTrue(all('COLOR_0' in m['primitives'][0]['attributes'] for m in j['meshes']))
        joints = json.load(open(os.path.join(ROOT, 'export', 'premaidai_joints.json')))
        self.assertEqual(sum(1 for r in joints if r['type'] == 'revolute'), 25)
        self.assertEqual(len(j['asset'].get('extras', {}).get('joints', joints)), len(joints))

    @unittest.skipIf(mujoco is None, 'mujoco not installed')
    def test_node_positions_match_mujoco_zero_pose(self):
        scene = trimesh.load(GLB, force='scene'); m = mujoco.MjModel.from_xml_path(os.path.join(ROOT, 'mujoco', 'premaidai.xml')); d = mujoco.MjData(m)
        d.qpos[:3] = 0; d.qpos[3] = 1; mujoco.mj_forward(m, d); checked = 0
        for b in range(1, m.nbody):
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
            if name not in scene.graph.nodes: continue
            T, _ = scene.graph.get(name); x, y, z = d.xpos[b]
            np.testing.assert_allclose(T[:3, 3], [x, z, -y], atol=1e-6, err_msg=name)           # glTF is Y-up: (x, y, z) -> (x, z, -y)
            R = np.zeros(9); mujoco.mju_quat2Mat(R, d.xquat[b]); Rm = R.reshape(3, 3); Y = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]])
            np.testing.assert_allclose(T[:3, :3], Y @ Rm, atol=1e-6, err_msg=name); checked += 1
        self.assertGreaterEqual(checked, 26)

    def test_size_is_a_robot(self):
        scene = trimesh.load(GLB, force='scene'); lo, hi = scene.bounds; ext = hi - lo
        self.assertTrue(0.2 < ext[1] < 0.6, ext)                      # height along Y

    def test_export_is_up_to_date(self):
        try:
            import collada, xacro                                  # noqa: F401
        except ImportError:
            self.skipTest('pycollada / xacro not installed')
        r = subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'export_gltf.py'), '--verify'], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)


if __name__ == '__main__':
    unittest.main()
