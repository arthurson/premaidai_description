import os, sys, unittest, xml.etree.ElementTree as ET
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, os.path.join(ROOT, 'python'))
import premaidai_description as pm                      # source tree: python/premaidai_description (data two levels up) or an installed wheel
try:
    import mujoco
except ImportError:
    mujoco = None


class PackageTest(unittest.TestCase):
    def test_paths(self):
        for p in (pm.URDF_PATH, pm.XACRO_PATH, pm.MJCF_PATH, pm.MESH_DIR):
            self.assertTrue(os.path.exists(p), p)
        self.assertTrue(os.path.isdir(os.path.join(pm.PACKAGE_PATH, 'premaidai_description')) or os.path.basename(pm.PACKAGE_PATH) == 'premaidai_description')
        self.assertEqual(len(pm.JOINT_NAMES), 25)

    def test_urdf_joints_and_meshes(self):
        root = ET.parse(pm.URDF_PATH).getroot()
        rev = [j.get('name') for j in root.findall('joint') if j.get('type') == 'revolute']
        self.assertEqual(sorted(rev), sorted(pm.JOINT_NAMES))
        files = {m.get('filename') for m in root.iter('mesh')}
        self.assertEqual(len(files), 26)
        for f in files:
            self.assertTrue(f.startswith('package://premaidai_description/meshes/'), f)
            self.assertTrue(os.path.exists(os.path.join(pm.MESH_DIR, os.path.basename(f))), f)
        absu = pm.urdf_path(absolute_meshes=True)
        for m in ET.parse(absu).getroot().iter('mesh'):
            self.assertTrue(os.path.exists(m.get('filename')), m.get('filename'))

    @unittest.skipIf(mujoco is None, 'mujoco not installed')
    def test_mujoco_loader(self):
        m = pm.load_mujoco()
        self.assertEqual((m.nu, m.nkey, m.ncam), (25, 3, 6))
        self.assertEqual([mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a, 0]) for a in range(m.nu)], list(pm.JOINT_NAMES))
        self.assertEqual([mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_KEY, i) for i in range(m.nkey)], list(pm.KEYFRAMES))
        for name in pm.SENSORS:
            self.assertGreaterEqual(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, name), 0, name)
        m, d = pm.load_mujoco('squat'); self.assertGreater(d.qpos[2], 0.1)

    def test_yourdfpy_loader(self):
        try:
            import yourdfpy, trimesh, collada               # noqa: F401
        except ImportError:
            self.skipTest('yourdfpy / trimesh / pycollada not installed')
        r = pm.load_urdf()
        self.assertEqual(set(r.actuated_joint_names), set(pm.JOINT_NAMES))
        self.assertGreater(len(r.scene.geometry), 20)

    def test_pybullet_loader(self):
        try:
            import pybullet as p
        except ImportError:
            self.skipTest('pybullet not installed')
        c = p.connect(p.DIRECT)
        try:
            b = pm.load_pybullet(p)
            self.assertEqual(sum(1 for i in range(p.getNumJoints(b)) if p.getJointInfo(b, i)[2] == p.JOINT_REVOLUTE), 25)
        finally:
            p.disconnect(c)


if __name__ == '__main__':
    unittest.main()
