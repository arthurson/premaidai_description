import math, os, subprocess, sys, unittest
try:
    import mujoco, numpy as np
except ImportError:
    mujoco = None
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); SCENE = os.path.join(ROOT, 'menagerie', 'premaid_ai', 'scene.xml')


@unittest.skipIf(mujoco is None, 'mujoco not installed')
class MenagerieTest(unittest.TestCase):
    def test_same_robot_as_the_main_mjcf(self):
        a = mujoco.MjModel.from_xml_path(os.path.join(ROOT, 'mujoco', 'premaidai.xml')); b = mujoco.MjModel.from_xml_path(SCENE)
        self.assertEqual((a.nq, a.nu, a.nkey, a.nsensor, a.ncam), (b.nq, b.nu, b.nkey, b.nsensor, b.ncam))
        np.testing.assert_allclose(a.body_mass, b.body_mass, rtol=1e-6); np.testing.assert_allclose(a.body_inertia, b.body_inertia, rtol=1e-4)
        np.testing.assert_allclose(a.jnt_range, b.jnt_range); np.testing.assert_allclose(a.actuator_gainprm, b.actuator_gainprm); np.testing.assert_allclose(a.actuator_forcerange, b.actuator_forcerange)
        self.assertGreater(b.ngeom, a.ngeom)                                  # visual meshes added
        self.assertEqual(sum(1 for g in range(b.ngeom) if b.geom_group[g] == 3), a.ngeom - 1)         # one collision hull per link (the main file also has the floor)

    def test_stands_like_the_main_model(self):
        z = []
        for path in (os.path.join(ROOT, 'mujoco', 'premaidai.xml'), SCENE):
            m = mujoco.MjModel.from_xml_path(path); d = mujoco.MjData(m); mujoco.mj_resetDataKeyframe(m, d, 2)
            for _ in range(int(2 / m.opt.timestep)):
                d.ctrl[:] = m.key_ctrl[2]; mujoco.mj_step(m, d)
            z.append(d.qpos[2])
        self.assertGreater(z[1], 0.12); self.assertAlmostEqual(z[0], z[1], delta=0.004)

    def test_assets_exist_and_folder_is_complete(self):
        d = os.path.dirname(SCENE)
        for f in ('README.md', 'LICENSE', 'CHANGELOG.md', 'premaid_ai.xml', 'scene.xml', 'premaid_ai.png', 'test_load.py'):
            self.assertTrue(os.path.exists(os.path.join(d, f)), f)
        self.assertLess(sum(os.path.getsize(os.path.join(r, f)) for r, _, fs in os.walk(d) for f in fs), 6 * 1024 * 1024)

    def test_folder_is_up_to_date(self):
        try:
            import trimesh, collada, scipy, xacro                                   # noqa: F401
        except ImportError:
            self.skipTest('trimesh / pycollada / scipy / xacro not installed')
        r = subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'make_menagerie.py'), '--verify'], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)


if __name__ == '__main__':
    unittest.main()
