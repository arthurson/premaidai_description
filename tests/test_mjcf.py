import math, os, subprocess, sys, unittest
try:
    import mujoco, numpy as np
except ImportError:                      # pip install mujoco numpy
    mujoco = None
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); XML = os.path.join(ROOT, 'mujoco', 'premaidai.xml')


@unittest.skipIf(mujoco is None, 'mujoco not installed')
class MjcfTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.m = mujoco.MjModel.from_xml_path(XML)

    def name(self, obj, i): return mujoco.mj_id2name(self.m, obj, i)

    def test_structure(self):
        m = self.m
        self.assertEqual((m.nu, m.nq, m.nkey, m.ncam), (25, 32, 3, 6))
        self.assertEqual(m.nsensor, 56); self.assertEqual(m.nsensordata, 65)
        sites = {self.name(mujoco.mjtObj.mjOBJ_SITE, i) for i in range(m.nsite)}
        self.assertEqual(sites, {'imu', 'r_sole', 'l_sole', 'r_foot_touch', 'l_foot_touch', 'r_hand_tip', 'l_hand_tip'})
        self.assertEqual([self.name(mujoco.mjtObj.mjOBJ_CAMERA, i) for i in range(m.ncam)], ['front', 'back', 'left', 'right', 'top', 'iso'])
        self.assertEqual([self.name(mujoco.mjtObj.mjOBJ_KEY, i) for i in range(m.nkey)], ['t_pose', 'arms_down', 'squat'])

    def settle(self, key, secs=2.0):
        m = self.m; d = mujoco.MjData(m); mujoco.mj_resetDataKeyframe(m, d, key)
        for _ in range(int(secs / m.opt.timestep)):
            d.ctrl[:] = m.key_ctrl[key]; mujoco.mj_step(m, d)
        return d

    def sensor(self, d, name):
        i = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SENSOR, name); a = self.m.sensor_adr[i]; return d.sensordata[a:a + self.m.sensor_dim[i]].copy()

    def test_keyframes_stand_and_sensors_read_correctly(self):
        weight = self.m.body_mass.sum() * 9.81
        for key in range(self.m.nkey):
            d = self.settle(key)
            R = np.zeros(9); mujoco.mju_quat2Mat(R, d.qpos[3:7]); tilt = math.degrees(math.acos(max(-1, min(1, R[8]))))
            self.assertGreater(d.qpos[2], 0.12, key); self.assertLess(tilt, 5, key)
            acc = self.sensor(d, 'imu_acc'); self.assertAlmostEqual(acc[2], 9.81, delta=0.4); self.assertLess(abs(acc[0]) + abs(acc[1]), 0.8)
            self.assertLess(np.abs(self.sensor(d, 'imu_gyro')).max(), 0.05)
            f = self.sensor(d, 'r_foot_touch')[0] + self.sensor(d, 'l_foot_touch')[0]
            self.assertAlmostEqual(f, weight, delta=0.15 * weight, msg='touch sensors must add up to the weight (key %d)' % key)
            q = self.sensor(d, 'imu_quat'); self.assertAlmostEqual(abs(q[0]), 1.0, delta=0.01)
            com = self.sensor(d, 'com'); self.assertTrue(0.05 < com[2] < 0.25)

    def test_joint_and_torque_sensors_match_state(self):
        d = self.settle(1, 1.0); m = self.m; mujoco.mj_forward(m, d)      # sensors are evaluated by the forward pass
        for k in range(m.nu):
            j = m.actuator_trnid[k, 0]; jn = self.name(mujoco.mjtObj.mjOBJ_JOINT, j)
            self.assertAlmostEqual(self.sensor(d, 'pos_' + jn)[0], d.qpos[m.jnt_qposadr[j]], places=9)
            self.assertAlmostEqual(self.sensor(d, 'tau_' + jn)[0], d.actuator_force[k], places=9)


@unittest.skipIf(mujoco is None, 'mujoco not installed')
class GeneratorTest(unittest.TestCase):
    def test_committed_outputs_are_up_to_date(self):
        try:
            import trimesh, collada, scipy, xacro                                   # noqa: F401
        except ImportError:
            self.skipTest('trimesh / pycollada / scipy / xacro not installed')
        r = subprocess.run([sys.executable, os.path.join(ROOT, 'mujoco', 'make_premaid_mjcf.py'), '--verify'], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)


if __name__ == '__main__':
    unittest.main()
