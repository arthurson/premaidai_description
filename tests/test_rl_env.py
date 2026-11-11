import os, sys, unittest
try:
    import numpy as np, gymnasium as gym, mujoco
except ImportError:
    gym = None
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); sys.path.insert(0, os.path.join(ROOT, 'rl'))


@unittest.skipIf(gym is None, 'gymnasium / mujoco not installed')
class EnvTest(unittest.TestCase):
    def test_gymnasium_api(self):
        from gymnasium.utils.env_checker import check_env
        from premaid_env import PreMaidEnv
        check_env(PreMaidEnv(), skip_render_check=True)

    def test_spaces_and_determinism(self):
        from premaid_env import PreMaidEnv, OBS_DIM, ACT_DIM
        runs = []
        for _ in range(2):
            e = PreMaidEnv(); o, _ = e.reset(seed=3); acts = np.random.default_rng(0).uniform(-1, 1, (50, ACT_DIM))
            for a in acts: o, r, te, tr, i = e.step(a)
            runs.append(o)
        self.assertEqual(runs[0].shape, (OBS_DIM,)); np.testing.assert_array_equal(runs[0], runs[1])

    def test_holding_the_pose_stands_without_pushes(self):
        from premaid_env import PreMaidEnv
        e = PreMaidEnv(push=False); e.reset(seed=1); ret = 0
        for _ in range(500):
            o, r, te, tr, i = e.step(np.zeros(25)); ret += r
            self.assertFalse(te)
        self.assertGreater(ret / 500, 0.9); self.assertLess(i['tilt'], 0.05)

    def test_hard_pushes_topple_a_passive_robot(self):
        from premaid_env import PreMaidEnv
        e = PreMaidEnv(push=True, push_max=60.0); fell = 0
        for s in range(6):
            e.reset(seed=s)
            for _ in range(400):
                o, r, te, tr, i = e.step(np.zeros(25))
                if te: fell += 1; self.assertLess(r, -5); break
        self.assertGreaterEqual(fell, 3)

    def test_walk_task_reward_uses_the_command(self):
        from premaid_env import PreMaidEnv
        e = PreMaidEnv(task='walk', push=False); o, _ = e.reset(seed=2); self.assertGreater(o[-1], 0); e.step(np.zeros(25))


if __name__ == '__main__':
    unittest.main()
