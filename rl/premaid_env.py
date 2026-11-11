"""Gymnasium environment for the PreMaid AI (MuJoCo, the repository's mujoco/premaidai.xml).

    env = PreMaidEnv(task="stand")        # keep upright and keep the pose while being pushed
    env = PreMaidEnv(task="walk")         # same + track a forward speed command (no trained policy ships for this task)

action  (25)  in [-1, 1]: servo target = pose0 + ACTION_SCALE * action          (position servos, kp 20, as on the robot)
obs     (82)  gyro (3), gravity in the base frame (3), joint angle error (25), joint velocity (25), previous action (25), speed command (1)
step          0.02 s (10 physics steps of 0.002 s); episode = 1000 steps; falls end the episode.
Random pushes (a 0.1 s force on the base, 0..push_max N, every 1..3 s) are part of the task, as in the web simulator's "push test"."""
import os
import numpy as np
import gymnasium as gym
from gymnasium import spaces
import mujoco

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XML = os.path.join(ROOT, 'mujoco', 'premaidai.xml')
ACTION_SCALE = 0.35          # rad
OBS_DIM, ACT_DIM = 82, 25
SUBSTEPS = 10


class PreMaidEnv(gym.Env):
    metadata = {'render_modes': []}

    def __init__(self, task='stand', max_steps=1000, push=True, push_max=14.0, noise=0.03):
        assert task in ('stand', 'walk')
        self.task, self.max_steps, self.push, self.push_max, self.noise = task, max_steps, push, push_max, noise
        self.m = mujoco.MjModel.from_xml_path(XML); self.d = mujoco.MjData(self.m)
        m = self.m; nm = lambda t, i: mujoco.mj_id2name(m, t, i)
        self.joint_names = [nm(mujoco.mjtObj.mjOBJ_JOINT, m.actuator_trnid[a, 0]) for a in range(m.nu)]
        self.qadr = np.array([m.jnt_qposadr[m.actuator_trnid[a, 0]] for a in range(m.nu)]); self.vadr = np.array([m.jnt_dofadr[m.actuator_trnid[a, 0]] for a in range(m.nu)])
        self.key = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_KEY, 'arms_down'); self.pose0 = m.key_ctrl[self.key].copy()
        self.lo, self.hi = m.actuator_ctrlrange[:, 0].copy(), m.actuator_ctrlrange[:, 1].copy()
        self.base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'base_link')
        self.action_space = spaces.Box(-1.0, 1.0, (ACT_DIM,), np.float32); self.observation_space = spaces.Box(-np.inf, np.inf, (OBS_DIM,), np.float32)
        self.last = np.zeros(ACT_DIM); self.cmd = 0.0; self.t = 0

    # ---- helpers
    def _R(self):
        R = np.zeros(9); mujoco.mju_quat2Mat(R, self.d.qpos[3:7]); return R.reshape(3, 3)

    def _obs(self):
        d, R = self.d, self._R(); g = R.T @ np.array([0.0, 0.0, -1.0])
        return np.concatenate([d.qvel[3:6] * 0.25, g, (d.qpos[self.qadr] - self.pose0) / 0.5, d.qvel[self.vadr] * 0.05, self.last, [self.cmd]]).astype(np.float32)

    def tilt(self): return float(np.arccos(np.clip(self._R()[2, 2], -1, 1)))

    def reset(self, seed=None, options=None):
        super().reset(seed=seed); m, d, rng = self.m, self.d, self.np_random
        mujoco.mj_resetDataKeyframe(m, d, self.key); d.qpos[self.qadr] += rng.normal(0, self.noise, ACT_DIM); d.qpos[2] += 0.002
        d.ctrl[:] = self.pose0; d.xfrc_applied[:] = 0; mujoco.mj_forward(m, d)
        self.last = np.zeros(ACT_DIM); self.t = 0; self.cmd = float(rng.uniform(0.0, 0.15)) if self.task == 'walk' else 0.0
        self.next_push = float(rng.uniform(0.5, 2.0)); self.push_end = -1.0; self.push_vec = np.zeros(3)
        return self._obs(), {}

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=np.float64), -1, 1); m, d = self.m, self.d
        d.ctrl[:] = np.clip(self.pose0 + ACTION_SCALE * a, self.lo, self.hi)
        for _ in range(SUBSTEPS):
            now = d.time
            if self.push and now >= self.next_push and self.push_end < 0:
                ang = self.np_random.uniform(0, 2 * np.pi); self.push_vec = np.array([np.cos(ang), np.sin(ang), 0.0]) * self.np_random.uniform(0, self.push_max); self.push_end = now + 0.1
            if self.push_end >= 0 and now >= self.push_end:
                self.push_vec = np.zeros(3); self.push_end = -1.0; self.next_push = now + self.np_random.uniform(1.0, 3.0)
            d.xfrc_applied[self.base, :3] = self.push_vec
            mujoco.mj_step(m, d)
        self.t += 1; tilt = self.tilt(); z = float(d.qpos[2]); fell = z < 0.12 or tilt > 0.8
        err = d.qpos[self.qadr] - self.pose0
        r = 1.0 - 2.0 * tilt ** 2 - 0.5 * float(np.mean(err ** 2)) - 0.01 * float(np.mean(a ** 2)) - 0.05 * float(np.mean((a - self.last) ** 2))
        if self.task == 'walk':
            R = self._R(); vx = float((R.T @ d.qvel[:3])[0]); r += 1.5 * np.exp(-((vx - self.cmd) / 0.05) ** 2) - 1.0 * abs(float((R.T @ d.qvel[:3])[1]))
        if fell: r -= 10.0
        self.last = a
        return self._obs(), float(r), bool(fell), self.t >= self.max_steps, {'tilt': tilt, 'z': z}
