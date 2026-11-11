"""Push tolerance: smallest 0.1 s force (N) on the base, from 8 directions, that makes the robot fall within 4 s.
    python rl/eval_push.py rl/runs/stand/policy.npz        # compares the policy with holding the pose (zero action)"""
import os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from premaid_env import PreMaidEnv
import export_policy as ep


def falls(env, act, F, ang, secs=4.0):
    obs, _ = env.reset(seed=0); env.push = False; env.noise = 0
    for step in range(int(secs / 0.02)):
        if step == 25: env.push_vec = np.array([np.cos(ang), np.sin(ang), 0.0]) * F; env.push_end = env.d.time + 0.1
        a = act(obs); obs, r, term, trunc, info = env.step(a)
        if term: return True
        if step == 30: env.push_vec = np.zeros(3)
    return False


def threshold(env, act, ang, lo=2.0, hi=60.0, step=0.5):
    F = lo
    while F <= hi:
        if falls(env, act, F, ang): return F
        F += step
    return hi


def main(path, dirs=8):
    p = ep.load_npz(path); env = PreMaidEnv(push=False)
    res = {}
    for name, act in (('hold pose', lambda o: np.zeros(25)), ('policy', lambda o: ep.forward(p, o))):
        res[name] = [threshold(env, act, 2 * np.pi * i / dirs) for i in range(dirs)]
        print('%-10s thresholds [N] per direction: %s   mean %.1f  min %.1f' % (name, ' '.join('%4.1f' % x for x in res[name]), np.mean(res[name]), np.min(res[name])), flush=True)
    return res


if __name__ == '__main__':
    main(sys.argv[1] if len(sys.argv) > 1 else 'rl/runs/stand/policy.npz')
