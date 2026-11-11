"""Train a policy with the numpy PPO:   python rl/train.py --task stand --steps 600000 --out rl/runs/stand
Writes <out>/policy.npz (checkpoint after every iteration) and prints one line per iteration. Export with rl/export_policy.py."""
import argparse, os, sys, time
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from premaid_env import PreMaidEnv, OBS_DIM, ACT_DIM
from ppo_numpy import PPO


def train(task='stand', steps=600_000, out='rl/runs/stand', seed=0, push_max=16.0, n_steps=4096, gamma=0.99, lam=0.95, hidden=64, lr=3e-4, quiet=False, resume=False):
    os.makedirs(out, exist_ok=True); env = PreMaidEnv(task=task, push_max=push_max * 0.4); ppo = PPO(OBS_DIM, ACT_DIM, hidden=hidden, lr=lr, seed=seed)
    path = os.path.join(out, 'policy.npz'); start = ppo.load(path) if resume and os.path.exists(path) else 0
    obs, _ = env.reset(seed=seed + start); eps_ret, eps_len, ret, ln, falls, t0 = [], [], 0.0, 0, 0, time.time(); iters = max(1, steps // n_steps)
    for it in range(start, iters):
        env.push_max = push_max * min(1.0, 0.4 + 0.6 * it / max(1, 0.6 * iters))          # push curriculum: 40 % -> 100 % of push_max over 60 % of the run
        X, A, LP, R, D, V, RAW = [], [], [], [], [], [], []
        for _ in range(n_steps):
            xn = ppo.norm(obs); mean, _ = ppo.pi.forward(xn[None]); mean = mean[0]; std = np.exp(ppo.log_std); a = mean + std * ppo.rng.standard_normal(mean.shape)
            v = float(ppo.vf.forward(xn[None])[0][0, 0]); RAW.append(obs); X.append(xn); A.append(a); LP.append(float(ppo.logp(mean, a))); V.append(v)
            obs, r, term, trunc, info = env.step(a); ret += r; ln += 1
            if trunc and not term: r += gamma * float(ppo.vf.forward(ppo.norm(obs)[None])[0][0, 0])          # time limit: bootstrap
            R.append(r); D.append(term or trunc)
            if term or trunc:
                eps_ret.append(ret); eps_len.append(ln); falls += int(term); ret, ln = 0.0, 0; obs, _ = env.reset()
        last_v = float(ppo.vf.forward(ppo.norm(obs)[None])[0][0, 0]); adv = np.zeros(n_steps); g = 0.0
        for t in reversed(range(n_steps)):
            nv = last_v if t == n_steps - 1 else V[t + 1]; nd = 0.0 if D[t] else 1.0
            delta = R[t] + gamma * nv * nd - V[t]; g = delta + gamma * lam * nd * g; adv[t] = g
        info = ppo.update(np.array(X), np.array(A), np.array(LP), adv, adv + np.array(V)); ppo.norm.update(np.array(RAW)); ppo.save(path, it + 1)
        if not quiet and (it % 2 == 0 or it == iters - 1):
            k = eps_ret[-10:]; print('it %3d/%d  steps %7d  push<=%4.1fN  return/ep %8.1f  len %6.1f  falls %3d  std %.2f  %.0fs' % (it + 1, iters, (it + 1) * n_steps, env.push_max, np.mean(k) if k else 0, np.mean(eps_len[-10:]) if eps_len else 0, falls, float(np.exp(ppo.log_std).mean()), time.time() - t0), flush=True)
    return ppo


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--task', default='stand'); ap.add_argument('--steps', type=int, default=600_000); ap.add_argument('--out', default='rl/runs/stand')
    ap.add_argument('--seed', type=int, default=0); ap.add_argument('--push-max', type=float, default=16.0); ap.add_argument('--resume', action='store_true', help='continue from <out>/policy.npz (the machine may be restarted)'); a = ap.parse_args(); train(a.task, a.steps, a.out, a.seed, a.push_max, resume=a.resume)
