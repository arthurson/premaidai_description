"""Small self-contained PPO (numpy only, no torch): Gaussian MLP policy (tanh), separate value MLP, GAE, Adam, running observation normalisation.
Good enough for the low-dimensional balance task on one CPU core; use stable-baselines3 / rsl_rl / mujoco_playground for anything big."""
import numpy as np


class MLP:
    def __init__(self, sizes, rng, out_scale=1.0):
        self.W = [rng.normal(0, np.sqrt(1.0 / a), (a, b)) for a, b in zip(sizes[:-1], sizes[1:])]; self.b = [np.zeros(b) for b in sizes[1:]]
        self.W[-1] *= 0.01 * out_scale

    def forward(self, x):
        hs = [x]
        for i, (W, b) in enumerate(zip(self.W, self.b)):
            x = x @ W + b
            if i < len(self.W) - 1: x = np.tanh(x)
            hs.append(x)
        return x, hs

    def backward(self, hs, dout):
        gW, gb = [None] * len(self.W), [None] * len(self.W); d = dout
        for i in range(len(self.W) - 1, -1, -1):
            gW[i] = hs[i].T @ d; gb[i] = d.sum(0)
            if i: d = (d @ self.W[i].T) * (1 - hs[i] ** 2)
        return gW, gb

    def params(self): return self.W + self.b


class Adam:
    def __init__(self, params, lr): self.p, self.lr, self.m, self.v, self.t = params, lr, [np.zeros_like(x) for x in params], [np.zeros_like(x) for x in params], 0

    def step(self, grads, clip=0.5):
        n = np.sqrt(sum(float((g ** 2).sum()) for g in grads)); s = min(1.0, clip / (n + 1e-8)); self.t += 1
        for p, g, m, v in zip(self.p, grads, self.m, self.v):
            g = g * s; m *= 0.9; m += 0.1 * g; v *= 0.999; v += 0.001 * g * g
            p -= self.lr * (m / (1 - 0.9 ** self.t)) / (np.sqrt(v / (1 - 0.999 ** self.t)) + 1e-8)


class RunningNorm:
    def __init__(self, n): self.mean, self.var, self.count = np.zeros(n), np.ones(n), 1e-4

    def update(self, x):
        bm, bv, bc = x.mean(0), x.var(0), len(x); d = bm - self.mean; tot = self.count + bc
        self.mean = self.mean + d * bc / tot; self.var = (self.var * self.count + bv * bc + d ** 2 * self.count * bc / tot) / tot; self.count = tot

    def __call__(self, x): return np.clip((x - self.mean) / np.sqrt(self.var + 1e-8), -10, 10)


class PPO:
    def __init__(self, obs_dim, act_dim, hidden=64, lr=3e-4, seed=0, log_std=-1.0):
        self.rng = np.random.default_rng(seed); self.pi = MLP([obs_dim, hidden, hidden, act_dim], self.rng); self.vf = MLP([obs_dim, hidden, hidden, 1], self.rng, out_scale=100)
        self.log_std = np.full(act_dim, log_std); self.norm = RunningNorm(obs_dim)
        self.opt_pi = Adam(self.pi.params() + [self.log_std], lr); self.opt_vf = Adam(self.vf.params(), lr)

    def act(self, obs, deterministic=False):
        mean, _ = self.pi.forward(self.norm(obs)[None]); mean = mean[0]
        return mean if deterministic else mean + np.exp(self.log_std) * self.rng.standard_normal(mean.shape)

    def logp(self, mean, a): return -0.5 * (((a - mean) / np.exp(self.log_std)) ** 2).sum(-1) - self.log_std.sum() - 0.5 * mean.shape[-1] * np.log(2 * np.pi)

    def update(self, xn, act, logp_old, adv, ret, epochs=8, mb=512, clip=0.2):
        """xn: observations already normalised with the statistics that were used while collecting (call norm.update(raw_obs) afterwards)"""
        n = len(xn); adv = (adv - adv.mean()) / (adv.std() + 1e-8); info = {}
        for _ in range(epochs):
            idx = self.rng.permutation(n)
            for s in range(0, n, mb):
                j = idx[s:s + mb]; xm, am, lo, ad, rt = xn[j], act[j], logp_old[j], adv[j], ret[j]
                mean, hs = self.pi.forward(xm); std = np.exp(self.log_std); ratio = np.exp(self.logp(mean, am) - lo)
                active = ratio * ad <= np.clip(ratio, 1 - clip, 1 + clip) * ad            # the unclipped term is the minimum
                g = -(ratio * ad) * active / len(j)                                        # d loss / d logp_i
                dmean = g[:, None] * (am - mean) / std ** 2; dls = (g[:, None] * (((am - mean) / std) ** 2 - 1)).sum(0)
                gW, gb = self.pi.backward(hs, dmean)
                v, vh = self.vf.forward(xm); gvW, gvb = self.vf.backward(vh, (v[:, 0] - rt)[:, None] / len(j))
                self.opt_pi.step(gW + gb + [dls]); self.opt_vf.step(gvW + gvb); np.clip(self.log_std, -3.0, 0.5, out=self.log_std)
                info = {'ratio': float(ratio.mean()), 'vloss': float(((v[:, 0] - rt) ** 2).mean())}
        return info

    def save(self, path, it=0):
        np.savez(path, **{'W%d' % i: w for i, w in enumerate(self.pi.W)}, **{'b%d' % i: b for i, b in enumerate(self.pi.b)}, **{'vW%d' % i: w for i, w in enumerate(self.vf.W)},
                 **{'vb%d' % i: b for i, b in enumerate(self.vf.b)}, log_std=self.log_std, obs_mean=self.norm.mean, obs_var=self.norm.var, obs_count=self.norm.count, it=it)

    def load(self, path):
        z = np.load(path)
        for i in range(len(self.pi.W)): self.pi.W[i][...] = z['W%d' % i]; self.pi.b[i][...] = z['b%d' % i]
        for i in range(len(self.vf.W)): self.vf.W[i][...] = z['vW%d' % i]; self.vf.b[i][...] = z['vb%d' % i]
        self.log_std[...] = z['log_std']; self.norm.mean, self.norm.var, self.norm.count = z['obs_mean'], z['obs_var'], float(z['obs_count']); return int(z['it'])
