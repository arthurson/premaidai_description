"""Export a checkpoint of rl/train.py:   python rl/export_policy.py rl/runs/stand/policy.npz --out web/policies --name stand
  <out>/<name>.json   weights + observation normalisation + the observation layout: loaded by the web simulator (AI policy panel, no dependencies)
  <out>/<name>.onnx   the same network as an ONNX graph (obs [1,82] -> action [1,25], normalisation and clipping included) for onnxruntime / onnxruntime-web / TensorRT"""
import argparse, json, os, sys
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from premaid_env import PreMaidEnv, ACTION_SCALE, OBS_DIM, ACT_DIM

LAYOUT = 'gyro*0.25 (3) | gravity in base frame (3) | (joint angle - pose0)/0.5 (25) | joint velocity*0.05 (25) | previous action (25) | speed command (1)'


def load_npz(path):
    z = np.load(path); n = len([k for k in z.files if k.startswith('W')])
    return {'W': [z['W%d' % i] for i in range(n)], 'b': [z['b%d' % i] for i in range(n)], 'obs_mean': z['obs_mean'], 'obs_var': z['obs_var']}


def forward(p, obs):
    """reference implementation (this is exactly what the browser runs): normalise, clip to +-10, tanh MLP, clip the action to +-1"""
    x = np.clip((np.asarray(obs, dtype=np.float64) - p['obs_mean']) / np.sqrt(p['obs_var'] + 1e-8), -10, 10)
    for i, (W, b) in enumerate(zip(p['W'], p['b'])):
        x = x @ W + b
        if i < len(p['W']) - 1: x = np.tanh(x)
    return np.clip(x, -1, 1)


def to_json(p, task='stand'):
    env = PreMaidEnv(task=task)
    return {'format': 'premaid-policy-v1', 'task': task, 'obs_dim': OBS_DIM, 'act_dim': ACT_DIM, 'activation': 'tanh', 'obs_clip': 10.0, 'obs_layout': LAYOUT,
            'obs_mean': [float(x) for x in p['obs_mean']], 'obs_var': [float(x) for x in p['obs_var']], 'layers': [{'W': w.tolist(), 'b': b.tolist()} for w, b in zip(p['W'], p['b'])],
            'action_scale': ACTION_SCALE, 'pose0': [float(x) for x in env.pose0], 'joint_names': env.joint_names, 'control_dt': 0.02}


def to_onnx(p, path):
    import onnx
    from onnx import TensorProto as T, helper as h, numpy_helper as nh
    f32 = lambda a: np.asarray(a, dtype=np.float32); std = np.sqrt(p['obs_var'] + 1e-8)
    inits = [nh.from_array(f32(p['obs_mean']), 'mean'), nh.from_array(f32(std), 'std'), nh.from_array(np.float32(-10), 'lo'), nh.from_array(np.float32(10), 'hi'), nh.from_array(np.float32(-1), 'alo'), nh.from_array(np.float32(1), 'ahi')]
    nodes = [h.make_node('Sub', ['obs', 'mean'], ['c0']), h.make_node('Div', ['c0', 'std'], ['c1']), h.make_node('Clip', ['c1', 'lo', 'hi'], ['x0'])]; cur = 'x0'
    for i, (W, b) in enumerate(zip(p['W'], p['b'])):
        inits += [nh.from_array(f32(W), 'W%d' % i), nh.from_array(f32(b), 'b%d' % i)]; out = 'y%d' % i
        nodes.append(h.make_node('Gemm', [cur, 'W%d' % i, 'b%d' % i], [out]))
        if i < len(p['W']) - 1: nodes.append(h.make_node('Tanh', [out], ['x%d' % (i + 1)])); cur = 'x%d' % (i + 1)
        else: cur = out
    nodes.append(h.make_node('Clip', [cur, 'alo', 'ahi'], ['action']))
    g = h.make_graph(nodes, 'premaid_policy', [h.make_tensor_value_info('obs', T.FLOAT, [1, OBS_DIM])], [h.make_tensor_value_info('action', T.FLOAT, [1, ACT_DIM])], inits)
    m = h.make_model(g, opset_imports=[h.make_opsetid('', 13)]); m.ir_version = 8; onnx.checker.check_model(m); onnx.save(m, path)


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('npz'); ap.add_argument('--out', default='web/policies'); ap.add_argument('--name', default='stand'); ap.add_argument('--task', default='stand'); a = ap.parse_args()
    p = load_npz(a.npz); os.makedirs(a.out, exist_ok=True)
    json.dump(to_json(p, a.task), open(os.path.join(a.out, a.name + '.json'), 'w'), separators=(',', ':')); to_onnx(p, os.path.join(a.out, a.name + '.onnx'))
    print('wrote', os.path.join(a.out, a.name + '.json|onnx'))
