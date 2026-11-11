# Reinforcement learning (Gymnasium + PPO + ONNX / browser)

```
rl/premaid_env.py     PreMaidEnv: Gymnasium env on mujoco/premaidai.xml   (task "stand": keep the pose under random pushes; "walk": + speed command, untrained)
rl/ppo_numpy.py       PPO in numpy (no torch): Gaussian MLP, GAE, Adam, running observation normalisation
rl/train.py           python rl/train.py --task stand --steps 1000000 --out rl/runs/stand [--resume]
rl/export_policy.py   python rl/export_policy.py rl/runs/stand/policy.npz --out web/policies --name stand     -> stand.json (browser) + stand.onnx
rl/eval_push.py       push tolerance of a policy versus holding the pose
web/policies/         stand.json / stand.onnx: the demo policy; the simulator loads it from Settings -> "AI policy"
```

* Observation (82): gyro, gravity direction in the base frame, joint angle error and velocity, previous action, speed command. Action (25): servo target = `arms_down` pose + 0.35 rad × action. Control 50 Hz.
* The browser runs the same network (`web/src/js/80-creative-tools.js`, section *AI policy*) with no dependency; the `.onnx` file is the identical graph for onnxruntime(-web) / TensorRT.
  `tests/e2e_p3.py` checks: JS forward pass = Python reference (1e-15), observation vector = the env's, ONNX = JSON network (1e-6), and the same policy in the Python and the wasm MuJoCo stays within a few degrees for 1 s (engines are 4.0.0 vs 3.17).
* `tests/test_rl_env.py` covers the Gymnasium API (`check_env`), determinism, standing without pushes and falling under hard pushes.

## What the shipped demo policy is — and is not

`stand.json` was trained for 1 M steps (~9 min, one CPU core, `--push-max 18`). **It does not beat simply holding the pose**: the smallest 0.1 s push that topples the robot, mean over 8 directions
(`python rl/eval_push.py`): hold pose **9.3 N** (min 7.5), policy **8.8 N** (min 7.5). The robot's tipping limit is set by its small feet and the 0.91 N·m servos, and PPO from scratch with this
budget did not find a recovery step. It is shipped to demonstrate (and test) the pipeline: train → export → run in the browser. For a policy that is actually better use a GPU,
stable-baselines3 / rsl_rl / mujoco_playground with this environment, a larger budget and a stepping reward, then export with `export_policy.py`
(same format; an SB3 `MlpPolicy` with tanh activations can be written into the `W0/b0 …` arrays of a `policy.npz`, `obs_mean/obs_var` from `VecNormalize` — not tested here).
For comparison the hand-written *stand assist* of the simulator (ankle PD + CoM trim, no learning) gives +0.5…1 N.
