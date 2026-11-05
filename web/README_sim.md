# PreMaid AI – MuJoCo WASM simulator (25 servos)

`web/premaid-sim.html` is a single page: three.js view + MuJoCo 3.17 WASM physics + 25 position servos
(3 head, 5+5 arm, 6+6 leg = every revolute joint of the URDF).

Run (online):   `python -m http.server` in the repo root, open `http://localhost:8000/web/premaid-sim.html`
Run (offline):  open `web/premaid-sim.html` from disk and pick `web/vendor/mujoco/mujoco.js` + `mujoco.wasm` when asked.

Regenerate after editing the xacro / `web/premaid-sim.src.html`:

    pip install mujoco numpy scipy trimesh pycollada xacro
    python mujoco/make_premaid_mjcf.py --check

Outputs: `mujoco/premaidai.xml` (self-contained MJCF) and `web/premaid-sim.html` (src + MJCF + coloured meshes).
Scripting from the console: `PMSIM.setTarget(k, deg)`, `PMSIM.applyPose({r_kny_joint: 40})`, `PMSIM.reset()`.
