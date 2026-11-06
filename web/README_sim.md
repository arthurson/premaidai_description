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

## GitHub Pages
Repo → Settings → Pages → *Deploy from a branch* → `master` / `/ (root)`. After a minute:
`https://<user>.github.io/premaidai_description/web/premaid-sim.html` (the repo-root `index.html` redirects there).
Everything is loaded by relative path (`vendor/three.min.js`, `vendor/OrbitControls.js`, `vendor/mujoco/mujoco.{js,wasm}`), so no build step on the server.
`.nojekyll` in the root stops Pages from running Jekyll over the repo.

## .pma replay
Load a PreMaid AI motion file (`📂 載入 .pma`, drag & drop onto the view, or the built-in `範例`); `files/pma.zip` in
`arthurson/premaiduino2` has samples.  UTF-8 and Shift-JIS files are both accepted.

* Packets `[LEN][CMD]…[XOR]`; `0x18` = raw ICS position per servo (`0` = release -> servo stiffness 0), `payload[1]` = SPD.
* Frame time = `max(SPD × 15 ms, largest travel / 12.7 pulse·ms⁻¹)` — the rule used by `motionPlaybackUpdate()` in `PmaProtocol.h`.
  `平滑` interpolates from the previous frame to the current one over that time; `逐幀跳` sends the target at frame start.
* Angle: `θ = sign × (raw − 7500) × 270/8000°`; travel = firmware limits shifted by `home − baseline` (`PMA_CAL` in
  `mujoco/make_premaid_mjcf.py`).  Signs were verified on the 10 sample files; shoulder-pitch / wrist limits therefore differ
  from the URDF's.  Servos marked 推定 (hip yaw, head, upper-arm / wrist twist) have an unverifiable absolute direction —
  tick `反向` in the PMA table if a motion looks mirrored.
* Playback is open loop (no balance control).  `0x19` speed / stretch packets are ignored.
