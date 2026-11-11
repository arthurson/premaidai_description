# Troubleshooting

## The simulator

| symptom | cause / fix |
|---|---|
| Blank page, "loading MuJoCo WASM" forever | WebGL or WebAssembly is blocked; use a current Chrome / Edge / Firefox / Safari. Open the console: a CSP or file:// problem shows up there. Serve the folder over http (`python -m http.server`) instead of opening it from disk if the browser blocks `fetch` on `file://`. |
| The robot falls over right away | You pressed *Reset* with a pose that is not stable (e.g. a raised leg). Start from `arms_down` / `squat`; with *Hang body* on the base is pinned. Pushes above about 9 N (0.1 s) topple it by design. |
| Motions play but dance poses look wrong | Set *Original loop* correctly (follow the file / play once) and check the `0x19` speed/stretch of the file in the motion info line. |
| Recording (WebM) is empty | The browser has no `MediaRecorder` / `captureStream` or the tab was in the background. GIF recording works without them. |
| Offline mode does not work | The service worker only exists over http(s) (not `file://`) and only after one full load. Clear the site data once if an old version is cached (the cache is `premaid-sim-v1`). |
| Share link opens a different motion | The link stores the motion *title*; if you renamed it in `pma/` regenerate the library (`python pma/make_index.py`). |

## Connecting to a real robot

| symptom | cause / fix |
|---|---|
| No *Connect* button / "Web Serial not supported" | Safari and iOS have no Web Serial. Use the **WS** row with `tools/ws_bridge.py` on a Raspberry Pi / PC (`python tools/ws_bridge.py /dev/ttyUSB0 --host 0.0.0.0`) and open the page over http from it. |
| WebSocket refuses to connect | An https page can only open `wss://` (or `ws://localhost`). Serve the simulator over http from the bridge, or put TLS in front of it. |
| The port list is empty | Pair the RN42 (Bluetooth SPP) first so the OS exposes a serial port; on Linux add your user to `dialout`. |
| Connected, but nothing moves | Baud rate must be 115200. Press *T pose* once: if the status line says "ack error" another motion is still running — press **STOP** first. |
| Status table stays empty / "timeouts" grow | Your firmware does not answer `0x01 … 05` (only the battery query). Turn *Poll servo status* off. |
| The page loses the link after a few minutes | The simulator reconnects by itself (USB: same vendor/product id; WebSocket: back-off 1…5 s) as long as you did not press *Disconnect*. Check the cable / Bluetooth range. |
| The robot goes limp when I start teaching | That is what *Start teaching (release)* does: it sends raw 0 to the released side. Hang the robot first; *Hold current pose* stiffens the servos again. |
| E-stop does nothing on the robot | It sends the stop packets, it does **not** release the servos. With the robot disconnected it only stops the page. |

## Real-robot safety

* Never test a new pose or a policy with the robot standing on a table edge. Hang it (or hold it) first.
* The LED / `0x1E` box can overwrite stored motions on premaiduino2. Do not use it on that firmware.
* Check *Safe limits* before streaming a `.pma` from an unknown source.

## Building

| symptom | fix |
|---|---|
| `make_premaid_mjcf.py --verify` says STALE | run `python mujoco/make_premaid_mjcf.py` and commit `mujoco/premaidai.xml` + `web/premaid-sim.html`. Edit `web/src/`, never the generated html. |
| `tools/make_urdf.py` / `make_menagerie.py` / `export_gltf.py` / `make_ros2.py` / `make_docs.py` STALE | run the same script without `--verify` and commit its output. |
| `pip install pybullet` takes forever | there is no wheel for new Python versions, it compiles from source (~10+ min on one core). The loader tests skip when it is missing. |
