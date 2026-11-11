# premaidai-description

URDF, MuJoCo MJCF and meshes of the **PreMaid AI** humanoid (25 position servos), with small loaders.

```python
import premaidai_description as pm

model = pm.load_mujoco()                      # mujoco.MjModel: IMU / foot-touch / torque sensors, 3 keyframes, 6 cameras
model, data = pm.load_mujoco("arms_down")     # reset to a keyframe (t_pose | arms_down | squat)
robot = pm.load_urdf()                        # yourdfpy.URDF
body = pm.load_pybullet(pybullet)             # pybullet body id
pm.URDF_PATH, pm.MJCF_PATH, pm.PACKAGE_PATH   # same names as robot_descriptions.py modules
```

Extras: `pip install premaidai-description[mujoco]`, `[urdf]`, `[pybullet]`.
Sources, web simulator (MuJoCo WASM, `.pma` player, motion editor) and documentation: <https://github.com/arthurson/premaidai_description>.
Apache-2.0. Meshes by kuroiwasi, ROS model by chikuta.
