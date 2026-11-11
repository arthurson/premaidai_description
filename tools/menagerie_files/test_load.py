"""Menagerie-style smoke test:  python test_load.py   (needs: pip install mujoco)"""
import os
import mujoco

here = os.path.dirname(os.path.abspath(__file__))
m = mujoco.MjModel.from_xml_path(os.path.join(here, "scene.xml"))
d = mujoco.MjData(m)
assert m.nu == 25 and m.nkey == 3 and m.nsensor == 56
for key in range(m.nkey):
    mujoco.mj_resetDataKeyframe(m, d, key)
    for _ in range(int(2 / m.opt.timestep)):
        d.ctrl[:] = m.key_ctrl[key]
        mujoco.mj_step(m, d)
    assert d.qpos[2] > 0.12, ("fell", key, d.qpos[2])
print("OK", m.nbody, "bodies", m.ngeom, "geoms")
