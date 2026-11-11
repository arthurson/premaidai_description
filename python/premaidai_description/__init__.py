"""PreMaid AI robot description: URDF, MJCF (MuJoCo), meshes and small loaders.

    import premaidai_description as pm
    model = pm.load_mujoco()              # mujoco.MjModel (pip install mujoco)
    robot = pm.load_urdf()                # yourdfpy.URDF  (pip install yourdfpy)
    pm.load_pybullet(p)                   # body id in a pybullet client (pip install pybullet)
    pm.URDF_PATH, pm.MJCF_PATH, pm.PACKAGE_PATH        # same names as robot_descriptions.py modules

The files are those of https://github.com/arthurson/premaidai_description (Apache-2.0; meshes by kuroiwasi, model by chikuta).
Works from an installed wheel and from a git checkout.
"""
import os
import tempfile

__version__ = '0.1.0'

_HERE = os.path.dirname(os.path.abspath(__file__))
# installed wheel: data sits next to this file;  git checkout: data sits two levels up (python/premaidai_description/)
DESCRIPTION_PATH = _HERE if os.path.isdir(os.path.join(_HERE, 'meshes')) else os.path.dirname(os.path.dirname(_HERE))
# a directory that contains a folder called "premaidai_description": what package://premaidai_description/... is resolved against
PACKAGE_PATH = os.path.dirname(_HERE) if DESCRIPTION_PATH == _HERE else DESCRIPTION_PATH
REPOSITORY_PATH = DESCRIPTION_PATH
MESH_DIR = os.path.join(DESCRIPTION_PATH, 'meshes')
URDF_PATH = os.path.join(DESCRIPTION_PATH, 'urdf', 'premaidai.urdf')
XACRO_PATH = os.path.join(DESCRIPTION_PATH, 'robots', 'premaidai.urdf.xacro')
MJCF_PATH = os.path.join(DESCRIPTION_PATH, 'mujoco', 'premaidai.xml')

# the 25 position servos, in MJCF actuator order (head, right arm, left arm, right leg, left leg)
JOINT_NAMES = (
    'head_ny_joint', 'head_nz_joint', 'head_nx_joint',
    'r_shy_joint', 'r_shx_joint', 'r_ely_joint', 'r_elx_joint', 'r_wry_joint',
    'l_shy_joint', 'l_shx_joint', 'l_ely_joint', 'l_elx_joint', 'l_wry_joint',
    'r_hpz_joint', 'r_hpx_joint', 'r_hpy_joint', 'r_kny_joint', 'r_aky_joint', 'r_akx_joint',
    'l_hpz_joint', 'l_hpx_joint', 'l_hpy_joint', 'l_kny_joint', 'l_aky_joint', 'l_akx_joint')
KEYFRAMES = ('t_pose', 'arms_down', 'squat')              # <keyframe> names in the MJCF
SENSORS = ('imu_gyro', 'imu_acc', 'imu_quat', 'r_foot_touch', 'l_foot_touch', 'com')   # plus pos_<joint> and tau_<joint>


def load_mujoco(keyframe=None):
    """mujoco.MjModel of the robot (25 position servos, free base, floor, IMU / foot-touch / torque sensors, 3 keyframes, 6 cameras).
    With keyframe="t_pose" | "arms_down" | "squat" the returned (model, data) pair is already reset to that pose."""
    import mujoco
    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    if keyframe is None:
        return model
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, KEYFRAMES.index(keyframe))
    data.ctrl[:] = model.key_ctrl[KEYFRAMES.index(keyframe)]
    mujoco.mj_forward(model, data)
    return model, data


def urdf_path(absolute_meshes=False):
    """Path of the plain URDF. package://premaidai_description/meshes/x.dae is kept unless absolute_meshes=True: then a copy with
    absolute file names (for loaders that cannot resolve package://) is written to a temporary directory and its path returned."""
    if not absolute_meshes:
        return URDF_PATH
    with open(URDF_PATH, encoding='utf-8') as f:
        text = f.read().replace('package://premaidai_description/meshes/', MESH_DIR.replace('\\', '/') + '/')
    out = os.path.join(tempfile.mkdtemp(prefix='premaidai_urdf_'), 'premaidai.urdf')
    with open(out, 'w', encoding='utf-8') as f:
        f.write(text)
    return out


def load_urdf(**kwargs):
    """yourdfpy.URDF with the meshes loaded (pip install yourdfpy trimesh pycollada)."""
    import yourdfpy
    return yourdfpy.URDF.load(urdf_path(absolute_meshes=True), **kwargs)


def load_pybullet(client=None, **kwargs):
    """Load the URDF into pybullet and return the body id.  client: a pybullet module or a BulletClient (default: the module)."""
    if client is None:
        import pybullet as client
    kwargs.setdefault('useFixedBase', False)
    return client.loadURDF(urdf_path(absolute_meshes=True), **kwargs)
