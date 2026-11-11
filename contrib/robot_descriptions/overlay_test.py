"""Try the robot_descriptions.py entry WITHOUT a PR: copies the installed robot_descriptions to a temp dir, adds the module and the two
registry entries (pointing at this local checkout, current HEAD) and loads the robot through robot_descriptions' own loaders.
    pip install robot_descriptions mujoco yourdfpy trimesh pycollada && git commit first (the clone only sees committed files)
    python contrib/robot_descriptions/overlay_test.py"""
import os, re, shutil, subprocess, sys, tempfile
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(os.path.dirname(HERE))
import robot_descriptions
tmp = tempfile.mkdtemp(prefix='rd_overlay_'); dst = os.path.join(tmp, 'robot_descriptions')
shutil.copytree(os.path.dirname(robot_descriptions.__file__), dst, ignore=shutil.ignore_patterns('__pycache__'))
shutil.copy(os.path.join(HERE, 'premaidai_description.py'), dst)
commit = subprocess.check_output(['git', '-C', ROOT, 'rev-parse', 'HEAD'], text=True).strip()
def patch(fname, marker, entry):
    p = os.path.join(dst, fname); s = open(p, encoding='utf-8').read(); i = s.index(marker) + len(marker); open(p, 'w', encoding='utf-8').write(s[:i] + '\n' + entry + s[i:])
d = open(os.path.join(HERE, 'descriptions_entry.txt')).read()
patch('_descriptions.py', 'DESCRIPTIONS: Dict[str, Description] = {', d)
r = open(os.path.join(HERE, 'repositories_entry.txt')).read().replace('@COMMIT@', commit).replace('https://github.com/arthurson/premaidai_description.git', ROOT)
patch('_repositories.py', 'REPOSITORIES: Dict[str, Repository] = {', r)
code = '''
import os, sys
from robot_descriptions import premaidai_description as pd
print("REPOSITORY_PATH", pd.REPOSITORY_PATH); assert os.path.exists(pd.URDF_PATH) and os.path.exists(pd.MJCF_PATH)
from robot_descriptions.loaders.mujoco import load_robot_description as mj
m = mj("premaidai_description"); print("mujoco nq", m.nq, "nu", m.nu, "nkey", m.nkey); assert (m.nq, m.nu) == (32, 25)
from robot_descriptions.loaders.yourdfpy import load_robot_description as yd
r = yd("premaidai_description"); print("yourdfpy actuated joints", len(r.actuated_joint_names)); assert len(r.actuated_joint_names) == 25
print("OVERLAY OK")
'''
env = dict(os.environ, PYTHONPATH=tmp, ROBOT_DESCRIPTIONS_CACHE=os.path.join(tmp, 'cache'))
sys.exit(subprocess.run([sys.executable, '-c', code], env=env, cwd=tmp).returncode)
