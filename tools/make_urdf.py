#!/usr/bin/env python3
"""Expand robots/premaidai.urdf.xacro into the plain urdf/premaidai.urdf (needs: pip install xacro).
    python tools/make_urdf.py            # write urdf/premaidai.urdf
    python tools/make_urdf.py --verify   # exit 1 when the committed file is stale (CI)
Mesh paths stay package://premaidai_description/meshes/..., so ROS, pybullet (with a package path), yourdfpy and Isaac importers all work.
<transmission> and <gazebo> blocks (ROS 1 / Gazebo Classic) are dropped from this generic file; the xacro keeps them for the ROS 1 package."""
import importlib.util, os, sys, xml.dom.minidom
HERE = os.path.dirname(os.path.abspath(__file__)); ROOT = os.path.dirname(HERE); OUT = os.path.join(ROOT, 'urdf', 'premaidai.urdf')
spec = importlib.util.spec_from_file_location('gen', os.path.join(ROOT, 'mujoco', 'make_premaid_mjcf.py')); gen = importlib.util.module_from_spec(spec); spec.loader.exec_module(gen)
doc = xml.dom.minidom.parseString(gen.expand_xacro())
for n in [n for n in doc.childNodes if n.nodeType == n.COMMENT_NODE]: doc.removeChild(n)      # xacro writes its (random) temp path into a comment
robot = doc.documentElement
for n in [n for n in robot.childNodes if n.nodeType == n.ELEMENT_NODE and n.tagName in ('transmission', 'gazebo')]: robot.removeChild(n)      # ROS 1 / Gazebo Classic only: the plain URDF stays generic
text = doc.toprettyxml(indent='  ')
text = '\n'.join(l for l in text.split('\n') if l.strip()) + '\n'
if '--verify' in sys.argv:
    cur = open(OUT, encoding='utf-8', newline='').read() if os.path.exists(OUT) else ''
    print('OK: urdf/premaidai.urdf is up to date' if cur == text else 'STALE: urdf/premaidai.urdf (run python tools/make_urdf.py and commit)'); sys.exit(0 if cur == text else 1)
open(OUT, 'w', encoding='utf-8', newline='\n').write(text); print('wrote', OUT, len(text) // 1024, 'KB')
