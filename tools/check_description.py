#!/usr/bin/env python3
"""CI checks for the ROS description (no ROS installation required).

    pip install xacro
    python tools/check_description.py

It checks the source xacro/URDF files, expands robots/premaidai.urdf.xacro
(the same $(find premaidai_description) rewrite as mujoco/make_premaid_mjcf.py)
and validates the resulting URDF:

  * no <jont> / </jont> typo anywhere in the description files
  * every <transmission> has a name, a <joint name="..."> and an <actuator>
  * every transmission joint exists and every non-fixed joint has one
  * parent/child links exist and form a single tree rooted at base_link
  * revolute joints have a sane limit and every referenced mesh exists

Exit code 1 lists every problem.
"""
import os
import re
import shutil
import sys
import tempfile
import xml.etree.ElementTree as ET

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKIP_DIRS = {'.git', '__pycache__', 'node_modules', 'vendor'}
TEXT_SUFFIXES = {'.xacro', '.urdf', '.xml', '.py', '.md', '.html', '.js', '.sh', '.yml', '.yaml'}
TYPO_RE = re.compile(r'<\s*/?\s*(jont|jiont)\b', re.I)
MESH_PREFIX = 'package://premaidai_description/'

errors = []


def fail(msg):
    errors.append(msg)


def local_name(tag):
    return tag.rsplit('}', 1)[-1]


def text_files():
    for root, dirs, files in os.walk(PKG):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        for name in files:
            if os.path.splitext(name)[1].lower() not in TEXT_SUFFIXES:
                continue
            path = os.path.join(root, name)
            if name == 'check_description.py':      # this file contains the typo pattern itself
                continue
            if os.path.getsize(path) > 3 * 1024 * 1024:   # generated bundles
                continue
            yield path


def check_typos():
    for path in text_files():
        try:
            with open(path, encoding='utf-8') as f:
                for n, line in enumerate(f, 1):
                    if TYPO_RE.search(line):
                        fail('%s:%d: <jont> typo: %s' % (os.path.relpath(path, PKG), n, line.strip()))
        except UnicodeDecodeError:
            pass


def check_source_xml():
    for path in text_files():
        if os.path.splitext(path)[1].lower() not in ('.xacro', '.urdf'):
            continue
        try:
            root = ET.parse(path).getroot()
        except ET.ParseError as e:
            fail('%s: XML parse error: %s' % (os.path.relpath(path, PKG), e))
            continue
        for el in root.iter():
            if local_name(el.tag) in ('jont', 'jiont'):
                fail('%s: element <%s>' % (os.path.relpath(path, PKG), local_name(el.tag)))
            elif local_name(el.tag) == 'transmission':
                if not el.get('name'):
                    fail('%s: <transmission> without name' % os.path.relpath(path, PKG))
                for child in ('joint', 'actuator'):
                    if el.find(child) is None:
                        fail('%s: <transmission> without <%s>' % (os.path.relpath(path, PKG), child))


def expand_xacro():
    import xacro
    tmp = tempfile.mkdtemp(prefix='premaidai_check_')
    try:
        for d in ('robots', 'urdf'):
            shutil.copytree(os.path.join(PKG, d), os.path.join(tmp, d))
        for root, _, files in os.walk(tmp):
            for name in files:
                path = os.path.join(root, name)
                with open(path, encoding='utf-8') as f:
                    text = f.read()
                with open(path, 'w', encoding='utf-8') as f:
                    f.write(text.replace('$(find premaidai_description)', tmp))
        return xacro.process_file(os.path.join(tmp, 'robots', 'premaidai.urdf.xacro')).toxml()
    finally:
        shutil.rmtree(tmp, ignore_errors=True)


def check_expanded(text):
    try:
        root = ET.fromstring(text)
    except ET.ParseError as e:
        fail('expanded URDF: XML parse error: %s' % e)
        return

    links = {}
    for el in root.findall('link'):
        name = el.get('name')
        if not name:
            fail('expanded URDF: <link> without name')
        elif name in links:
            fail('expanded URDF: duplicate link %s' % name)
        links[name] = el

    joints = {}
    nonfixed = {}
    for el in root.findall('joint'):
        name = el.get('name')
        if not name:
            fail('expanded URDF: <joint> without name')
            continue
        joints[name] = el
        parent = el.find('parent')
        child = el.find('child')
        if parent is None or child is None:
            fail('expanded URDF: joint %s without parent/child' % name)
            continue
        if parent.get('link') not in links:
            fail('expanded URDF: joint %s parent %s missing' % (name, parent.get('link')))
        if child.get('link') not in links:
            fail('expanded URDF: joint %s child %s missing' % (name, child.get('link')))
        if el.get('type') != 'fixed':
            nonfixed[name] = el
            lim = el.find('limit')
            if lim is None:
                fail('expanded URDF: movable joint %s has no <limit>' % name)
            else:
                for attr in ('lower', 'upper', 'effort', 'velocity'):
                    if lim.get(attr) is None:
                        fail('expanded URDF: joint %s limit missing %s' % (name, attr))
                try:
                    if float(lim.get('lower')) > float(lim.get('upper')):
                        fail('expanded URDF: joint %s lower > upper' % name)
                    if float(lim.get('effort')) <= 0:
                        fail('expanded URDF: joint %s effort <= 0' % name)
                except (TypeError, ValueError):
                    fail('expanded URDF: joint %s has a non-numeric limit' % name)
        if el.get('type') == 'revolute':
            axis = el.find('axis')
            if axis is None or len((axis.get('xyz') or '').split()) != 3:
                fail('expanded URDF: revolute joint %s has no 3-axis' % name)

    # single tree rooted at base_link
    if 'base_link' not in links:
        fail('expanded URDF: no base_link')
    children = {}
    for name, el in joints.items():
        child = el.find('child')
        if child is not None:
            children.setdefault(child.get('link'), []).append(name)
    for link in links:
        parents = children.get(link, [])
        if link == 'base_link':
            if parents:
                fail('expanded URDF: base_link has parent joints: %s' % parents)
        elif len(parents) != 1:
            fail('expanded URDF: link %s has %d parent joints' % (link, len(parents)))
    seen = set()
    stack = ['base_link']
    while stack:
        link = stack.pop()
        if link in seen:
            fail('expanded URDF: link %s reachable twice (cycle)' % link)
            continue
        seen.add(link)
        for name, el in joints.items():
            child = el.find('child')
            if el.find('parent').get('link') == link and child is not None:
                stack.append(child.get('link'))
    for link in links:
        if link not in seen:
            fail('expanded URDF: link %s is not connected to base_link' % link)

    # transmissions
    by_joint = {}
    names = set()
    for el in root.findall('transmission'):
        name = el.get('name')
        if not name:
            fail('expanded URDF: <transmission> without name')
        elif name in names:
            fail('expanded URDF: duplicate transmission %s' % name)
        names.add(name)
        j = el.find('joint')
        a = el.find('actuator')
        if j is None or not j.get('name'):
            fail('expanded URDF: transmission %s has no <joint name>' % name)
            continue
        if a is None or not a.get('name'):
            fail('expanded URDF: transmission %s has no <actuator name>' % name)
        joint = j.get('name')
        if joint not in joints:
            fail('expanded URDF: transmission %s references unknown joint %s' % (name, joint))
        elif joint in by_joint:
            fail('expanded URDF: joint %s has two transmissions' % joint)
        by_joint[joint] = name
    for joint in nonfixed:
        if joint not in by_joint:
            fail('expanded URDF: movable joint %s has no transmission' % joint)
    if len(nonfixed) != 25:
        fail('expected 25 movable joints, found %d' % len(nonfixed))
    if len(root.findall('transmission')) != 25:
        fail('expected 25 transmissions, found %d' % len(root.findall('transmission')))

    # meshes
    for el in root.iter('mesh'):
        fn = el.get('filename') or ''
        if fn.startswith(MESH_PREFIX):
            path = os.path.join(PKG, fn[len(MESH_PREFIX):].replace('/', os.sep))
            if not os.path.isfile(path):
                fail('expanded URDF: mesh not found: %s' % fn)


def main():
    check_typos()
    check_source_xml()
    try:
        text = expand_xacro()
    except ImportError:
        fail('xacro is not installed (pip install xacro)')
        text = None
    except Exception as e:
        fail('xacro expansion failed: %s' % e)
        text = None
    if text:
        check_expanded(text)
    if errors:
        print('FAILED: %d problem(s)' % len(errors))
        for msg in errors:
            print('  - ' + msg)
        return 1
    print('OK: xacro expands to a valid URDF; 25 transmissions reference existing joints; no <jont> typo')
    return 0


if __name__ == '__main__':
    sys.exit(main())
