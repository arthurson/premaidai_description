"""Static checks of the ROS 2 packages (ros2/): no ROS 2 installation is needed, so nothing here runs a node, ros2_control or MoveIt."""
import ast, os, re, shutil, subprocess, sys, tempfile, unittest, xml.etree.ElementTree as ET
try:
    import xacro, yaml
except ImportError:
    xacro = None
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__))); D = os.path.join(ROOT, 'ros2', 'premaidai_description'); M = os.path.join(ROOT, 'ros2', 'premaidai_moveit_config')


@unittest.skipIf(xacro is None, 'xacro / PyYAML not installed')
class Ros2Test(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.share = tempfile.mkdtemp(prefix='ros2share_'); pkg = os.path.join(cls.share, 'premaidai_description'); os.makedirs(os.path.join(pkg, 'urdf'))
        shutil.copy(os.path.join(ROOT, 'urdf', 'premaidai.urdf'), os.path.join(pkg, 'urdf'))
        for f in ('premaidai.urdf.xacro', 'premaidai.ros2_control.xacro'):                       # what `$(find premaidai_description)` resolves to after `colcon build`
            shutil.copy(os.path.join(D, 'urdf', f), os.path.join(pkg, 'urdf', f))
            p = os.path.join(pkg, 'urdf', f); text = open(p, encoding='utf-8').read().replace('$(find premaidai_description)', pkg); open(p, 'w', encoding='utf-8').write(text)
        cls.top = os.path.join(pkg, 'urdf', 'premaidai.urdf.xacro')
        cls.urdf = ET.parse(os.path.join(ROOT, 'urdf', 'premaidai.urdf')).getroot()
        cls.rev = {j.get('name'): j for j in cls.urdf.findall('joint') if j.get('type') == 'revolute'}

    def expand(self, **kw):
        return ET.fromstring(xacro.process_file(self.top, mappings={k: ('true' if v is True else 'false' if v is False else str(v)) for k, v in kw.items()}).toxml())

    def test_generated_files_are_up_to_date(self):
        r = subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'make_ros2.py'), '--verify'], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stdout + r.stderr)

    def test_mock_hardware_expansion(self):
        r = self.expand(use_mock_hardware=True); self.assertNotIn('$(', ET.tostring(r, encoding='unicode'))
        rc = r.find('ros2_control'); self.assertEqual(rc.find('hardware/plugin').text, 'mock_components/GenericSystem')
        joints = rc.findall('joint'); self.assertEqual(sorted(j.get('name') for j in joints), sorted(self.rev))
        for j in joints:
            lim = self.rev[j.get('name')].find('limit'); p = {e.get('name'): e.text for e in j.find('command_interface')}
            self.assertAlmostEqual(float(p['min']), float(lim.get('lower')), places=6); self.assertAlmostEqual(float(p['max']), float(lim.get('upper')), places=6)
        self.assertEqual(len(r.findall('link')), len(self.urdf.findall('link'))); self.assertIsNone(r.find('gazebo'))

    def test_gazebo_expansion(self):
        r = self.expand(use_gz=True, use_mock_hardware=False, controllers_file='/c.yaml')
        self.assertEqual(r.find('ros2_control/hardware/plugin').text, 'gz_ros2_control/GazeboSimSystem')
        self.assertEqual(r.find('gazebo/plugin/parameters').text, '/c.yaml')

    def test_real_hardware_expansion(self):
        r = self.expand(use_mock_hardware=False, serial_port='/dev/ttyACM0'); self.assertEqual(r.find('ros2_control/hardware/plugin').text, 'premaidai_hardware/PreMaidSystem')
        self.assertEqual({p.get('name'): p.text for p in r.findall('ros2_control/hardware/param')}['serial_port'], '/dev/ttyACM0')

    def yaml_of(self, *p): return yaml.safe_load(open(os.path.join(*p), encoding='utf-8'))

    def test_controller_configs_match_the_urdf(self):
        c = self.yaml_of(D, 'config', 'controllers.yaml'); self.assertEqual(sorted(c['joint_trajectory_controller']['ros__parameters']['joints']), sorted(self.rev))
        self.assertEqual(c['controller_manager']['ros__parameters']['joint_trajectory_controller']['type'], 'joint_trajectory_controller/JointTrajectoryController')
        mc = self.yaml_of(M, 'config', 'moveit_controllers.yaml')['moveit_simple_controller_manager']['joint_trajectory_controller']; self.assertEqual(sorted(mc['joints']), sorted(self.rev))
        lim = self.yaml_of(M, 'config', 'joint_limits.yaml')['joint_limits']; self.assertEqual(sorted(lim), sorted(self.rev))
        for n, l in lim.items(): self.assertAlmostEqual(l['max_velocity'], float(self.rev[n].find('limit').get('velocity')), places=6)
        self.assertEqual(sorted(self.yaml_of(M, 'config', 'initial_positions.yaml')['initial_positions']), sorted(self.rev))

    def test_srdf_is_consistent(self):
        s = ET.parse(os.path.join(M, 'config', 'premaidai.srdf')).getroot(); links = {l.get('name') for l in self.urdf.findall('link')}
        groups = {g.get('name'): g for g in s.findall('group')}
        for g in groups.values():
            for ch in g.findall('chain'): self.assertIn(ch.get('base_link'), links); self.assertIn(ch.get('tip_link'), links)
            for sub in g.findall('group'): self.assertIn(sub.get('name'), groups)
        for st in s.findall('group_state'):
            self.assertEqual(sorted(j.get('name') for j in st.findall('joint')), sorted(self.rev)); self.assertIn(st.get('group'), groups)
        for dc in s.findall('disable_collisions'): self.assertIn(dc.get('link1'), links); self.assertIn(dc.get('link2'), links)
        k = self.yaml_of(M, 'config', 'kinematics.yaml'); self.assertEqual(sorted(k), ['head', 'l_arm', 'l_leg', 'r_arm', 'r_leg'])
        o = self.yaml_of(M, 'config', 'ompl_planning.yaml'); self.assertTrue(set(groups) <= set(o))
        # every limb chain must really be a chain of revolute joints ending at the tip link
        for g, n in (('r_arm', 5), ('l_arm', 5), ('r_leg', 6), ('l_leg', 6), ('head', 3)):
            tip = groups[g].find('chain').get('tip_link'); cnt = 0; cur = tip; parent = {j.find('child').get('link'): j for j in self.urdf.findall('joint')}
            while cur != 'base_link': j = parent[cur]; cnt += j.get('type') == 'revolute'; cur = j.find('parent').get('link')
            self.assertEqual(cnt, n, g)

    def test_launch_files_and_packages(self):
        for root in (D, M):
            for f in os.listdir(os.path.join(root, 'launch')):
                p = os.path.join(root, 'launch', f); ast.parse(open(p, encoding='utf-8').read(), p)
                self.assertIn('def generate_launch_description', open(p, encoding='utf-8').read())
        for root, name in ((D, 'premaidai_description'), (M, 'premaidai_moveit_config')):
            pk = ET.parse(os.path.join(root, 'package.xml')).getroot(); self.assertEqual(pk.find('name').text, name); self.assertEqual(pk.get('format'), '3')
            self.assertIn('project(%s)' % name, open(os.path.join(root, 'CMakeLists.txt')).read())
        deps = {e.text for e in ET.parse(os.path.join(D, 'package.xml')).getroot().iter('exec_depend')}
        for need in ('xacro', 'robot_state_publisher', 'controller_manager', 'joint_trajectory_controller', 'gz_ros2_control', 'ros_gz_sim'): self.assertIn(need, deps)


if __name__ == '__main__':
    unittest.main()
