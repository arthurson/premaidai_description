"""Gazebo (gz-sim) + gz_ros2_control:  ros2 launch premaidai_description gz.launch.py     (needs ros_gz_sim, ros_gz_bridge, gz_ros2_control)
The collision meshes are the visual meshes and the URDF inertias are not all physical: expect to tune friction / PID before it stands."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('premaidai_description')
    xacro_file = PathJoinSubstitution([share, 'urdf', 'premaidai.urdf.xacro'])
    controllers = PathJoinSubstitution([share, 'config', 'controllers.yaml'])
    robot_description = ParameterValue(Command([FindExecutable(name='xacro'), ' ', xacro_file, ' use_gz:=true use_mock_hardware:=false controllers_file:=', controllers]), value_type=str)
    gz = IncludeLaunchDescription(PythonLaunchDescriptionSource([PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]), launch_arguments={'gz_args': '-r empty.sdf'}.items())
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': robot_description, 'use_sim_time': True}])
    spawn = Node(package='ros_gz_sim', executable='create', arguments=['-topic', 'robot_description', '-name', 'premaidai', '-z', '0.35'], output='screen')
    clock = Node(package='ros_gz_bridge', executable='parameter_bridge', arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
    jsb = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'])
    jtc = Node(package='controller_manager', executable='spawner', arguments=['joint_trajectory_controller'])
    return LaunchDescription([gz, rsp, spawn, clock, RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])), RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[jtc]))])
