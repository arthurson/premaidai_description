"""ros2_control with mock hardware + joint_trajectory_controller:  ros2 launch premaidai_description control.launch.py
Send a goal:  ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory ..."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('premaidai_description')
    xacro_file = PathJoinSubstitution([share, 'urdf', 'premaidai.urdf.xacro'])
    controllers = PathJoinSubstitution([share, 'config', 'controllers.yaml'])
    robot_description = ParameterValue(Command([FindExecutable(name='xacro'), ' ', xacro_file, ' use_mock_hardware:=', LaunchConfiguration('use_mock_hardware')]), value_type=str)
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': robot_description}])
    cm = Node(package='controller_manager', executable='ros2_control_node', parameters=[{'robot_description': robot_description}, controllers], output='screen')
    jsb = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'])
    jtc = Node(package='controller_manager', executable='spawner', arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'])
    rviz = Node(package='rviz2', executable='rviz2', condition=IfCondition(LaunchConfiguration('rviz')),
                arguments=['-d', PathJoinSubstitution([share, 'rviz', 'premaidai.rviz'])])
    return LaunchDescription([
        DeclareLaunchArgument('use_mock_hardware', default_value='true'), DeclareLaunchArgument('rviz', default_value='true'),
        rsp, cm, jsb, rviz,
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[jtc])),
    ])
