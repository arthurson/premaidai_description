"""View the robot in RViz with sliders:  ros2 launch premaidai_description display.launch.py"""
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution([FindPackageShare('premaidai_description'), 'urdf', 'premaidai.urdf.xacro'])
    robot_description = ParameterValue(Command([FindExecutable(name='xacro'), ' ', xacro_file]), value_type=str)
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher', parameters=[{'robot_description': robot_description}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', PathJoinSubstitution([FindPackageShare('premaidai_description'), 'rviz', 'premaidai.rviz'])]),
    ])
