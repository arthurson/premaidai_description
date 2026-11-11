"""MoveIt 2 demo with mock hardware:  ros2 launch premaidai_moveit_config demo.launch.py   (the base is fixed to the world: the robot hangs in the air)"""
import os

from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    description = os.path.join(get_package_share_directory('premaidai_description'), 'urdf', 'premaidai.urdf.xacro')
    config = (MoveItConfigsBuilder('premaidai', package_name='premaidai_moveit_config')
              .robot_description(file_path=description, mappings={'use_mock_hardware': 'true'})
              .robot_description_semantic(file_path='config/premaidai.srdf')
              .robot_description_kinematics(file_path='config/kinematics.yaml')
              .joint_limits(file_path='config/joint_limits.yaml')
              .trajectory_execution(file_path='config/moveit_controllers.yaml')
              .planning_pipelines(pipelines=['ompl'])
              .to_moveit_configs())
    return generate_demo_launch(config)
