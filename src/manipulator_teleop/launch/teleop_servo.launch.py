import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # 1. Process Robot Description URDF/Xacro
    desc_pkg_share = get_package_share_directory('manipulator_description')
    xacro_file = os.path.join(desc_pkg_share, 'urdf', 'manipulator.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={'use_mock_hardware': 'true'})
    robot_description = {'robot_description': doc.toxml()}

    # 2. Semantic Robot Description (SRDF)
    robot_description_semantic_config = load_file('manipulator_moveit_config', 'config/arm_6dof.srdf')
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config}

    # 3. Kinematics & Joint Limits
    kinematics_yaml = load_yaml('manipulator_moveit_config', 'config/kinematics.yaml')
    joint_limits_yaml = load_yaml('manipulator_moveit_config', 'config/joint_limits.yaml')

    # 4. Servo Configuration
    servo_yaml = load_yaml('manipulator_teleop', 'config/servo_config.yaml')
    servo_params = {'moveit_servo': servo_yaml}

    # MoveIt Servo Realtime Jacobian IK Node
    servo_node = Node(
        package='moveit_servo',
        executable='servo_node_main',
        name='servo_node',
        output='screen',
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
        ],
    )

    return LaunchDescription([
        servo_node
    ])
