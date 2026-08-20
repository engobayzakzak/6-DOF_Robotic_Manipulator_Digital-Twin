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

    # 4. Planning Pipeline (OMPL)
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planning_request_adapters/AddTimeOptimalParameterization '
                'default_planning_request_adapters/ResolveConstraintFrames '
                'default_planning_request_adapters/FixWorkspaceBounds '
                'default_planning_request_adapters/FixStartStateBounds '
                'default_planning_request_adapters/FixStartStateCollision '
                'default_planning_request_adapters/FixStartStatePathConstraints'
            ),
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_yaml = load_yaml('manipulator_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config['move_group'].update(ompl_yaml)

    # 5. Trajectory Execution & Controllers
    controllers_yaml = load_yaml('manipulator_moveit_config', 'config/moveit_controllers.yaml')
    moveit_controllers = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': controllers_yaml['moveit_simple_controller_manager'],
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }

    # Explicit Planning Scene Monitor Configuration
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_monitored_planning_scene': True,
    }

    # MoveGroup Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # Pre-configured RViz Node
    rviz_config_file = os.path.join(get_package_share_directory('manipulator_moveit_config'), 'launch', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            joint_limits_yaml,
        ],
    )

    # Static TF: world -> base_link
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base_link'],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # ros2_control Node
    ros2_controllers_path = os.path.join(
        get_package_share_directory('manipulator_moveit_config'),
        'config',
        'ros2_controllers.yaml'
    )
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, ros2_controllers_path],
        output='screen',
    )

    # Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
    )

    return LaunchDescription([
        static_tf,
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        move_group_node,
        rviz_node,
    ])
