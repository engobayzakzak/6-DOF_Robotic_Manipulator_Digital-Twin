import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    pkg_manipulator_gazebo = get_package_share_directory('manipulator_gazebo')
    pkg_manipulator_description = get_package_share_directory('manipulator_description')
    pkg_manipulator_moveit_config = get_package_share_directory('manipulator_moveit_config')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_manipulator_gazebo, 'worlds', 'workcell.sdf')

    # 1. Process URDF for Gazebo Sim
    xacro_file = os.path.join(pkg_manipulator_description, 'urdf', 'manipulator.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={'use_mock_hardware': 'false'})
    robot_description = {'robot_description': doc.toxml()}

    # 2. Semantic Robot Description & MoveIt Parameters
    robot_description_semantic = {
        'robot_description_semantic': load_file('manipulator_moveit_config', 'config/arm_6dof.srdf')
    }
    
    # Wrap kinematics properly for MoveIt 2 loader
    kinematics_dict = load_yaml('manipulator_moveit_config', 'config/kinematics.yaml')
    robot_description_kinematics = {'robot_description_kinematics': kinematics_dict}
    joint_limits_yaml = load_yaml('manipulator_moveit_config', 'config/joint_limits.yaml')

    # Jazzy-compliant OMPL planning pipeline
    ompl_yaml = load_yaml('manipulator_moveit_config', 'config/ompl_planning.yaml')
    ompl_planning_pipeline_config = {
        'move_group': {
            'planning_plugin': 'ompl_interface/OMPLPlanner',
            'request_adapters': (
                'default_planning_request_adapters/ResolveConstraintFrames '
                'default_planning_request_adapters/ValidateWorkspaceBounds '
                'default_planning_request_adapters/CheckStartStateBounds '
                'default_planning_request_adapters/CheckStartStateCollision'
            ),
            'response_adapters': (
                'default_planning_response_adapters/AddTimeOptimalParameterization'
            ),
            'start_state_max_bounds_error': 0.1,
        }
    }
    ompl_planning_pipeline_config['move_group'].update(ompl_yaml)

    controllers_yaml = load_yaml('manipulator_moveit_config', 'config/moveit_controllers.yaml')
    moveit_controllers = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager': controllers_yaml['moveit_simple_controller_manager'],
    }

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.5,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.05,
    }

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_monitored_planning_scene': True,
    }

    # Gazebo Sim
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 3 {world_path}'}.items(),
    )

    # Spawn Robot Entity
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', doc.toxml(),
            '-name', 'arm_6dof',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
        ],
    )

    # Clock Bridge (Gazebo -> ROS 2)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # Controller Spawners
    ros2_controllers_path = os.path.join(pkg_manipulator_moveit_config, 'config', 'ros2_controllers.yaml')
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', ros2_controllers_path],
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--param-file', ros2_controllers_path],
    )

    # MoveGroup Node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {'use_sim_time': True},
        ],
    )

    # RViz2 Node
    rviz_config_file = os.path.join(pkg_manipulator_moveit_config, 'launch', 'moveit.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            joint_limits_yaml,
            {'use_sim_time': True},
        ],
    )

    return LaunchDescription([
        gazebo_sim,
        spawn_robot,
        gz_bridge,
        robot_state_publisher,
        joint_state_broadcaster,
        arm_controller,
        move_group_node,
        rviz_node,
    ])
