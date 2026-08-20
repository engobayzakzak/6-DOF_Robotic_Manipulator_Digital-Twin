import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_manipulator_gazebo = get_package_share_directory('manipulator_gazebo')
    pkg_manipulator_description = get_package_share_directory('manipulator_description')
    pkg_manipulator_moveit_config = get_package_share_directory('manipulator_moveit_config')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_manipulator_gazebo, 'worlds', 'workcell.sdf')

    # 1. Process URDF with gz_ros2_control enabled
    xacro_file = os.path.join(pkg_manipulator_description, 'urdf', 'manipulator.urdf.xacro')
    doc = xacro.process_file(xacro_file, mappings={'use_mock_hardware': 'false'})
    robot_description = {'robot_description': doc.toxml()}

    # 2. Gazebo Sim
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 3 {world_path}'}.items(),
    )

    # 3. Spawn Robot Model into Gazebo
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

    # 4. Clock Bridge (Gazebo Sim -> ROS 2)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # 5. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # 6. Controller Spawners
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', os.path.join(pkg_manipulator_moveit_config, 'config', 'ros2_controllers.yaml')],
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--param-file', os.path.join(pkg_manipulator_moveit_config, 'config', 'ros2_controllers.yaml')],
    )

    return LaunchDescription([
        gazebo_sim,
        spawn_robot,
        gz_bridge,
        robot_state_publisher,
        joint_state_broadcaster,
        arm_controller,
    ])
