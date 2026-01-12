import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_fanuc = get_package_share_directory('fanuc_robot')
    urdf_file = os.path.join(pkg_fanuc, 'urdf', 'fanuc_crx10.urdf.xacro')

    # Process xacro
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # Gazebo resource path
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', value=pkg_fanuc
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Bridge clock
    gz_ros_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Start Gazebo
    start_gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    # Spawn robot
    spawn_robot = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-topic', '/robot_description',
            '-name', 'fanuc_crx10',
            '-z', '0.5'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        start_gazebo,
        robot_state_publisher,
        gz_ros_bridge,
        spawn_robot
    ])
