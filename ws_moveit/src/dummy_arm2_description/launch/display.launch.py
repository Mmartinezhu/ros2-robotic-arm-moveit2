from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_path = get_package_share_directory('dummy_arm2_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'dummy_arm2.xacro')
    robot_description_config = xacro.process_file(xacro_file).toxml()

    rviz_config_file = os.path.join(pkg_path, 'rviz', 'urdf.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config}],
            output='screen'
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])

