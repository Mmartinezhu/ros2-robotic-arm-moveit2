from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction

def generate_launch_description():
    # Construye la configuración base de MoveIt
    moveit_config = (
        MoveItConfigsBuilder("robert_arm", package_name="robotarm_moveit_config")
        .robot_description(file_path="config/robotarm.urdf.xacro")
        .robot_description_semantic(file_path="config/robotarm.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # Robot description parameter
    robot_description = moveit_config.robot_description

    # Rutas de configuración de controladores
    ros2_controllers_path = os.path.join(
        get_package_share_directory("robotarm_moveit_config"),
        "config",
        "ros2_controllers.yaml"
    )

    # Nodo principal de ros2_control
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output="screen",
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="screen",
    )

    # Spawners de controladores con timers más largos
    joint_state_broadcaster = TimerAction(
        period=8.0,  # Aumentado a 8 segundos
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ]
    )

    manipulator_controller = TimerAction(
        period=12.0,  # Aumentado a 12 segundos
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["manipulator_controller", "--controller-manager", "/controller_manager"],
                output="screen",
            )
        ]
    )

    # Move group con timer
    move_group_node = TimerAction(
        period=15.0,  # Aumentado a 15 segundos
        actions=[
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                parameters=[moveit_config.to_dict()],
                output="screen",
            )
        ]
    )

    # RViz con timer
    rviz_config_path = os.path.join(
        get_package_share_directory("robotarm_moveit_config"),
        "config",
        "moveit.rviz"
    )
    rviz_node = TimerAction(
        period=18.0,  # Aumentado a 18 segundos
        actions=[
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_path],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                ],
                output="screen",
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        ros2_control_node,
        joint_state_broadcaster,
        manipulator_controller,
        move_group_node,
        rviz_node,
    ])
