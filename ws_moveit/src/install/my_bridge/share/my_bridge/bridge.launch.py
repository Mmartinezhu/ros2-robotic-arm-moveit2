from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_bridge',
            executable='joint_states_to_delta_int',
            name='joint_states_to_delta_int',
            parameters=[{
                'joint_names': ['rev_1','rev_2','rev_3'],
                'output_topic': '/motor_angles',
                'deadband_deg': 0.3,
                'max_step_deg': 8.0,
                'pad_size': 3,
            }]
        )
    ])

