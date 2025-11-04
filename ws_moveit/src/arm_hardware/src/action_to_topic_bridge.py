#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory

class ActionToTopicBridge(Node):
    def __init__(self):
        super().__init__('action_to_topic_bridge')
        
        # Publisher para el topic del controlador
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, 
            '/manipulator_controller/joint_trajectory', 
            10
        )
        
        # Action server para MoveIt
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/manipulator_controller/follow_joint_trajectory',
            self.execute_callback
        )
        
        self.get_logger().info('Action to Topic Bridge started')
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Received trajectory goal')
        
        # Convertir el goal de action a mensaje de topic
        trajectory_msg = goal_handle.request.trajectory
        
        # Publicar en el topic del controlador
        self.trajectory_pub.publish(trajectory_msg)
        
        # Marcar como exitoso
        goal_handle.succeed()
        
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

def main():
    rclpy.init()
    node = ActionToTopicBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
