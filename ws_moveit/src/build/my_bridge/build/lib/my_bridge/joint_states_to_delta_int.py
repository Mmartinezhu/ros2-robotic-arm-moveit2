#!/usr/bin/env python3
import math
from typing import Dict, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray
from std_srvs.srv import Trigger

def rad2deg(x: float) -> float:
    return x * 180.0 / math.pi

class JointStatesToDeltaInt(Node):
    def __init__(self):
        super().__init__('joint_states_to_delta_int')
        self.declare_parameter('joint_names', ['rev_1','rev_2','rev_3'])
        self.declare_parameter('output_topic', '/motor_angles')
        self.declare_parameter('deadband_deg', 0.3)
        self.declare_parameter('max_step_deg', 8.0)
        self.declare_parameter('pad_size', 3)

        self.joint_names = list(self.get_parameter('joint_names').get_parameter_value().string_array_value)
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.deadband_deg = float(self.get_parameter('deadband_deg').get_parameter_value().double_value)
        self.max_step_deg = float(self.get_parameter('max_step_deg').get_parameter_value().double_value)
        self.pad_size = int(self.get_parameter('pad_size').get_parameter_value().integer_value)

        self.pub = self.create_publisher(Int32MultiArray, self.output_topic, 10)
        self.sub = self.create_subscription(JointState, '/joint_states', self.js_cb, 50)
        self.srv = self.create_service(Trigger, 'reset_joint_delta_ref', self.srv_reset)

        self.last_pos: Dict[str, float] = {}
        self.seen_once = False

    def srv_reset(self, req, res):
        self.last_pos.clear()
        self.seen_once = False
        res.success = True
        res.message = 'Referencia reiniciada.'
        self.get_logger().info(res.message)
        return res

    def js_cb(self, msg: JointState):
        pos = {name: msg.position[i] for i, name in enumerate(msg.name) if i < len(msg.position)}

        if not self.seen_once:
            if all(j in pos for j in self.joint_names):
                for j in self.joint_names:
                    self.last_pos[j] = pos[j]
                self.seen_once = True
                self.get_logger().info('Primera referencia tomada; publicando deltas a partir de ahora.')
            return

        deltas: List[int] = []
        any_move = False

        for j in self.joint_names:
            if j not in pos:
                deltas.append(0)
                continue
            curr = pos[j]
            last = self.last_pos.get(j, curr)
            d_deg = rad2deg(curr - last)

            if abs(d_deg) < self.deadband_deg:
                d_deg = 0.0
            else:
                any_move = True

            if d_deg > self.max_step_deg:  d_deg = self.max_step_deg
            if d_deg < -self.max_step_deg: d_deg = -self.max_step_deg

            deltas.append(int(round(d_deg)))
            self.last_pos[j] = curr

        if not any_move:
            return

        if len(deltas) < self.pad_size:
            deltas += [0] * (self.pad_size - len(deltas))
        elif len(deltas) > self.pad_size:
            deltas = deltas[:self.pad_size]

        out = Int32MultiArray()
        out.data = deltas
        self.pub.publish(out)

def main():
    rclpy.init()
    node = JointStatesToDeltaInt()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
