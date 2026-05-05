#!/usr/bin/env python3
"""Publishes finger joint states at open position so MoveIt has a complete robot state."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, DurabilityPolicy

class FingerStatePublisher(Node):
    def __init__(self):
        super().__init__('finger_state_publisher')
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.1, self.publish)

    def publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'white_fr3_finger_joint1',
            'white_fr3_finger_joint2',
            'black_fr3_finger_joint1',
            'black_fr3_finger_joint2',
        ]
        msg.position = [0.04, 0.04, 0.04, 0.04]
        msg.velocity = [0.0, 0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0, 0.0]
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = FingerStatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
