import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
import time

class SimplePickPlace(Node):
    def __init__(self):
        super().__init__('simple_pick_place')
        self.arm_publisher = self.create_publisher(JointTrajectory, '/fr3_arm_controller/joint_trajectory', 10)
        self.gripper_client = ActionClient(self, GripperCommand, '/fr3_hand_controller/gripper_cmd')
        
        self.joints = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4', 
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]

    def send_arm_trajectory(self, target_angles, duration=2.0):
        msg = JointTrajectory()
        msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        point.positions = target_angles
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        msg.points.append(point)
        self.arm_publisher.publish(msg)
        self.get_logger().info(f'Sent trajectory to {target_angles}')
        time.sleep(duration + 0.5)

    def control_gripper(self, position, max_effort=10.0):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort
        
        self.get_logger().info(f'Sending gripper goal: {position}')
        self.gripper_client.wait_for_server()
        return self.gripper_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimplePickPlace()

    # ready pose
    ready = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    # pick pose (approximated for demo)
    pick_hover = [0.0, -0.4, 0.0, -2.0, 0.0, 1.6, 0.785]
    pick_down = [0.0, -0.2, 0.0, -1.8, 0.0, 1.6, 0.785]
    # place pose
    place_hover = [0.5, -0.4, 0.0, -2.0, 0.0, 1.6, 0.785]
    place_down = [0.5, -0.2, 0.0, -1.8, 0.0, 1.6, 0.785]

    node.get_logger().info('Starting Pick and Place Demo')
    
    # 1. Open Gripper
    node.control_gripper(0.04) # 4cm open
    time.sleep(1.0)

    # 2. Move to Ready
    node.send_arm_trajectory(ready)

    # 3. Hover over piece
    node.send_arm_trajectory(pick_hover)

    # 4. Lower to piece
    node.send_arm_trajectory(pick_down)

    # 5. Close Gripper
    node.control_gripper(0.01) # 1cm closed (holding piece)
    time.sleep(1.0)

    # 6. Lift up
    node.send_arm_trajectory(pick_hover)

    # 7. Move to place hover
    node.send_arm_trajectory(place_hover)

    # 8. Lower to place
    node.send_arm_trajectory(place_down)

    # 9. Open Gripper
    node.control_gripper(0.04)
    time.sleep(1.0)

    # 10. Lift up
    node.send_arm_trajectory(place_hover)

    # 11. Return to ready
    node.send_arm_trajectory(ready)

    node.get_logger().info('Demo Complete')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
