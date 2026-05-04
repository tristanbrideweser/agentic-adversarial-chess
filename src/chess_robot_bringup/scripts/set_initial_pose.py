import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

WHITE_JOINTS = [
    "white_fr3_joint1", "white_fr3_joint2", "white_fr3_joint3",
    "white_fr3_joint4", "white_fr3_joint5", "white_fr3_joint6", "white_fr3_joint7"
]
BLACK_JOINTS = [
    "black_fr3_joint1", "black_fr3_joint2", "black_fr3_joint3",
    "black_fr3_joint4", "black_fr3_joint5", "black_fr3_joint6", "black_fr3_joint7"
]
READY_POSE = [0.0, -1.0, 0.0, -2.5, 0.0, 2.1, 0.0]

def main():
    rclpy.init()
    node = Node("set_initial_pose")

    white_pub = node.create_publisher(
        JointTrajectory, "/white_arm_controller/joint_trajectory", 10
    )
    black_pub = node.create_publisher(
        JointTrajectory, "/black_arm_controller/joint_trajectory", 10
    )

    # Wait for subscribers
    import time
    time.sleep(2.0)

    for pub, joints in [(white_pub, WHITE_JOINTS), (black_pub, BLACK_JOINTS)]:
        traj = JointTrajectory()
        traj.joint_names = joints
        pt = JointTrajectoryPoint()
        pt.positions = READY_POSE
        pt.time_from_start = Duration(sec=3, nanosec=0)
        traj.points = [pt]
        pub.publish(traj)
        node.get_logger().info(f"Sent ready pose to {joints[0][:5]} arm")

    time.sleep(1.0)
    rclpy.shutdown()

if __name__ == "__main__":
    main()