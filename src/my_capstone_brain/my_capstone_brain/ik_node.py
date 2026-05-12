import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class MycobotIKNode(Node):
    def __init__(self):
        super().__init__('mycobot_ik_node')
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.timer = self.create_timer(0.5, self.timer_callback) 

        # --- TARGET COORDINATES (in meters) ---
        self.target_x = 0.15 
        self.target_y = 0.10
        self.target_z = 0.15

        # Mycobot 280 approximate link lengths (meters)
        self.L1 = 0.13156 # link1 to link2 (height)
        self.L2 = 0.1104  # link3 to link4
        self.L3 = 0.096   # link4 to link5

        self.joint_names = [
            'link1_to_link2', 'link2_to_link3', 'link3_to_link4',
            'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange'
        ]

    def timer_callback(self):
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        
        try:
            # 1. Base Rotation (J1)
            theta1 = math.atan2(self.target_y, self.target_x)

            # 2. Planar distance
            r = math.sqrt(self.target_x**2 + self.target_y**2)
            
            # Simple 3-joint IK mapping for demo purposes
            adj_z = self.target_z - self.L1
            
            d_sq = r**2 + adj_z**2
            cos_theta3 = (d_sq - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
            cos_theta3 = max(-1.0, min(1.0, cos_theta3))
            
            theta3 = math.acos(cos_theta3)
            theta2 = math.atan2(adj_z, r) + math.atan2(self.L3 * math.sin(theta3), self.L2 + self.L3 * math.cos(theta3))

            j2_val = (math.pi/2) - theta2
            j3_val = -theta3
            
            point.positions = [theta1, j2_val, j3_val, 0.0, 0.0, 0.0]
            point.time_from_start = Duration(sec=0, nanosec=500000000)
            msg.points.append(point)
            
            self.publisher_.publish(msg)

            self.get_logger().info(
                f"Target: ({self.target_x}, {self.target_y}, {self.target_z}) | "
                f"Published Mycobot Trajectory"
            )

        except Exception as e:
            self.get_logger().error(f"IK Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MycobotIKNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()