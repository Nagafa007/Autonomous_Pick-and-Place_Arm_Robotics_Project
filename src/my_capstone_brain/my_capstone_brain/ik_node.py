import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class BrainNode(Node):
    def __init__(self):
        super().__init__('capstone_brain')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.5, self.timer_callback) 

        # --- YOUR TARGET COORDINATES (in meters) ---
        # Right now the target is 15cm forward, 10cm left, and 15cm high.
        self.target_x = 0.15 
        self.target_y = 0.10
        self.target_z = 0.15

        # EEZYbotARM approximate link lengths (meters)
        self.L1 = 0.135  # Shoulder to Elbow
        self.L2 = 0.147  # Elbow to Wrist

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4'] 

        try:
            # 1. Base Rotation (Theta 1)
            theta1 = math.atan2(self.target_y, self.target_x)

            # 2. Distance from base to target (planar distance)
            r = math.sqrt(self.target_x**2 + self.target_y**2)

            # 3. Inverse Kinematics for Shoulder and Elbow using Law of Cosines
            d_sq = r**2 + self.target_z**2
            cos_theta3 = (d_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)

            # Prevent math errors if you type a coordinate that is too far away
            if cos_theta3 > 1.0 or cos_theta3 < -1.0:
                self.get_logger().warning("Target is out of reach!")
                return

            theta3 = math.acos(cos_theta3)
            theta2 = math.atan2(self.target_z, r) + math.atan2(self.L2 * math.sin(theta3), self.L1 + self.L2 * math.cos(theta3))

            # Publish the calculated angles to the robot
            # (Note: Every 3D model has specific 'zero' resting positions. 
            # We may need to tweak the +/- signs later depending on how Antonio built his URDF)
            msg.position = [theta1, theta2, -theta3, 0.0] 
            self.publisher_.publish(msg)

            # Print the math to the terminal so you can see it working!
            self.get_logger().info(
                f"Target: ({self.target_x}, {self.target_y}, {self.target_z}) | "
                f"Angles: [Base: {math.degrees(theta1):.1f}°, Shoulder: {math.degrees(theta2):.1f}°, Elbow: {math.degrees(-theta3):.1f}°]"
            )

        except Exception as e:
            self.get_logger().error(f"IK Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()