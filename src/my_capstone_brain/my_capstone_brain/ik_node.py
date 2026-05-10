import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point  
import math

class BrainNode(Node):
    def __init__(self):
        super().__init__('capstone_brain')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.5, self.timer_callback) 

        # --- NEW: SUBSCRIBER TO VISION NODE ---
        self.subscription = self.create_subscription(
            Point,
            '/target_coordinates',
            self.target_callback,
            10
        )

        # --- YOUR TARGET COORDINATES (in meters) ---
        # These are now just default "resting" positions when it first boots up.
        # They will be overwritten as soon as the camera sees a box!
        self.target_x = 0.15 
        self.target_y = 0.0
        self.target_z = 0.15

        # EEZYbotARM approximate link lengths (meters)
        self.L1 = 0.135  # Shoulder to Elbow
        self.L2 = 0.147  # Elbow to Wrist

    # --- NEW: CALLBACK FUNCTION ---
    # This triggers instantly whenever vision_node.py sends a new location
    def target_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_z = msg.z
        self.get_logger().info(f"Vision Target Received! Moving to: X={self.target_x:.3f}, Y={self.target_y:.3f}, Z={self.target_z:.3f}")

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
                # Silently skip moving if out of reach to avoid crashing
                return

            theta3 = math.acos(cos_theta3)
            theta2 = math.atan2(self.target_z, r) + math.atan2(self.L2 * math.sin(theta3), self.L1 + self.L2 * math.cos(theta3))

            # Publish the calculated angles to the robot
            # (Note: Every 3D model has specific 'zero' resting positions. 
            # We may need to tweak the +/- signs later depending on how Antonio built his URDF)
            msg.position = [theta1, theta2, -theta3, 0.0] 
            self.publisher_.publish(msg)

            # I commented this out so it doesn't spam your terminal 2 times a second! 
            # It will now only print when it receives a new vision target above.
            # self.get_logger().info(
            #     f"Target: ({self.target_x}, {self.target_y}, {self.target_z}) | "
            #     f"Angles: [Base: {math.degrees(theta1):.1f}°, Shoulder: {math.degrees(theta2):.1f}°, Elbow: {math.degrees(-theta3):.1f}°]"
            # )

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