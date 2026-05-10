import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point  
import math
import time

class BrainNode(Node):
    def __init__(self):
        super().__init__('capstone_brain')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        
        # Faster timer for smoother state machine movement
        self.timer = self.create_timer(0.1, self.timer_callback) 

        # SUBSCRIBER TO VISION NODE
        self.subscription = self.create_subscription(
            Point,
            '/target_coordinates',
            self.target_callback,
            10
        )

        # --- State Machine Variables ---
        self.state = "WAITING"
        self.target_x = 0.15 
        self.target_y = 0.0
        self.target_z = 0.25 # Default Hover height
        self.gripper_angle = 0.0 # 0.0 = Open, 1.0 = Closed
        
        self.box_x = 0.0
        self.box_y = 0.0
        self.box_z = 0.0

        # EEZYbotARM approximate link lengths (meters)
        self.L1 = 0.135  # Shoulder to Elbow
        self.L2 = 0.147  # Elbow to Wrist
        
        self.sequence_start_time = 0.0

    # Triggers instantly whenever vision_node.py sends a new location
    def target_callback(self, msg):
        if self.state == "WAITING":
            self.get_logger().info(f"Vision Target Received! Starting Pick Sequence to: X={msg.x:.3f}, Y={msg.y:.3f}, Z={msg.z:.3f}")
            self.box_x = msg.x
            self.box_y = msg.y
            self.box_z = msg.z
            
            # Start the state machine sequence
            self.state = "HOVER"
            self.sequence_start_time = time.time()

    def timer_callback(self):
        current_time = time.time()
        elapsed = current_time - self.sequence_start_time

        # --- STATE MACHINE LOGIC ---
        if self.state == "HOVER":
            self.target_x = self.box_x
            self.target_y = self.box_y
            self.target_z = self.box_z + 0.15 # Hover 15cm above box
            self.gripper_angle = 0.0 # Open
            if elapsed > 2.0: # Wait 2 seconds for arm to arrive physically
                self.state = "DROP_DOWN"
                
        elif self.state == "DROP_DOWN":
            self.target_z = self.box_z # Go straight down to box level
            if elapsed > 4.0:
                self.state = "GRAB"
                
        elif self.state == "GRAB":
            self.gripper_angle = 1.0 # Close gripper (Adjust '1.0' if your URDF uses a different max angle)
            if elapsed > 5.0:
                self.state = "LIFT"
                
        elif self.state == "LIFT":
            self.target_z = self.box_z + 0.15 # Lift box straight up
            if elapsed > 7.0:
                # Add logic here to move to a drop-off bin later!
                # For now, it rests and waits for the next 10-second vision trigger
                self.state = "WAITING" 

        # --- KINEMATICS MATH ---
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

            # Prevent math errors if coordinate is too far away
            if cos_theta3 > 1.0 or cos_theta3 < -1.0:
                return # Silently skip moving if out of reach to avoid crashing

            theta3 = math.acos(cos_theta3)
            theta2 = math.atan2(self.target_z, r) + math.atan2(self.L2 * math.sin(theta3), self.L1 + self.L2 * math.cos(theta3))

            # Publish the calculated angles to the robot, now including gripper!
            msg.position = [theta1, theta2, -theta3, self.gripper_angle] 
            self.publisher_.publish(msg)

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