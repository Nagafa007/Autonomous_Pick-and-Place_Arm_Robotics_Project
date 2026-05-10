import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math

class BrainNode(Node):
    def __init__(self):
        super().__init__('capstone_brain')
        
        # Publishers for the controllers
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.gripper_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        
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
        self.target_x = 0.5 
        self.target_y = 0.0
        self.target_z = 1.0 # Default Hover height
        self.gripper_angle = 0.0 # 0.0 = Open, -1.57 = Closed
        
        self.box_x = 0.0
        self.box_y = 0.0
        self.box_z = 0.0

        # URDF based link lengths (meters)
        # Note: In URDF, joint_2->joint_3 is 0.8m (Z), joint_3->claw is 0.82m (Y)
        self.L1 = 0.80  
        self.L2 = 0.82  
        self.Z_BASE = 0.657 # Height of shoulder joint (joint_2)
        
        self.sequence_start_time = self.get_clock().now()

    def target_callback(self, msg):
        if self.state == "WAITING":
            self.get_logger().info(f"Vision Target Received! Starting Pick Sequence to: X={msg.x:.3f}, Y={msg.y:.3f}, Z={msg.z:.3f}")
            self.box_x = msg.x
            self.box_y = msg.y
            self.box_z = msg.z
            
            # Start the state machine sequence
            self.state = "HOVER"
            self.sequence_start_time = self.get_clock().now()

    def timer_callback(self):
        current_time = self.get_clock().now()
        elapsed = (current_time - self.sequence_start_time).nanoseconds / 1e9

        # --- STATE MACHINE LOGIC ---
        if self.state == "HOVER":
            self.target_x = self.box_x
            self.target_y = self.box_y
            self.target_z = self.box_z + 0.3 # Hover 30cm above box
            self.gripper_angle = 0.0 # Open
            if elapsed > 3.0: 
                self.state = "DROP_DOWN"
                
        elif self.state == "DROP_DOWN":
            self.target_z = self.box_z + 0.05 # Go down to box level (with small offset)
            if elapsed > 6.0:
                self.state = "GRAB"
                
        elif self.state == "GRAB":
            self.gripper_angle = -1.57 # Close gripper
            if elapsed > 8.0:
                self.state = "LIFT"
                
        elif self.state == "LIFT":
            self.target_z = self.box_z + 0.4 # Lift box straight up
            if elapsed > 11.0:
                self.state = "WAITING" 

        # --- KINEMATICS MATH ---
        # Adjust target Z for base height
        z_rel = self.target_z - self.Z_BASE
        
        # 1. Base Rotation (Theta 1)
        theta1 = math.atan2(self.target_y, self.target_x)

        # 2. Distance from base to target (planar distance)
        r = math.sqrt(self.target_x**2 + self.target_y**2)

        # 3. Inverse Kinematics for Shoulder and Elbow
        # We treat it as a 2-link planar arm in the (r, z_rel) plane
        d_sq = r**2 + z_rel**2
        cos_theta3 = (d_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)

        if cos_theta3 > 1.0 or cos_theta3 < -1.0:
            return # Out of reach

        theta3 = math.acos(cos_theta3)
        theta2 = math.atan2(z_rel, r) + math.atan2(self.L2 * math.sin(theta3), self.L1 + self.L2 * math.cos(theta3))

        # URDF Adjustment: 
        # In URDF, at theta2=0, link 1 is UP. In our math, theta2=0 is horizontal.
        # So we subtract PI/2.
        joint_2_cmd = theta2 - (math.pi / 2.0)
        # Joint 3 is relative to Joint 2.
        joint_3_cmd = -theta3 + (math.pi / 2.0)

        self.publish_arm_command(theta1, joint_2_cmd, joint_3_cmd)
        self.publish_gripper_command(self.gripper_angle)

    def publish_arm_command(self, q1, q2, q3):
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3']
        point = JointTrajectoryPoint()
        point.positions = [float(q1), float(q2), float(q3)]
        point.time_from_start = Duration(sec=0, nanosec=100000000) # 0.1s
        msg.points.append(point)
        self.arm_pub.publish(msg)

    def publish_gripper_command(self, angle):
        msg = JointTrajectory()
        msg.joint_names = ['joint_4']
        point = JointTrajectoryPoint()
        point.positions = [float(angle)]
        point.time_from_start = Duration(sec=0, nanosec=100000000) # 0.1s
        msg.points.append(point)
        self.gripper_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BrainNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()