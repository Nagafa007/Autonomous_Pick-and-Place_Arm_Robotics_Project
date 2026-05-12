import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class MycobotVisionBrain(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # ROS 2 Pubs/Subs
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        
        self.bridge = CvBridge()
        
        # State Machine
        # SEARCHING -> MOVE_TO_BELT -> LEAN_TO_PICK -> GRASP -> STRAIGHTEN_PICK -> MOVE_TO_DROP_ZONE -> LEAN_TO_DROP -> RELEASE -> STRAIGHTEN_DROP -> RESET
        self.state = "SEARCHING"
        self.last_state_time = time.time()
        
        # ============================================================
        # TUNED PARAMETERS (Adjust these as needed)
        # ============================================================
        # Joint Names Mapping:
        # 1: link1_to_link2 (Base rotation J1)
        # 2: link2_to_link3 (Lean J2)
        # 3: link3_to_link4 (Elbow J3)
        # 4: link4_to_link5 (Wrist pitch J4)
        # 5: link5_to_link6 (Wrist roll J5 - "Ready" Orientation)
        # 6: link6_to_link6_flange (J6)
        
        # J1 Positions (Rotation)
        self.J1_BELT = 1.57   # 90 degrees to face the belt
        self.J1_DROP = -1.57  # -90 degrees to face the drop zone
        self.J1_IDLE = 0.0    # Face forward
        
        # J2 Positions (Lean)
        self.J2_UP   = 0.0    # Arm standing still (vertical)
        self.J2_DOWN = 1.1    # Arm leaning on top of belt/cube (Adjust this!)
        
        # J5 Orientation (Ready)
        self.J5_READY = 0.0   # Adjust this if the gripper needs a specific roll to be "ready"
        
        # Constant joints (keep simple for now)
        self.J3_FIXED = 0.0
        self.J4_FIXED = 0.0
        self.J6_FIXED = 0.0

        # Pose Definitions [J1, J2, J3, J4, J5, J6]
        self.POSE_IDLE      = [self.J1_IDLE, self.J2_UP,   self.J3_FIXED, self.J4_FIXED, self.J5_READY, self.J6_FIXED]
        self.POSE_PRE_PICK  = [self.J1_BELT, self.J2_UP,   self.J3_FIXED, self.J4_FIXED, self.J5_READY, self.J6_FIXED]
        self.POSE_PICK       = [self.J1_BELT, self.J2_DOWN, self.J3_FIXED, self.J4_FIXED, self.J5_READY, self.J6_FIXED]
        
        self.POSE_PRE_DROP  = [self.J1_DROP, self.J2_UP,   self.J3_FIXED, self.J4_FIXED, self.J5_READY, self.J6_FIXED]
        self.POSE_DROP       = [self.J1_DROP, self.J2_DOWN, self.J3_FIXED, self.J4_FIXED, self.J5_READY, self.J6_FIXED]
        
        # Gripper
        self.grip_pub = self.create_publisher(JointTrajectory, '/gripper_action_controller/joint_trajectory', 10)
        self.GRIP_OPEN   = [0.0]
        self.GRIP_CLOSED = [-0.7]
        # ============================================================

        self.get_logger().info("🤖 Mycobot Vision Brain Online. Searching for Red Cubes...")

    def move_arm(self, positions, duration_sec=2):
        msg = JointTrajectory()
        msg.joint_names = [
            'link1_to_link2', 'link2_to_link3', 'link3_to_link4', 
            'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange'
        ]
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=duration_sec, nanosec=0)
        msg.points.append(point)
        self.arm_pub.publish(msg)

    def move_gripper(self, positions, duration_sec=1):
        msg = JointTrajectory()
        msg.joint_names = ['gripper_controller']
        point = JointTrajectoryPoint()
        point.positions = [float(p) for p in positions]
        point.time_from_start = Duration(sec=duration_sec, nanosec=0)
        msg.points.append(point)
        self.grip_pub.publish(msg)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {e}")
            return

        # Simple Red Detection
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, np.array([0, 100, 50]), np.array([10, 255, 255]))
        mask2 = cv2.inRange(hsv, np.array([170, 100, 50]), np.array([180, 255, 255]))
        mask = cv2.bitwise_or(mask1, mask2)
        
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        red_found = False
        if contours:
            largest = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest) > 500: # Threshold for detection
                red_found = True
        
        # State Machine Logic
        now = time.time()
        elapsed = now - self.last_state_time

        if self.state == "SEARCHING":
            if red_found:
                self.get_logger().info("🎯 Red Cube Detected! Rotating J1 to Belt...")
                self.state = "MOVE_TO_BELT"
                self.last_state_time = now
                self.move_arm(self.POSE_PRE_PICK)
                self.move_gripper(self.GRIP_OPEN)

        elif self.state == "MOVE_TO_BELT" and elapsed > 2.0:
            self.get_logger().info("⬇️ Leaning J2 to Cube...")
            self.state = "LEAN_TO_PICK"
            self.last_state_time = now
            self.move_arm(self.POSE_PICK)

        elif self.state == "LEAN_TO_PICK" and elapsed > 2.0:
            self.get_logger().info("✊ Grasping...")
            self.state = "GRASP"
            self.last_state_time = now
            self.move_gripper(self.GRIP_CLOSED)

        elif self.state == "GRASP" and elapsed > 1.5:
            self.get_logger().info("⬆️ Inverting J2 (Straighten Up)...")
            self.state = "STRAIGHTEN_PICK"
            self.last_state_time = now
            self.move_arm(self.POSE_PRE_PICK)

        elif self.state == "STRAIGHTEN_PICK" and elapsed > 2.0:
            self.get_logger().info("↪️ Rotating J1 to Drop Zone...")
            self.state = "MOVE_TO_DROP_ZONE"
            self.last_state_time = now
            self.move_arm(self.POSE_PRE_DROP)

        elif self.state == "MOVE_TO_DROP_ZONE" and elapsed > 2.5:
            self.get_logger().info("⬇️ Leaning J2 to Drop Position...")
            self.state = "LEAN_TO_DROP"
            self.last_state_time = now
            self.move_arm(self.POSE_DROP)

        elif self.state == "LEAN_TO_DROP" and elapsed > 2.0:
            self.get_logger().info("🖐 Releasing Cube...")
            self.state = "RELEASE"
            self.last_state_time = now
            self.move_gripper(self.GRIP_OPEN)

        elif self.state == "RELEASE" and elapsed > 1.5:
            self.get_logger().info("⬆️ Inverting J2 (Straighten Up)...")
            self.state = "STRAIGHTEN_DROP"
            self.last_state_time = now
            self.move_arm(self.POSE_PRE_DROP)

        elif self.state == "STRAIGHTEN_DROP" and elapsed > 2.0:
            self.get_logger().info("🔄 Returning to Initial Position...")
            self.state = "RESET"
            self.last_state_time = now
            self.move_arm(self.POSE_IDLE)

        elif self.state == "RESET" and elapsed > 2.5:
            self.get_logger().info("🔍 Searching for next red cube...")
            self.state = "SEARCHING"
            self.last_state_time = now

def main(args=None):
    rclpy.init(args=args)
    node = MycobotVisionBrain()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
