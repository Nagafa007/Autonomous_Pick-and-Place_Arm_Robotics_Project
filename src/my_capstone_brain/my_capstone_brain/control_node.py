import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import sys
import select
import termios
import tty
import threading

# ============================================================
# CONFIGURABLE KEY BINDINGS & SETTINGS
# ============================================================
KEY_BINDINGS = {
    'x': 'increase_x', 'X': 'decrease_x',  # Forward / Backward
    'y': 'increase_y', 'Y': 'decrease_y',  # Left / Right
    'z': 'increase_z', 'Z': 'decrease_z',  # Up / Down
    'q': 'open_gripper',                   # Open Claw
    'e': 'close_gripper',                  # Close Claw
}

STEP_SIZE = 0.01  # How many meters the arm moves per key press (0.01 = 1 cm)

GRIP_OPEN = 0.0
GRIP_CLOSED = -0.5
# ============================================================

msg = """
=============================================
🤖 CAPSTONE KEYBOARD TELEOPERATION ONLINE 🤖
=============================================
Control the End-Effector in Cartesian Space:
---------------------------------------------
  x : Move Forward (+X)   |  X (Shift+x): Move Backward (-X)
  y : Move Left    (+Y)   |  Y (Shift+y): Move Right    (-Y)
  z : Move Up      (+Z)   |  Z (Shift+z): Move Down     (-Z)
  
  q : Open Gripper        |  e : Close Gripper

CTRL-C to quit
=============================================
"""

def get_key(settings):
    """Reads a single keypress from the terminal without requiring 'Enter'."""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        self.arm_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.grip_pub = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        
        # EEZYbotARM link lengths
        self.L1 = 0.135
        self.L2 = 0.147
        
        # Initial starting position
        self.target_x = 0.15
        self.target_y = 0.0
        self.target_z = 0.15
        
        # Move to initial position on boot
        self.move_arm(self.target_x, self.target_y, self.target_z)
        self.move_gripper(open_gripper=True)

    def calculate_ik(self, target_x, target_y, target_z):
        """Bulletproof Inverse Kinematics Math"""
        theta1 = math.atan2(target_y, target_x)
        r = math.sqrt(target_x**2 + target_y**2)
        
        # BULLETPROOF CLAMP: Prevent unreachable coordinates!
        max_reach = self.L1 + self.L2 - 0.002
        d_sq = r**2 + target_z**2
        
        if d_sq > max_reach**2:
            target_z = min(target_z, max_reach - 0.01)
            r = math.sqrt(max_reach**2 - target_z**2)
            d_sq = max_reach**2
            
        cos_theta3 = (d_sq - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        cos_theta3 = max(-1.0, min(1.0, cos_theta3))

        theta3 = math.acos(cos_theta3)
        theta2 = math.atan2(target_z, r) + math.atan2(self.L2 * math.sin(theta3), self.L1 + self.L2 * math.cos(theta3))

        return [theta1, theta2, -theta3]

    def move_arm(self, target_x, target_y, target_z):
        """Publishes the exact angle targets to Gazebo using IK"""
        angles = self.calculate_ik(target_x, target_y, target_z)
        msg = JointTrajectory()
        # Leaving timestamp empty fixes the Gazebo Time-Traveler bug!
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3']
        
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = Duration(sec=0, nanosec=500000000) # Fast 0.5s response
        msg.points.append(point)
        self.arm_pub.publish(msg)

    def move_gripper(self, open_gripper=True):
        """Opens or closes the claws"""
        msg = JointTrajectory()
        msg.joint_names = ['joint_4']
        
        point = JointTrajectoryPoint()
        point.positions = [GRIP_OPEN] if open_gripper else [GRIP_CLOSED] 
        point.time_from_start = Duration(sec=0, nanosec=500000000)
        msg.points.append(point)
        self.grip_pub.publish(msg)

    def process_key(self, key):
        """Updates XYZ based on the key pressed."""
        if key not in KEY_BINDINGS:
            return

        action = KEY_BINDINGS[key]

        if action == 'increase_x':
            self.target_x += STEP_SIZE
        elif action == 'decrease_x':
            self.target_x -= STEP_SIZE
        elif action == 'increase_y':
            self.target_y += STEP_SIZE
        elif action == 'decrease_y':
            self.target_y -= STEP_SIZE
        elif action == 'increase_z':
            self.target_z += STEP_SIZE
        elif action == 'decrease_z':
            self.target_z -= STEP_SIZE
        elif action == 'open_gripper':
            self.move_gripper(open_gripper=True)
            print("🖐  Gripper: OPEN")
            return
        elif action == 'close_gripper':
            self.move_gripper(open_gripper=False)
            print("✊ Gripper: CLOSED")
            return

        # Constrain X so the arm doesn't try to fold inside itself
        self.target_x = max(0.05, self.target_x)

        # Move the arm
        self.move_arm(self.target_x, self.target_y, self.target_z)
        
        # Clear the current line and print the new coordinates dynamically
        sys.stdout.write(f"\r🎯 Target [X: {self.target_x:.2f} | Y: {self.target_y:.2f} | Z: {self.target_z:.2f}]   ")
        sys.stdout.flush()


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init(args=args)
    node = KeyboardTeleopNode()
    
    # Run the ROS 2 spin loop in a background thread so the terminal can listen to keys
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print(msg)
    
    try:
        while True:
            key = get_key(settings)
            if key == '\x03':  # CTRL-C
                break
            node.process_key(key)
            
    except Exception as e:
        print(f"\n[ERROR] {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()