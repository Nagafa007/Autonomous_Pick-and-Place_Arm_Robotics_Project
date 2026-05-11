import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Point
import time

class VisionBrain(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # 1. Subscribe to the camera feed (Bridged from Gazebo)
        self.subscription = self.create_subscription(
            Image, 
            '/camera/image', 
            self.image_callback, 
            10
        )
        
        # 2. Setup the ROS-to-OpenCV bridge
        self.bridge = CvBridge()
        self.get_logger().info("Vision Node started! Looking for red boxes...")

        # 3. Setup the Publisher to send coordinates to the IK node
        self.target_pub = self.create_publisher(Point, '/target_coordinates', 10)
        
        # 4. Debug Image Publisher
        self.debug_pub = self.create_publisher(Image, '/camera/debug_image', 10)
        
        # Anti-Spam timeout tracker
        self.last_target_time = 0.0

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Convert the image from BGR to HSV color space
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the color range for RED in HSV
        lower_red1 = np.array([0, 150, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 150, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        full_red_mask = mask1 + mask2

        # Find the outlines (contours) of the red objects
        contours, _ = cv2.findContours(full_red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            
            if cv2.contourArea(largest_contour) > 500: 
                M = cv2.moments(largest_contour)
                if M['m00'] != 0:
                    pixel_x = int(M['m10'] / M['m00'])
                    pixel_y = int(M['m01'] / M['m00'])
                    
                    # Drawing for visualization
                    cv2.drawMarker(cv_image, (pixel_x, pixel_y), (0, 255, 0), cv2.MARKER_CROSS, 20, 2)
                    cv2.putText(cv_image, "RED BOX", (pixel_x+10, pixel_y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    
                    # --- STEP 4: Pixels to Real-World Coordinates ---
                    # Camera is at (0, 1.2, 2.0) looking down.
                    # Image size 640x480.
                    CAMERA_GAZEBO_X = 0.0
                    CAMERA_GAZEBO_Y = 1.2
                    CAMERA_PIXEL_CX = 160 
                    CAMERA_PIXEL_CY = 120 
                    
                    # Based on FoV 1.047 (60 deg) at 2m height:
                    # width_m = 2 * tan(30 deg) * 2 = 2.3m
                    # METERS_PER_PIXEL = 2.3 / 320 = 0.0072
                    METERS_PER_PIXEL = 0.0072 

                    # How many pixels is the box away from the center?
                    pixel_dx = pixel_x - CAMERA_PIXEL_CX
                    pixel_dy = pixel_y - CAMERA_PIXEL_CY

                    # Convert pixel offset to meter offset
                    meters_dx = pixel_dx * METERS_PER_PIXEL
                    meters_dy = pixel_dy * METERS_PER_PIXEL

                    # Map back to Gazebo World
                    # Camera orientation 0 1.5708 -1.5708
                    # This means Image X is World Y, Image Y is World X
                    world_x = CAMERA_GAZEBO_X + meters_dy 
                    world_y = CAMERA_GAZEBO_Y - meters_dx
                    world_z = 0.60 # The height of the box
                    
                    # --- STEP 5: Send Coordinates to IK Node ---
                    current_time = self.get_clock().now().seconds_nanoseconds()[0]
                    
                    if current_time - self.last_target_time > 15.0: # 15s delay between picks
                        self.get_logger().info(f"BOX TARGET DETECTED: X={world_x:.3f}, Y={world_y:.3f}, Z={world_z:.3f}")
                        
                        target_msg = Point()
                        target_msg.x = world_x
                        target_msg.y = world_y
                        target_msg.z = world_z
                        
                        self.target_pub.publish(target_msg)
                        self.last_target_time = float(current_time)

        # Publish the debug image back to ROS
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish debug image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionBrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()