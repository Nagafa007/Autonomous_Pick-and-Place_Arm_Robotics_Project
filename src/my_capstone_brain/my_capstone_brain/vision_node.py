import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Point

class VisionBrain(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # 1. Subscribe to the camera feed
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
        # IMPORTANT: Check your ik_node.py to see exactly what topic it listens to! 
        # We will assume it's called '/target_coordinates' for now.
        self.target_pub = self.create_publisher(Point, '/target_coordinates', 10)
        
        # Anti-Spam toggle
        self.box_targeted = False

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Convert the image from BGR to HSV color space (better for color detection)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the color range for RED in HSV
        # Note: Red wraps around the HSV spectrum, so we often combine two masks
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        full_red_mask = mask1 + mask2

        # Find the outlines (contours) of the red objects
        contours, _ = cv2.findContours(full_red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Grab the largest red object (ignoring tiny red specks/noise)
            largest_contour = max(contours, key=cv2.contourArea)
            
            if cv2.contourArea(largest_contour) > 500: # 500 pixels minimum size
                # Calculate the center of the contour using Image Moments
                M = cv2.moments(largest_contour)
                if M['m00'] != 0:
                    pixel_x = int(M['m10'] / M['m00'])
                    pixel_y = int(M['m01'] / M['m00'])
                    
                    self.get_logger().info(f"Red box located at Pixel (X: {pixel_x}, Y: {pixel_y})")
                    
                    # Optional: Draw a green circle on the center to verify it works visually
                    cv2.circle(cv_image, (pixel_x, pixel_y), 5, (0, 255, 0), -1)
                    cv2.imshow("Robot Camera View", cv_image)
                    cv2.waitKey(1)
                    
                    # --- STEP 4: Pixels to Real-World Coordinates ---
                    # 1. Known Camera Data
                    CAMERA_GAZEBO_X = 0.0
                    CAMERA_GAZEBO_Y = 1.2
                    CAMERA_PIXEL_CX = 320 # Center X of a 640x480 image
                    CAMERA_PIXEL_CY = 240 # Center Y of a 640x480 image
                    METERS_PER_PIXEL = 0.0027 

                    # 2. How many pixels is the box away from the center?
                    pixel_dx = pixel_x - CAMERA_PIXEL_CX
                    pixel_dy = pixel_y - CAMERA_PIXEL_CY

                    # 3. Convert pixel offset to meter offset
                    meters_dx = pixel_dx * METERS_PER_PIXEL
                    meters_dy = pixel_dy * METERS_PER_PIXEL

                    # 4. Map back to Gazebo World (Applying camera rotation)
                    # Note: Because your camera has a Yaw of -1.5708 (-90 degrees), 
                    # the image axes are rotated relative to the world axes.
                    world_x = CAMERA_GAZEBO_X - meters_dy 
                    world_y = CAMERA_GAZEBO_Y - meters_dx
                    world_z = 0.55 # The height of the box resting on the belt
                    
                    self.get_logger().info(f"BOX TARGET: X={world_x:.3f}, Y={world_y:.3f}, Z={world_z:.3f}")

                    # --- STEP 5: Send Coordinates to IK Node ---
                    if not self.box_targeted:
                        target_msg = Point()
                        target_msg.x = world_x
                        target_msg.y = world_y
                        target_msg.z = world_z
                        
                        self.target_pub.publish(target_msg)
                        self.get_logger().info("TARGET SENT TO ARM!")
                        
                        # Set to true so we don't spam the arm with 30 messages a second
                        self.box_targeted = True 
                    # ------------------------------------------------

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