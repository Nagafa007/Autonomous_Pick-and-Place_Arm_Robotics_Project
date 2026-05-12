import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import subprocess, time

# ============================================================
# CONFIGURABLE SETTINGS
# ============================================================
WAIT_TIME      = 10.0   # Seconds the belt pauses (initial wait & between shifts)
BELT_SPEED     = 0.05    # Reduced speed for smaller scale
SHIFT_DISTANCE = 0.4    
# ============================================================

CUBE_SDF = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <pose>0 0 0 0 0 0</pose>
    <allow_auto_disable>false</allow_auto_disable>
    <link name='link'>
      <inertial><mass>0.05</mass>
        <inertia><ixx>0.00001</ixx><ixy>0</ixy><ixz>0</ixz>
                 <iyy>0.00001</iyy><iyz>0</iyz><izz>0.00001</izz></inertia>
      </inertial>
      <collision name='col'>
        <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
      </collision>
      <visual name='vis'>
        <geometry><box><size>0.05 0.05 0.05</size></box></geometry>
        <material>
          <ambient>{ambient}</ambient>
          <diffuse>{diffuse}</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
          <emissive>{emissive}</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

COLOR_PROFILES = [
    ("red",   "1.0 0.0 0.0 1", "0.9 0.0 0.0 1", "0.5 0.0 0.0 1"),
    ("green", "0.0 1.0 0.0 1", "0.0 0.9 0.0 1", "0.0 0.5 0.0 1"),
    ("blue",  "0.0 0.5 1.0 1", "0.0 0.4 1.0 1", "0.0 0.0 0.5 1"),
]

NUM_CUBES = 6
START_X   = 0.5
SPACING   = 0.4

def spawn_cube(cube):
    """Spawn a cube into Gazebo using physical drops."""
    sdf = CUBE_SDF.format(
        name=cube["name"],
        ambient=cube["ambient"], diffuse=cube["diffuse"],
        emissive=cube["emissive"]
    )
    path = f"/tmp/{cube['name']}.sdf"
    with open(path, "w") as f:
        f.write(sdf)
    
    subprocess.Popen(
        ["ros2", "run", "ros_gz_sim", "create", 
         "-file", path, 
         "-name", cube["name"],
         "-x", str(cube["x"]),
         "-y", "0.28",
         "-z", "0.25"], 
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

class ConveyorNode(Node):
    def __init__(self):
        super().__init__("conveyor_node")
        
        self.belt_pub = self.create_publisher(
            Float64, 
            '/model/conveyor_belt_model/link/belt_link/track_cmd_vel', 
            10
        )
        
        self.get_logger().info("Waiting 3s for Gazebo physics to settle...")
        time.sleep(3.0)

        # Spawn cubes 
        for i in range(NUM_CUBES):
            color_name, amb, dif, emi = COLOR_PROFILES[i % len(COLOR_PROFILES)]
            cube = {
                "name": f"{color_name}_cube_{i}",
                "x":    START_X - (i * SPACING),
                "ambient": amb, "diffuse": dif, "emissive": emi,
            }
            spawn_cube(cube)
            self.get_logger().info(f"Dropped {cube['name']} onto the belt at x={cube['x']:.1f}")
            time.sleep(0.5)

        self.state = "WAITING"
        self.state_start_time = self.get_clock().now()
        self.move_duration = SHIFT_DISTANCE / BELT_SPEED
        
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f"Conveyor ready! | Shift Distance = {SHIFT_DISTANCE}m | Wait = {WAIT_TIME}s")

    def tick(self):
        now = self.get_clock().now()
        elapsed = (now.nanoseconds - self.state_start_time.nanoseconds) / 1e9
        
        msg = Float64()

        if self.state == "WAITING":
            msg.data = 0.0
            self.belt_pub.publish(msg)
            
            if elapsed >= WAIT_TIME:
                self.state = "MOVING"
                self.state_start_time = now
                self.get_logger().info(f"▶▶▶ BELT MOVING: Shifting cubes by {SHIFT_DISTANCE} meters...")

        elif self.state == "MOVING":
            msg.data = -BELT_SPEED 
            self.belt_pub.publish(msg)
            
            if elapsed >= self.move_duration:
                self.state = "WAITING"
                self.state_start_time = now
                self.get_logger().info(f"🛑 BELT STOPPED: Waiting {WAIT_TIME}s for arm interaction...")

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()