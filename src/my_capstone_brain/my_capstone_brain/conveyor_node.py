#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import subprocess, time

CUBE_SDF = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <link name='link'>
      <inertial><mass>0.08</mass>
        <inertia><ixx>0.00005</ixx><ixy>0</ixy><ixz>0</ixz>
                 <iyy>0.00005</iyy><iyz>0</iyz><izz>0.00005</izz></inertia>
      </inertial>
      <collision name='col'>
        <geometry><box><size>0.06 0.06 0.06</size></box></geometry>
        <surface><friction><ode><mu>0.4</mu><mu2>0.4</mu2></ode></friction></surface>
      </collision>
      <visual name='vis'>
        <geometry><box><size>0.06 0.06 0.06</size></box></geometry>
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

# x = position along belt, y = 1.2 (belt location), z = 0.115 (on top of belt)
CUBES = [
    ("red_cube",    0.20, "1.0 0.0 0.0 1", "0.9 0.0 0.0 1", "0.5 0.0 0.0 1"),
    ("green_cube",  0.00, "0.0 1.0 0.0 1", "0.0 0.9 0.0 1", "0.0 0.5 0.0 1"),
    ("blue_cube",  -0.20, "0.0 0.5 1.0 1", "0.0 0.4 1.0 1", "0.0 0.0 0.5 1"),
]

def spawn_cube(name, x, ambient, diffuse, emissive):
    sdf = CUBE_SDF.format(name=name, ambient=ambient,
                           diffuse=diffuse, emissive=emissive)
    path = f"/tmp/{name}.sdf"
    with open(path, "w") as f:
        f.write(sdf)
    # Pass position explicitly via -x -y -z flags
    subprocess.Popen(
        ["ros2", "run", "ros_gz_sim", "create",
         "-file", path,
         "-name", name,
         "-x", str(x),
         "-y", "1.2",
         "-z", "0.115"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    print(f"[Conveyor] Spawned {name} at x={x}, y=1.2, z=0.115")

def push_cube(name):
    req = (f'entity: {{name: "{name}/link", type: LINK}} '
           f'wrench: {{force: {{x: -0.3, y: 0.0, z: 0.0}} '
           f'torque: {{x: 0.0, y: 0.0, z: 0.0}}}} '
           f'duration: {{sec: 1, nsec: 0}}')
    subprocess.Popen([
        "gz", "service",
        "-s", "/world/capstone_world/apply_link_wrench",
        "--reqtype", "gz.msgs.EntityWrench",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "500", "--req", req
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

class ConveyorNode(Node):
    def __init__(self):
        super().__init__("conveyor_node")
        self.get_logger().info("Waiting for world to load...")
        time.sleep(5.0)
        for name, x, amb, dif, emi in CUBES:
            spawn_cube(name, x, amb, dif, emi)
            time.sleep(1.0)
        self.get_logger().info("All cubes spawned on belt!")
        self.timer = self.create_timer(2.0, self.push_all)

    def push_all(self):
        for name, _, _, _, _ in CUBES:
            push_cube(name)

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
