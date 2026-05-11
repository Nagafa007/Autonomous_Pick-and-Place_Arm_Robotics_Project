#!/usr/bin/env python3
"""
Circular Factory Conveyor - Optimized
- Reaps subprocesses to prevent zombie process flooding
- Uses synchronous service calls for better stability
"""
import rclpy
from rclpy.node import Node
import subprocess, time

CUBE_SDF = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
    <static>true</static>
    <pose>{x} 1.2 0.60 0 0 0</pose>
    <link name='link'>
      <inertial><mass>0.08</mass>
        <inertia><ixx>0.00005</ixx><ixy>0</ixy><ixz>0</ixz>
                 <iyy>0.00005</iyy><iyz>0</iyz><izz>0.00005</izz></inertia>
      </inertial>
      <collision name='col'>
        <geometry><box><size>0.15 0.15 0.15</size></box></geometry>
        <surface><friction><ode><mu>0.4</mu><mu2>0.4</mu2></ode></friction></surface>
      </collision>
      <visual name='vis'>
        <geometry><box><size>0.15 0.15 0.15</size></box></geometry>
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

NUM_CUBES = 8
START_X   = 2.2
END_X     = -2.5
SPACING   = 0.5
BELT_Y    = 1.2
BELT_Z    = 0.60
STOP_X    = 0.0
STOP_SEC  = 10.0
STEP      = 0.02
TICK      = 0.1     # 10Hz for smoother movement

CUBES = []
for i in range(NUM_CUBES):
    color_name, amb, dif, emi = COLOR_PROFILES[i % len(COLOR_PROFILES)]
    CUBES.append({
        "name":     f"{color_name}_cube_{i}",
        "x":        START_X - (i * SPACING),
        "ambient":  amb,
        "diffuse":  dif,
        "emissive": emi,
    })

def spawn_cube(cube):
    sdf = CUBE_SDF.format(
        name=cube["name"], x=cube["x"],
        ambient=cube["ambient"], diffuse=cube["diffuse"],
        emissive=cube["emissive"]
    )
    path = f"/tmp/{cube['name']}.sdf"
    with open(path, "w") as f:
        f.write(sdf)
    subprocess.run(
        ["ros2", "run", "ros_gz_sim", "create",
         "-file", path, "-name", cube["name"],
         "-x", str(cube["x"]), "-y", str(BELT_Y), "-z", str(BELT_Z)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

def set_pose(name, x):
    req = (f'name: "{name}" '
           f'position: {{x: {x:.3f}, y: {BELT_Y}, z: {BELT_Z}}} '
           f'orientation: {{w: 1.0, x: 0.0, y: 0.0, z: 0.0}}')
    # Use Popen but we will collect them
    return subprocess.Popen([
        "gz", "service",
        "-s", "/world/capstone_world/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "100", "--req", req
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

class ConveyorNode(Node):
    MOVING  = "MOVING"
    STOPPED = "STOPPED"

    def __init__(self):
        super().__init__("conveyor_node")
        self.get_logger().info("Initializing Conveyor...")
        
        self.cubes        = CUBES
        self.belt_state   = self.MOVING
        self.stop_elapsed = 0.0

        for cube in self.cubes:
            spawn_cube(cube)
            time.sleep(0.5)

        self.get_logger().info("CONVEYOR RUNNING")
        self.create_timer(TICK, self.tick)

    def tick(self):
        # Move all at constant speed
        procs = []
        for cube in self.cubes:
            new_x = cube["x"] - STEP
            if new_x <= END_X:
                new_x = START_X
            cube["x"] = new_x
            procs.append(set_pose(cube["name"], new_x))
        
        # Wait for all to finish to prevent process leak
        for p in procs:
            try:
                p.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                p.kill()

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
