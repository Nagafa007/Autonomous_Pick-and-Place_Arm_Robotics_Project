#!/usr/bin/env python3
"""
Circular Factory Conveyor
- All boxes move together at equal spacing
- Belt fully freezes (all boxes stop exactly where they are)
- Box reaching belt end teleports to START_X (beginning of belt)
- Forms fresh groups of 8 naturally due to equal speed
- Stop time = 10 seconds
"""
import rclpy
from rclpy.node import Node
import subprocess, time

CUBE_SDF = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='{name}'>
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
START_X   = 2.2     # Right edge of belt — recycled boxes appear here
END_X     = -2.5    # Left edge — boxes recycle when they cross this
SPACING   = 0.45
BELT_Y    = 1.2
BELT_Z    = 0.60
STOP_X    = 0.0     # In front of arm
STOP_SEC  = 10.0
STEP      = 0.05    # Meters per tick
TICK      = 0.5     # Seconds per tick

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
    subprocess.Popen(
        ["ros2", "run", "ros_gz_sim", "create",
         "-file", path, "-name", cube["name"],
         "-x", str(cube["x"]), "-y", str(BELT_Y), "-z", str(BELT_Z)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )
    print(f"[Conveyor] Spawned {cube['name']} at x={cube['x']:.2f}")


def set_pose(name, x):
    req = (f'name: "{name}" '
           f'position: {{x: {x:.3f}, y: {BELT_Y}, z: {BELT_Z}}} '
           f'orientation: {{w: 1.0, x: 0.0, y: 0.0, z: 0.0}}')
    subprocess.Popen([
        "gz", "service",
        "-s", "/world/capstone_world/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "300", "--req", req
    ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class ConveyorNode(Node):

    MOVING  = "MOVING"
    STOPPED = "STOPPED"

    def __init__(self):
        super().__init__("conveyor_node")
        self.get_logger().info("Waiting for world to load...")
        time.sleep(5.0)

        self.cubes        = CUBES
        self.belt_state   = self.MOVING
        self.stop_elapsed = 0.0

        for cube in self.cubes:
            spawn_cube(cube)
            time.sleep(0.8)

        self.get_logger().info("=" * 50)
        self.get_logger().info("  CONVEYOR RUNNING ▶")
        self.get_logger().info("=" * 50)
        self.create_timer(TICK, self.tick)

    # ------------------------------------------------------------------
    def tick(self):

        # ── BELT STOPPED ───────────────────────────────────────────────
        if self.belt_state == self.STOPPED:
            self.stop_elapsed += TICK
            remaining = STOP_SEC - self.stop_elapsed
            if remaining > 0:
                self.get_logger().info(
                    f"  🛑 Belt stopped — resuming in {remaining:.0f}s")
            else:
                self.stop_elapsed = 0.0
                self.belt_state   = self.MOVING
                self.get_logger().info("  ▶ Belt RESUMING")
            return   # ← ALL boxes stay frozen, nothing moves

        # ── BELT MOVING ────────────────────────────────────────────────

        # Check first if any box will cross STOP_X this tick
        crossing = next(
            (c for c in self.cubes
             if c["x"] > STOP_X >= (c["x"] - STEP)),
            None
        )

        if crossing:
            # Snap crossing box to arm position exactly
            crossing["x"] = STOP_X
            set_pose(crossing["name"], STOP_X)

            # ALL other boxes stay exactly where they are — don't move them
            self.belt_state   = self.STOPPED
            self.stop_elapsed = 0.0
            self.get_logger().info("=" * 50)
            self.get_logger().info(
                f"  🛑 [{crossing['name']}] at arm — BELT STOPPED (10s)")
            self.get_logger().info("=" * 50)
            return

        # No stop triggered — move ALL boxes by one step
        for cube in self.cubes:
            new_x = cube["x"] - STEP

            # ♻ Box reached belt end → teleport to START_X
            if new_x <= END_X:
                new_x = START_X
                self.get_logger().info(
                    f"  ♻ [{cube['name']}] recycled → back to x={START_X}")

            cube["x"] = new_x
            set_pose(cube["name"], new_x)


def main(args=None):
    rclpy.init(args=args)
    node = ConveyorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
