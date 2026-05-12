#!/bin/bash

# ==========================================
# CAPSTONE MASTER LAUNCH SCRIPT (Mycobot 280)
# ==========================================

# 1. Cleanup old processes
echo "[INFO] Cleaning up previous sessions..."
pkill -f gz || true
pkill -f ros2 || true
pkill -f rqt || true
sleep 3

# 2. Fix Graphics & Networking
export GZ_IP_ADDRESS=127.0.0.1
export ROS_LOCALHOST_ONLY=1
export QT_X11_NO_MITSHM=1
export GZ_PARTITION=capstone_sim

# 3. The Cleanup Trap
trap 'echo -e "\n[INFO] Shutting down all Capstone nodes..."; kill $(jobs -p); exit' SIGINT

SCRIPT_DIR=$(dirname "$(readlink -f "$0")")

echo "[INFO] Sourcing ROS 2 Workspace..."
if [ ! -f "$SCRIPT_DIR/install/setup.bash" ]; then
    echo "[ERROR] Workspace not built. Run 'colcon build' first."
    exit 1
fi
source "$SCRIPT_DIR/install/setup.bash"

# 4. Start the Physics World
echo "[INFO] Booting Gazebo Harmonic..."
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="$SCRIPT_DIR/worlds/capstone_world.sdf -r" &
sleep 10

# 5. Inject Mycobot Robotic Arm and Controllers
echo "[INFO] Injecting Mycobot Arm and starting controllers..."
ros2 launch my_capstone_brain mycobot_spawn.launch.py &
sleep 10

# 6. Start the Conveyor Belt
echo "[INFO] Bridging the Conveyor Belt Plugin..."
ros2 run ros_gz_bridge parameter_bridge /model/conveyor_belt_model/link/belt_link/track_cmd_vel@std_msgs/msg/Float64@gz.msgs.Double &
sleep 2

echo "[INFO] Starting Conveyor Node..."
ros2 run my_capstone_brain conveyor_node &

echo "==================================================="
echo "✅ SIMULATION READY AND RUNNING!"
echo "👉 TO CONTROL THE ARM, OPEN A NEW TERMINAL AND RUN:"
echo "   cd $SCRIPT_DIR"
echo "   source install/setup.bash"
echo "   ros2 run my_capstone_brain control_node"
echo "==================================================="

# Keep the script running
wait