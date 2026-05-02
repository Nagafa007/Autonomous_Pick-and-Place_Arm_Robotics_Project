#!/bin/bash

# ==========================================
# CAPSTONE MASTER LAUNCH SCRIPT
# ==========================================

# 1. The Cleanup Trap: If you press Ctrl+C, this automatically kills ALL background processes
trap 'echo -e "\n[INFO] Shutting down all Capstone nodes..."; kill $(jobs -p); exit' SIGINT

echo "[INFO] Sourcing ROS 2 Workspace..."
source ~/ros2_ws/src/Autonomous_Pick-and-Place_Arm_Robotics_Project/install/setup.bash
export GZ_SIM_RESOURCE_PATH=/home/ady/ros2_ws/src/Autonomous_Pick-and-Place_Arm_Robotics_Project/src/Arduino-Bot

# 2. Start the Physics World in the background (&)
echo "[INFO] Booting Gazebo Harmonic..."
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="/home/ady/ros2_ws/src/Autonomous_Pick-and-Place_Arm_Robotics_Project/worlds/capstone_world.sdf -r" &
sleep 4 # Give Gazebo 4 seconds to open

# 3. Publish the Blueprint
echo "[INFO] Publishing Robot Blueprint..."
xacro $(ros2 pkg prefix arduinobot_description --share)/urdf/arduinobot.urdf.xacro is_ignition:="false" > /tmp/robot.urdf
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat /tmp/robot.urdf)" -p use_sim_time:=true &
sleep 2

# 4. Drop the Robot into the world
echo "[INFO] Spawning EEZYbotARM..."
ros2 run ros_gz_sim create -topic robot_description -name arduinobot -z 0.1
sleep 2

# 5. Activate the Motors
echo "[INFO] Activating Motor Controllers..."
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner arm_controller
ros2 run controller_manager spawner gripper_controller

# 6. Start the Vision Pipeline
echo "[INFO] Establishing Video Feed..."
ros2 run ros_gz_image image_bridge /image_raw &
ros2 run rqt_image_view rqt_image_view &

# 7. Start the Control Dashboard
echo "[INFO] Opening Control Dashboard..."
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller &

echo "==================================================="
echo "🚀 CAPSTONE SIMULATION ONLINE 🚀"
echo "Press [Ctrl+C] in this terminal to safely shut down."
echo "==================================================="


echo "[INFO] Starting Conveyor Node..."
ros2 run my_capstone_brain conveyor_node &

# 8. Keep the script running so the trap works
wait