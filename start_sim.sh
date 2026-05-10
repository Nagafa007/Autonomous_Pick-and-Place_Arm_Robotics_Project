#!/bin/bash
# start_sim.sh

# 1. Cleanup old processes
echo "Cleaning up old processes..."
pkill -9 gz
pkill -9 ruby
pkill -9 ros2
sleep 2

export LIBGL_ALWAYS_SOFTWARE=1
export ROS_LOCALHOST_ONLY=1
WS_ROOT="/home/ziad/ros2_ws/src/final_proj/Autonomous_Pick-and-Place_Arm_Robotics_Project"
source $WS_ROOT/install/setup.bash
export GZ_SIM_RESOURCE_PATH="$WS_ROOT/src/Arduino-Bot"

echo "1. Starting Gazebo (Headless-ish)..."
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="$WS_ROOT/worlds/capstone_world.sdf -r" > /tmp/gazebo.log 2>&1 &

echo "2. Starting Bridges..."
sleep 8
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock > /tmp/bridge.log 2>&1 &
ros2 run ros_gz_image image_bridge /camera/image > /tmp/image.log 2>&1 &

echo "3. Spawning Robot..."
sleep 5
xacro $(ros2 pkg prefix arduinobot_description --share)/urdf/arduinobot.urdf.xacro is_ignition:="false" > /tmp/robot.urdf
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat /tmp/robot.urdf)" -p use_sim_time:=true > /tmp/rsp.log 2>&1 &
sleep 5
ros2 run ros_gz_sim create -topic robot_description -name arduinobot -z 0.1

echo "4. Spawning Controllers..."
sleep 5
ros2 run controller_manager spawner joint_state_broadcaster > /tmp/c1.log 2>&1
ros2 run controller_manager spawner arm_controller > /tmp/c2.log 2>&1
ros2 run controller_manager spawner gripper_controller > /tmp/c3.log 2>&1

echo "5. Starting Brain (Conveyor, Vision, IK)..."
ros2 run my_capstone_brain conveyor_node > /tmp/conveyor.log 2>&1 &
ros2 run my_capstone_brain vision_node > /tmp/vision.log 2>&1 &
ros2 run my_capstone_brain ik_node > /tmp/ik.log 2>&1 &

echo "------------------------------------------------"
echo "SIMULATION STARTED SUCCESSFULLY!"
echo "To stop everything, run: ./stop_sim.sh"
echo "To see what the robot sees, run: ros2 run rqt_image_view rqt_image_view"
echo "------------------------------------------------"
