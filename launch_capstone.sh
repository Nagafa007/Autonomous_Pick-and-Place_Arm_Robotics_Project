#!/bin/bash
trap 'echo "Shutting down..."; kill $(jobs -p); exit' SIGINT

PROJECT_DIR="$HOME/ros2_ws/src/final_proj/Autonomous_Pick-and-Place_Arm_Robotics_Project"
source $PROJECT_DIR/install/setup.bash
export GZ_SIM_RESOURCE_PATH=$PROJECT_DIR/src/Arduino-Bot

echo "1. Booting Gazebo..."
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="$PROJECT_DIR/worlds/capstone_world.sdf -r" &
sleep 8 # Big pause to let WSL graphics load

echo "2. Spawning Robot..."
xacro $(ros2 pkg prefix arduinobot_description --share)/urdf/arduinobot.urdf.xacro is_ignition:="false" > /tmp/robot.urdf
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat /tmp/robot.urdf)" -p use_sim_time:=true &
ros2 run ros_gz_sim create -topic robot_description -name arduinobot -z 0.1
sleep 5 # Pause to let arm drop to the table

echo "3. Activating Motors..."
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner arm_controller
ros2 run controller_manager spawner gripper_controller
sleep 3

echo "4. Starting Camera..."
ros2 run ros_gz_bridge parameter_bridge /camera/image@sensor_msgs/msg/Image@gz.msgs.Image &
ros2 run rqt_image_view rqt_image_view &
sleep 5 # Pause to let camera feed stabilize

echo "5. Starting Conveyor..."
ros2 run my_capstone_brain conveyor_node &

echo "SYSTEM READY."
wait