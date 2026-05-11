#!/bin/bash

# Cleanup first
echo "Cleaning up..."
pkill -9 gz
pkill -9 ruby
pkill -9 ros2
sleep 2

export LIBGL_ALWAYS_SOFTWARE=1
export ROS_LOCALHOST_ONLY=1
WS_ROOT="/home/ziad/ros2_ws/src/final_proj/Autonomous_Pick-and-Place_Arm_Robotics_Project"
source $WS_ROOT/install/setup.bash
export GZ_SIM_RESOURCE_PATH="$WS_ROOT/src/Arduino-Bot"

echo "Starting Terminal 1: Gazebo..."
ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="$WS_ROOT/worlds/capstone_world.sdf -r" > gazebo.log 2>&1 &
GZ_PID=$!

sleep 10

echo "Starting Terminal 2: Bridges..."
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock > bridge_clock.log 2>&1 &
ros2 run ros_gz_image image_bridge /camera/image > bridge_image.log 2>&1 &

sleep 5

echo "Starting Terminal 3: Robot Spawn..."
xacro $(ros2 pkg prefix arduinobot_description --share)/urdf/arduinobot.urdf.xacro is_ignition:="false" > /tmp/robot.urdf
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat /tmp/robot.urdf)" -p use_sim_time:=true > rsp.log 2>&1 &
sleep 5
ros2 run ros_gz_sim create -topic robot_description -name arduinobot -z 0.1 > spawn.log 2>&1

sleep 5

echo "Starting Terminal 4: Controllers..."
ros2 run controller_manager spawner joint_state_broadcaster > controller_jsb.log 2>&1
ros2 run controller_manager spawner arm_controller > controller_arm.log 2>&1
ros2 run controller_manager spawner gripper_controller > controller_gripper.log 2>&1

echo "Starting Terminal 5: Conveyor..."
ros2 run my_capstone_brain conveyor_node > conveyor.log 2>&1 &

# echo "Starting Terminal 6: Vision & IK..."
# ros2 run my_capstone_brain vision_node > vision.log 2>&1 &
# ros2 run my_capstone_brain ik_node > ik.log 2>&1 &

echo "All systems launched. Monitoring Gazebo (PID $GZ_PID)..."
echo "Check *.log files for details."

# Monitor Gazebo
while kill -0 $GZ_PID 2>/dev/null; do
    sleep 5
done

echo "Gazebo has CLOSED. Diagnostic logs:"
tail -n 20 gazebo.log
