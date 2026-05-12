# 🤖 Robotics Capstone: Autonomous Pick-and-Place Arm

## 📖 Project Overview

This repository contains the simulation and control software for a 3-DOF robotic arm (EEZYbotARM MK1/MK2) designed to autonomously detect, pick up, and place objects.

To overcome hardware limitations (Arduino Uno memory constraints), this project uses a distributed architecture:

-   **The Brain (Laptop):** Runs ROS 2, handles Inverse Kinematics (IK), and processes Computer Vision (OpenCV).
    
-   **The Eyes (Overhead Camera):** An "Eye-to-Hand" top-down camera setup (simulated in Gazebo, physically an Android phone).
    
-   **The Muscle (Arduino Uno):** Receives target joint angles via serial communication and actuates the servos.
    

Currently, this repository contains the **Minimum Viable Product (MVP) Simulation**, fully bridging ROS 2 Math/IK nodes with a Gazebo physics environment and a simulated overhead camera feed.

* * *

## 🛠️ Prerequisites & System Requirements

This project supports **Ubuntu 22.04 (ROS 2 Humble)** and **Ubuntu 24.04 (ROS 2 Jazzy)**.

### ⚠️ Special Instructions for WSL Users

If you are running Windows Subsystem for Linux (WSL), Gazebo and ROS 2 GUI tools require a few specific configurations to avoid crashing:

1.  **Windows 11 is highly recommended** because it includes WSLg (native GUI support).
    
2.  **Hardware Acceleration:** Gazebo requires 3D acceleration. If Gazebo crashes on startup, force software rendering by adding this to your `~/.bashrc`:
    
    Bash
    
        export LIBGL_ALWAYS_SOFTWARE=1
    
3.  **Network Isolation:** Fast-DDS (ROS 2's networking backend) often gets confused by WSL's virtual network adapters, causing infinite loops when looking for controllers. Add this to your `~/.bashrc` to isolate ROS traffic:
    
    Bash
    
        export ROS_LOCALHOST_ONLY=1
    

### Core Dependencies

Run the following commands to install the necessary simulation, control, and vision libraries based on your OS:

**For Ubuntu 24.04 (Jazzy):**

Bash

    sudo apt update && sudo apt upgrade -y
    sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-joint-state-publisher-gui ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-image ros-jazzy-rqt-image-view ros-jazzy-rqt-joint-trajectory-controller ros-jazzy-cv-bridge python3-opencv libserial-dev -y

**For Ubuntu 22.04 (Humble):**

Bash

    sudo apt update && sudo apt upgrade -y
    sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-ros-ign ros-humble-ign-ros2-control ros-humble-joint-state-publisher-gui ros-humble-ros-ign-bridge ros-humble-ros-ign-image ros-humble-rqt-image-view ros-humble-rqt-joint-trajectory-controller ros-humble-cv-bridge python3-opencv libserial-dev -y

* * *

## 📁 Repository Structure

Plaintext

    capstone_ws/
    │
    ├── src/
    │   ├── Arduino-Bot/              # Cloned URDF/Meshes for the EEZYbotARM
    │   │   ├── arduinobot_description/ # 3D models and robot skeleton
    │   │   ├── arduinobot_controller/  # ros2_control hardware interface
    │   │   └── ...
    │   │
    │   └── my_capstone_brain/        # Custom Python Nodes (Our logic)
    │       ├── ik_node.py            # Calculates Inverse Kinematics geometry
    │       └── vision_node.py        # OpenCV object detection (WIP)

* * *

## 🚀 Installation & Setup

**CRITICAL RULE:** Never put this workspace in a folder path that contains spaces (e.g., `New Volume/3rd year/`). ROS 2's URDF parser (`xacro`) will crash. Clone this directly into your home directory (`~/`).

1.  **Clone the Repository:**
    
    Bash
    
        git clone <YOUR_GITHUB_REPO_URL> ~/capstone_ws
        cd ~/capstone_ws
    
2.  **Bypass the Firmware Package:** The `arduinobot_firmware` package contains C++ code that frequently fails to link against `libserial` on newer Ubuntu systems. Since we are using Python (`pyserial`) for Arduino communication, we will tell the compiler to ignore it, along with unneeded C++ examples.
    
    Bash
    
        touch src/Arduino-Bot/arduinobot_firmware/COLCON_IGNORE
        touch src/Arduino-Bot/arduinobot_remote/COLCON_IGNORE
        touch src/Arduino-Bot/arduinobot_cpp_examples/COLCON_IGNORE
    
3.  **Resolve ROS Dependencies:**
    
    Bash
    
        rosdep update
        rosdep install --from-paths src --ignore-src -r -y
    
4.  **Build the Workspace:** If you ever upgrade system packages, delete the `build/`, `install/`, and `log/` folders before running this command to avoid Fast-DDS ABI mismatch errors.
    
    Bash
    
        colcon build --symlink-install
    

* * *

## 🎮 Running the Simulation

Because of the transition from Gazebo Classic (Humble) to Gazebo Harmonic (Jazzy), the standard launch files require specific arguments. To ensure maximum stability, we launch the matrix using 4 separate terminals.

_Note: Replace `is_ignition:="false"` with `is_ignition:="true"` if you are using ROS 2 Humble._

**Terminal 1: Start the Physics World** _We export the resource path so Gazebo can find the `.STL` 3D meshes; otherwise, the robot will be invisible._

Bash

    source ~/capstone_ws/install/setup.bash
    export GZ_SIM_RESOURCE_PATH=/home/$USER/capstone_ws/src/Arduino-Bot
    ros2 launch ros_gz_sim gz_sim.launch.py gz_args:="empty.sdf -r"

**Terminal 2: Publish the Blueprint** _We use `use_sim_time:=true` to sync the ROS clock with the Gazebo physics clock._

Bash

    source ~/capstone_ws/install/setup.bash
    xacro $(ros2 pkg prefix arduinobot_description --share)/urdf/arduinobot.urdf.xacro is_ignition:="false" > /tmp/robot.urdf
    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(cat /tmp/robot.urdf)" -p use_sim_time:=true

**Terminal 3: Spawn Robot & Activate Motors**

Bash

    source ~/capstone_ws/install/setup.bash
    ros2 run ros_gz_sim create -topic robot_description -name arduinobot -z 0.1
    ros2 run controller_manager spawner joint_state_broadcaster
    ros2 run controller_manager spawner arm_controller
    ros2 run controller_manager spawner gripper_controller

**Terminal 4: Start the Vision Bridge**

Bash

    source ~/capstone_ws/install/setup.bash
    ros2 run ros_gz_image image_bridge /image_raw &
    ros2 run rqt_image_view rqt_image_view

_In `rqt_image_view`, select `/image_raw` from the top dropdown to see the top-down satellite view of the workspace._

* * *

## 🚨 Troubleshooting & Known Issues

-   **Error: `link 'overhead_camera_link' is not unique`**
    
    -   _Cause:_ The URDF file has duplicate XML tags. Ensure there is only one `<link name="overhead_camera_link">` at the bottom of `arduinobot.urdf.xacro`.
        
-   **Gazebo Crashes Instantly / Spawners Hanging:**
    
    -   _Cause:_ Zombie Fast-DDS nodes are blocking the ports from a previous bad shutdown.
        
    -   _Fix:_ Run `pkill -f gz` and `pkill -f ros2`, then restart the terminals.
        
-   **The camera feed in `rqt_image_view` is pure white:**
    
    -   _Cause:_ The camera is pointing at the Gazebo sky or spawned inside its own collision mesh. Ensure the URDF origin is set to `<origin xyz="0.30 0.0 10.0" rpy="0 1.5708 0"/>` (Z = 10 meters for bird's-eye view).
        
-   **Terminal spams `Compressed Depth Image Transport` errors:**
    
    -   _Cause:_ ROS 2 is trying to compress a 2D RGB stream using a 3D depth algorithm.
        
    -   _Fix:_ Ignore it. It is a false alarm and does not affect the raw `/image_raw` stream.
-   **Error: `Package 'my_capstone_brain' not found`:**

    -   _Cause:_ You are trying to run a node in a new terminal that hasn't been "sourced" yet.
    
    -   _Fix:_ Run `source install/setup.bash` in the new terminal before running any `ros2` commands. Alternatively, use the `launch_capstone.sh` script which handles this for you.
