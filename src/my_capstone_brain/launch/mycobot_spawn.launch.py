import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Paths
    pkg_mycobot_description = FindPackageShare('mycobot_description')
    
    # Launch Configurations
    robot_name = LaunchConfiguration('robot_name', default='mycobot_280')
    x = LaunchConfiguration('x', default='0.0')
    y = LaunchConfiguration('y', default='0.0')
    z = LaunchConfiguration('z', default='0.0')
    yaw = LaunchConfiguration('yaw', default='0.0') # Facing forward (belt is at +Y)

    # Robot Description (XACRO)
    robot_description_content = ParameterValue(
        Command([
            'xacro ',
            PathJoinSubstitution([pkg_mycobot_description, 'urdf', 'robots', 'mycobot_280.urdf.xacro']),
            ' use_gazebo:=true'
        ]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    # Spawn Robot in Gazebo
    spawn_robot_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', robot_name,
            '-allow_renaming', 'true',
            '-x', x,
            '-y', y,
            '-z', z,
            '-Y', yaw
        ]
    )

    # Bridge for Joint States, Clock, and Camera
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/capstone_world/model/mycobot_280/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        remappings=[
            ('/world/capstone_world/model/mycobot_280/joint_state', '/joint_states')
        ],
        output='screen'
    )

    # Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"],
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_action_controller"],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        spawn_robot_node,
        bridge_node,
        TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=5.0, actions=[arm_controller_spawner]),
        TimerAction(period=7.0, actions=[gripper_controller_spawner]),
    ])
