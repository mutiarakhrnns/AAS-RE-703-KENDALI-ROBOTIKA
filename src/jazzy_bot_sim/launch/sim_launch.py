"""
Launch file for JazzyBot FSM simulation in Gazebo Harmonic.

This launch file orchestrates:
1. Gazebo Harmonic simulator
2. Robot model spawning
3. ROS-Gazebo bridges for communication
4. FSM control node for autonomous behavior

Author: Your Name
Date: 2026
License: Apache-2.0
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    
    # ===== PACKAGE PATHS =====
    pkg_jazzy_bot_sim = get_package_share_directory('jazzy_bot_sim')
    urdf_file = os.path.join(pkg_jazzy_bot_sim, 'urdf', 'jazzy_bot.urdf.xacro')
    bridge_config = os.path.join(
    pkg_jazzy_bot_sim,
    'config',
    'bridge.yaml'
)

    # ===== LAUNCH ARGUMENTS =====
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    spawn_x_arg = DeclareLaunchArgument(
        'spawn_x',
        default_value='0.0',
        description='X coordinate for robot spawn'
    )
    
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y',
        default_value='0.0',
        description='Y coordinate for robot spawn'
    )
    
    spawn_z_arg = DeclareLaunchArgument(
        'spawn_z',
        default_value='0.3',
        description='Z coordinate for robot spawn'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    spawn_x = LaunchConfiguration('spawn_x')
    spawn_y = LaunchConfiguration('spawn_y')
    spawn_z = LaunchConfiguration('spawn_z')
    
    # ===== ROBOT DESCRIPTION =====
    # Process xacro file to generate URDF
    robot_description_content = xacro.process_file(urdf_file).toxml()
    
    # ===== ROBOT STATE PUBLISHER =====
    # Publishes robot_description and TF transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time,
            'publish_frequency': 30.0
        }]
    )
    
    # ===== GAZEBO SIMULATOR =====
    # Launch Gazebo Harmonic with empty world
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen',
        shell=False
    )
    
    # ===== ROBOT SPAWNER =====
    # Spawn robot entity in Gazebo at specified pose
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'jazzy_bot',
            '-topic', 'robot_description',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z
        ],
        output='screen'
    )
    
    
    gz_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    parameters=[{
        'use_sim_time': use_sim_time,
        'config_file': bridge_config
    }],
    output='screen'
)

    # Delay spawn to ensure Gazebo and robot_state_publisher are ready
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn_entity]
    )
    
    # ===== ROS-GAZEBO BRIDGES =====
    
    # Clock Bridge: Synchronizes ROS time with Gazebo time
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )
    
    # cmd_vel Bridge: ROS → Gazebo (control commands)
    # The ] symbol indicates direction: ROS to Gazebo
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/jazzy_bot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'
        ],
        remappings=[
            ('/model/jazzy_bot/cmd_vel', '/cmd_vel')
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # LiDAR Bridge: Gazebo → ROS (sensor data)
    # The [ symbol indicates direction: Gazebo to ROS
    bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"
        ],
        # remappings=[
        #     ('/world/empty/model/jazzy_bot/link/lidar_link/sensor/lidar_sensor/scan', '/scan')
        # ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Odometry Bridge: Gazebo → ROS (optional, for monitoring)
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/jazzy_bot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        remappings=[
            ('/model/jazzy_bot/odometry', '/odom')
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    
    
    # Delay bridges to ensure robot is spawned first
    delayed_bridges = TimerAction(
    period=7.0,
    actions=[gz_bridge]
)


    # delayed_bridges = TimerAction( period=7.0, actions=[bridge_cmd_vel, bridge_scan, bridge_odom] )
    
    # ===== FSM CONTROL NODE =====
    # Autonomous navigation controller
    fsm_node = Node(
        package='jazzy_bot_sim',
        executable='fsm_node',
        name='fsm_controller',
        parameters=[{
            'use_sim_time': use_sim_time,
            'wander_linear_velocity': 0.3,
            'recovery_angular_velocity': 0.5,
            'obstacle_threshold': 0.8,
            'recovery_duration': 2.0,
            'control_frequency': 20.0
        }],
        output='screen'
    )
    
    # Delay FSM node to ensure all bridges are active
    delayed_fsm = TimerAction(
        period=9.0,
        actions=[fsm_node]
    )
    
    # ===== LAUNCH DESCRIPTION =====
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        
        # Core components (in order)
        robot_state_publisher,  # 1. Publish robot model
        gz_sim,                 # 2. Start Gazebo
        # bridge_clock,           # 3. Synchronize time
        delayed_spawn,          # 4. Spawn robot (5s delay)
        delayed_bridges,        # 5. Start data bridges (7s delay)
        delayed_fsm             # 6. Start FSM controller (9s delay)
    ])