#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import cv2

def generate_launch_description():
    # Get directory paths
    turtlebot4_safety_dir = get_package_share_directory('turtlebot4_safety')
    turtlebot4_navigation_dir = get_package_share_directory('turtlebot4_navigation')
    turtlebot4_bringup_dir = get_package_share_directory('turtlebot4_bringup')
    turtlebot4_viz_dir = get_package_share_directory('turtlebot4_viz')
    
    # Map file path (if needed)
    map_dir = os.path.join(turtlebot4_safety_dir, 'maps')
    map_yaml_file = os.path.join(map_dir, 'lab211_empty.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')  # Set to false for physical robot
    
    # Include the TurtleBot4 physical robot bringup
    turtlebot4_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot4_bringup_dir, 'launch', 'robot.launch.py')
        ])
    )
    
    # Launch SLAM - using async for better performance on Raspberry Pi
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot4_navigation_dir, 'launch', 'slam.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'sync': 'false',  # Use async SLAM for better performance on Raspberry Pi
        }.items()
    )
    
    # Launch Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot4_navigation_dir, 'launch', 'nav2.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Include RViz visualization
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot4_viz_dir, 'launch', 'view_robot.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Fiducial detection node
    fiducial_detection_node = Node(
        package='turtlebot4_safety',
        executable='fiducial_detection_node',
        name='fiducial_detection_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'marker_size': 0.2,  # Size in meters
            'dictionary_id': cv2.aruco.DICT_5X5_100,
            'tf_buffer_duration': 30.0,  # Longer buffer duration
            'tf_timeout': 1.0  # Longer timeout for transform lookups
        }]
    )
    
    # Direction navigation node
    direction_navigation_node = Node(
        package='turtlebot4_safety',
        executable='direction_navigation_node',
        name='direction_navigation_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'turn_angle': 1.5708,  # 90 degrees in radians
            'forward_distance': 2.0,  # 2 meters
            'tf_buffer_duration': 30.0,
            'tf_timeout': 1.0
        }]
    )
    
    # Safety parameter node
    safety_params_node = Node(
        package='turtlebot4_safety',
        executable='safety_params',
        name='safety_parameter_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'inflation_radius': 0.20,       # Higher value gives obstacles wider berth
            'obstacle_range': 3.5,          # How far to look for obstacles
            'min_approach_distance': 0.15,  # Min distance to obstacles
            'max_vel_x': 0.15,              # Conservative max speed
            'footprint_padding': 0.08,      # Extra padding around robot
        }]
    )
    
    # TF buffer server with increased buffer duration
    tf_buffer_server = Node(
        package='tf2_ros',
        executable='buffer_server',
        name='tf_buffer_server',
        parameters=[{
            'buffer_duration': 15.0,  # Seconds
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # Define LaunchDescription variable and actions
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='false',
                             description='Use simulation time if true'),
        
        # Launch physical robot
        turtlebot4_robot_launch,
        
        # Launch SLAM
        slam_launch,
        
        # Launch Nav2
        nav2_launch,
        
        # Launch RViz visualization
        rviz_launch,
        
        # Launch TF buffer server
        tf_buffer_server,
        
        # Launch fiducial detection node
        fiducial_detection_node,
        
        # Launch direction navigation node
        direction_navigation_node,
        
        # Launch safety parameter node
        safety_params_node,
    ])