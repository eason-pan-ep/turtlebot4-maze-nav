#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get directory paths
    turtlebot4_safety_dir = get_package_share_directory('turtlebot4_safety')
    turtlebot4_ignition_bringup_dir = get_package_share_directory('turtlebot4_ignition_bringup')
    
    # Map file path - using your empty map that has walls but no obstacles
    map_dir = os.path.join(get_package_share_directory('turtlebot4_safety'), 'maps')
    map_file = os.path.join(map_dir, 'lab211_empty.yaml')
    
    # Nav2 parameters file
    nav2_params_file = os.path.join(get_package_share_directory('turtlebot4_safety'), 
                                   'config', 'nav2_params.yaml')
    
    # Include the TurtleBot4 Ignition simulator launch
    turtlebot4_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot4_ignition_bringup_dir, 'launch', 'turtlebot4_ignition.launch.py')
        ]),
        launch_arguments={
            'world': 'lab211_with_boxes_1',  # Your custom world
            'rviz': 'true',                  # Launch with RViz
            'model': 'lite',                # Use the LITE model
            'nav2': 'true',                  # Launch with Nav2
            'slam': 'false',                 # No SLAM (using map)
            'localization': 'true',          # Use AMCL localization
            'map': map_file,                 # Use empty map
            'nav2_params_file': nav2_params_file  # Use your custom nav2 params
        }.items()
    )
    
    # Safety parameter node with timer
    safety_node = TimerAction(
        period=5.0,  # Start after navigation
        actions=[
            Node(
                package='turtlebot4_safety',
                executable='safety_params',
                name='safety_parameter_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'inflation_radius': 0.15,
                    'obstacle_range': 3.5,
                    'min_approach_distance': 0.05,
                    'max_vel_x': 0.15,
                    'footprint_padding': 0.05,
                }]
            )
        ]
    )
    
    # Obstacle avoidance navigation node with timer
    obstacle_nav_node = TimerAction(
        period=8.0,  # Start after safety params
        actions=[
            Node(
                package='turtlebot4_safety',
                executable='obstacle_avoidance_nav',
                name='obstacle_avoidance_navigator',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'obstacle_threshold': 0.5,
                    'scan_angle_min': -60.0,
                    'scan_angle_max': 60.0,
                    'inflation_radius': 0.15,
                    'footprint_padding': 0.05,
                }]
            )
        ]
    )
    
    # Robot localization node with timer
    localization_node = TimerAction(
        period=3.0,  # Start early
        actions=[
            Node(
                package='turtlebot4_safety',
                executable='robot_localization_node',
                name='robot_localization_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'frequency': 30.0,
                    'publish_tf': False,  # Don't publish tf to avoid conflicts with AMCL
                }]
            )
        ]
    )
    
    # Reset pose service with timer
    reset_pose_node = TimerAction(
        period=10.0,  # Wait for everything to start
        actions=[
            Node(
                package='turtlebot4_safety',
                executable='reset_pose_service',
                name='reset_pose_service',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'default_x': 0.0,
                    'default_y': 0.0,
                    'default_yaw': 0.0,
                }]
            )
        ]
    )
    
    # AMCL tuner node with timer
    amcl_tuner_node = TimerAction(
        period=15.0,  # Wait for AMCL to initialize
        actions=[
            Node(
                package='turtlebot4_safety',
                executable='amcl_tuner',
                name='amcl_tuner',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'check_interval': 5.0,
                    'drift_threshold': 0.5,
                    'rotation_drift_threshold': 0.3,
                }]
            )
        ]
    )
    
    return LaunchDescription([
        # Launch TurtleBot4 Ignition simulation
        turtlebot4_sim_launch,
        
        # Launch localization node
        # localization_node,
        
        # Launch safety parameter node
        safety_node,
        
        # Launch obstacle avoidance navigation node
        obstacle_nav_node,
        
        # Launch reset pose service
        # reset_pose_node,
        
        # Launch AMCL tuner
        # amcl_tuner_node,
    ])