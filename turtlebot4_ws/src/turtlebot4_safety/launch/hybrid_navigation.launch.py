#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get directory paths
    turtlebot4_ignition_bringup_dir = get_package_share_directory('turtlebot4_ignition_bringup')
    turtlebot4_safety_dir = get_package_share_directory('turtlebot4_safety')
    
    # Map file path - using your empty map that has walls but no obstacles
    map_dir = os.path.join(turtlebot4_safety_dir, 'maps')
    map_yaml_file = os.path.join(map_dir, 'lab211_empty.yaml')
    
    # Include the TurtleBot4 Ignition simulator launch with both SLAM and localization
    turtlebot4_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot4_ignition_bringup_dir, 'launch', 'turtlebot4_ignition.launch.py')
        ]),
        launch_arguments={
            'world': 'lab211_with_boxes_1',  # Your custom world
            'model': 'lite',                 # Use the LITE model
            'rviz': 'true',                  # Launch with RViz
            'nav2': 'true',                  # Launch with Nav2
            'slam': 'true',                  # Enable SLAM
            'localization': 'true',          # Enable AMCL localization
            'map': map_yaml_file,           # Use empty map for localization
        }.items()
    )
    
    # Safety parameter node
    safety_params_node = Node(
        package='turtlebot4_safety',
        executable='safety_params',
        name='safety_parameter_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'inflation_radius': 0.20,
            'obstacle_range': 3.5,
            'min_approach_distance': 0.10,
            'max_vel_x': 0.15,
            'footprint_padding': 0.08,
        }]
    )
    
    # Obstacle avoidance navigation node
    obstacle_nav_node = Node(
        package='turtlebot4_safety',
        executable='obstacle_avoidance_nav',
        name='obstacle_avoidance_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'obstacle_threshold': 0.5,
            'scan_angle_min': -60.0,
            'scan_angle_max': 60.0,
            'inflation_radius': 0.20,
            'footprint_padding': 0.08,
        }]
    )
    
    return LaunchDescription([
        # Launch TurtleBot4 Ignition with both SLAM and localization
        turtlebot4_sim_launch,
        
        # Launch safety parameter node
        safety_params_node,
        
        # Launch obstacle avoidance navigation node
        obstacle_nav_node,
    ])