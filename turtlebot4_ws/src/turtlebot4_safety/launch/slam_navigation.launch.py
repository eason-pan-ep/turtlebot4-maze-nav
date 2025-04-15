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
    
    # Include the TurtleBot4 Ignition simulator launch with SLAM enabled
    turtlebot4_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot4_ignition_bringup_dir, 'launch', 'turtlebot4_ignition.launch.py')
        ]),
        launch_arguments={
            'world': 'lab211_with_boxes_1',  # the lab211 world with boxes
            'model': 'lite',                 # Use the LITE model
            'rviz': 'true',                  # Launch with RViz
            'nav2': 'true',                  # Launch with Nav2
            'slam': 'true',                  # Enable SLAM
            'localization': 'false',         # Disable AMCL localization (use SLAM instead)
        }.items()
    )
    
    # Safety parameter node for better obstacle avoidance
    safety_params_node = Node(
        package='turtlebot4_safety',
        executable='safety_params',
        name='safety_parameter_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'inflation_radius': 0.20,       # Increased from 0.15 for better obstacle avoidance
            'obstacle_range': 3.5,
            'min_approach_distance': 0.10,  # Increased from 0.05 for safer navigation
            'max_vel_x': 0.15,
            'footprint_padding': 0.08,      # Increased from 0.05 for safer navigation
        }]
    )
    
    return LaunchDescription([
        # Launch TurtleBot4 Ignition simulation with SLAM
        turtlebot4_sim_launch,
        
        # Launch safety parameter node
        safety_params_node,
    ])