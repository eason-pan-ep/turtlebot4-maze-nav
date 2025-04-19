#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get directory paths
    turtlebot4_ignition_bringup_dir = get_package_share_directory('turtlebot4_ignition_bringup')
    turtlebot4_safety_dir = get_package_share_directory('turtlebot4_safety')
    
    # Map file path - using empty map that has walls but no obstacles
    map_dir = os.path.join(turtlebot4_safety_dir, 'maps')
    map_yaml_file = os.path.join(map_dir, 'lab211_empty.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Include the TurtleBot4 Ignition simulator launch with SLAM enabled
    turtlebot4_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot4_ignition_bringup_dir, 'launch', 'turtlebot4_ignition.launch.py')
        ]),
        launch_arguments={
            'world': 'lab211_with_boxes_1',  # custom world
            'model': 'lite',                 # Use the LITE model
            'rviz': 'true',                  # Launch with RViz
            'nav2': 'true',                  # Launch with Nav2
            'slam': 'true',                  # Enable SLAM
            'localization': 'false',         # Disable AMCL localization
            'map': map_yaml_file,           # Use empty map as initial map
        }.items()
    )
    
    # Load the map server separately to initialize SLAM with the prior map
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='prior_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file,
        }]
    )
    
    # Safety parameter node for better obstacle avoidance
    safety_params_node = Node(
        package='turtlebot4_safety',
        executable='safety_params',
        name='safety_parameter_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'inflation_radius': 0.20,       # Increased for better obstacle avoidance
            'obstacle_range': 3.5,
            'min_approach_distance': 0.10,  # Increased for safer navigation
            'max_vel_x': 0.15,
            'footprint_padding': 0.08,      # Increased for safer navigation
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
    
    # Launch args declaration
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time if true'
    )
    
    return LaunchDescription([
        # Declare launch args
        declare_use_sim_time,
        
        # Launch TurtleBot4 Ignition simulation with SLAM
        turtlebot4_sim_launch,
        
        # Launch the map server with prior map
        map_server_node,
        
        # Launch safety parameter node
        safety_params_node,
        
        # Launch obstacle avoidance navigation node
        obstacle_nav_node,
    ])