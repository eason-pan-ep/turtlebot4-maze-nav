#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import cv2

def generate_launch_description():
    # Get directory paths
    turtlebot4_ignition_bringup_dir = get_package_share_directory('turtlebot4_ignition_bringup')
    turtlebot4_safety_dir = get_package_share_directory('turtlebot4_safety')
    
    # Map file path
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
            'world': 'lab211_with_boxes_fiducial_test',  # Custom world with fiducial markers
            'model': 'lite',                   # Use the LITE model
            'rviz': 'true',                    # Launch with RViz
            'nav2': 'true',                    # Launch with Nav2
            'slam': 'true',                    # Enable SLAM
            'localization': 'false',           # Disable AMCL localization
            'map': map_yaml_file,              # Use empty map as initial map
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
            'marker_size': 0.2,
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
            'turn_angle': 1.5708,          # 90 degrees in radians
            'forward_distance': 2.0,        # 2 meters
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
            'use_sim_time': True,
            'inflation_radius': 0.20,
            'obstacle_range': 3.5,
            'min_approach_distance': 0.10,
            'max_vel_x': 0.15,
            'footprint_padding': 0.08,
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
        
        # Launch TF buffer server
        tf_buffer_server,
        
        # Launch fiducial detection node
        fiducial_detection_node,
        
        # Launch direction navigation node
        direction_navigation_node,
        
        # Launch safety parameter node
        safety_params_node,
    ])