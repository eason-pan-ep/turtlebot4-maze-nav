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
    turtlebot4_bringup_dir = get_package_share_directory('turtlebot4_bringup')
    turtlebot4_navigation_dir = get_package_share_directory('turtlebot4_navigation')
    turtlebot4_safety_dir = get_package_share_directory('turtlebot4_safety')
    turtlebot4_viz_dir = get_package_share_directory('turtlebot4_viz')
    
    # Map file path - using empty map that has walls but no obstacles
    map_dir = os.path.join(turtlebot4_safety_dir, 'maps')
    map_yaml_file = os.path.join(map_dir, 'lab211_empty.yaml')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Include the TurtleBot4 physical robot bringup
    turtlebot4_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot4_bringup_dir, 'launch', 'robot.launch.py')
        ])
    )
    
    # Include SLAM toolbox for real robot
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot4_navigation_dir, 'launch', 'slam.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'false',
        }.items()
    )
    
    # Include Nav2 for real robot
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot4_navigation_dir, 'launch', 'nav2.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'false',
            'map': map_yaml_file,
        }.items()
    )
    
    # Include RViz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(turtlebot4_viz_dir, 'launch', 'view_robot.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': 'false',
        }.items()
    )
    
    # Load the map server to initialize SLAM with the prior map
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='prior_map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
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
            'use_sim_time': False,
            'inflation_radius': 0.20,
            'obstacle_range': 3.5,
            'min_approach_distance': 0.15,
            'max_vel_x': 0.12,
            'footprint_padding': 0.10,
        }]
    )
    
    # Obstacle avoidance navigation node
    obstacle_nav_node = Node(
        package='turtlebot4_safety',
        executable='obstacle_avoidance_nav',
        name='obstacle_avoidance_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'obstacle_threshold': 0.55,
            'scan_angle_min': -60.0,
            'scan_angle_max': 60.0,
            'inflation_radius': 0.20,
            'footprint_padding': 0.10,
        }]
    )
    
    # Launch args declaration
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )
    
    return LaunchDescription([
        # Declare launch args
        declare_use_sim_time,
        
        # Launch TurtleBot4 robot bringup
        turtlebot4_robot_launch,
        
        # Launch SLAM
        slam_launch,
        
        # Launch Nav2
        nav2_launch,
        
        # Launch RViz
        rviz_launch,
        
        # Launch the map server with prior map
        map_server_node,
        
        # Launch safety parameter node
        safety_params_node,
        
        # Launch obstacle avoidance navigation node
        obstacle_nav_node,
    ])