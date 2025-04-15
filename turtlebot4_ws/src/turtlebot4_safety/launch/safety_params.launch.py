from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    # Launch safety parameter node with a short delay to make sure navigation is ready
    safety_node = TimerAction(
        period=3.0,  # Shorter delay since we're launching this after the simulator
        actions=[
            Node(
                package='turtlebot4_safety',
                executable='safety_params',
                name='safety_parameter_node',
                output='screen',
                parameters=[{
                    'inflation_radius': 0.10,  # how much space the nav stack should keep around obstacles
                    'obstacle_range': 3.5, # how far the robot should look for obstacles
                    'min_approach_distance': 0.05, # how close the robot can get to an obstacle
                    'max_vel_x': 0.15, # max forward speed
                    'footprint_padding': 0.05,   # Minimal padding around the robot footprint
                }]
            )
        ]
    )
    
    return LaunchDescription([
        safety_node
    ])