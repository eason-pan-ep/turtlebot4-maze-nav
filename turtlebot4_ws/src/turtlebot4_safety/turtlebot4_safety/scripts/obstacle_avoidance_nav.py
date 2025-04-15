#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
from std_msgs.msg import Bool, Header
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import time
import math
import tf_transformations

class ObstacleAvoidanceNavigator(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_navigator')
        
        # Parameters
        self.declare_parameter('obstacle_threshold', 0.5)  # Distance in meters to consider as obstacle
        self.declare_parameter('scan_angle_min', -30.0)    # Minimum angle to check for obstacles (degrees)
        self.declare_parameter('scan_angle_max', 30.0)     # Maximum angle to check for obstacles (degrees)
        self.declare_parameter('inflation_radius', 0.20)   # Inflation radius for obstacles
        self.declare_parameter('footprint_padding', 0.05)  # Robot footprint padding
        self.declare_parameter('use_sim_time', True)       # Use simulation time
        
        # Get parameters
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        self.scan_angle_min = self.get_parameter('scan_angle_min').value
        self.scan_angle_max = self.get_parameter('scan_angle_max').value
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.footprint_padding = self.get_parameter('footprint_padding').value
        
        # Navigation action client
        self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # RViz goal subscription - subscribes to goals sent via RViz
        self.rviz_goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.rviz_goal_callback,
            10
        )
        
        # Path subscription to monitor planned paths
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )
        
        # Scan subscriber for obstacle detection
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        # Recovery trigger
        self.recovery_pub = self.create_publisher(Bool, 'recovery_trigger', 10)
        
        # Visualization marker publisher for detected obstacles
        self.marker_pub = self.create_publisher(MarkerArray, 'obstacle_markers', 10)
        
        # Obstacle state
        self.obstacles_detected = False
        self.last_obstacle_time = self.get_clock().now()
        self.obstacle_points = []  # List to store detected obstacle points
        
        # Pending goal
        self.current_goal = None
        self.goal_in_progress = False
        self.current_path = None
        
        # Timer for visualization updates
        self.create_timer(0.5, self.publish_visualization)
        
        self.get_logger().info('Obstacle Avoidance Navigator initialized - Ready for RViz goals')
        
    def rviz_goal_callback(self, goal_msg):
        """Handle goals sent from RViz"""
        self.get_logger().info(f'Received goal from RViz: position=({goal_msg.pose.position.x:.2f}, {goal_msg.pose.position.y:.2f})')
        
        # Store current goal
        self.current_goal = goal_msg
        
        # Send the goal
        self.send_goal()
    
    def path_callback(self, path_msg):
        """Monitor the planned navigation path"""
        if len(path_msg.poses) > 0:
            self.current_path = path_msg
            self.get_logger().debug(f'Received path with {len(path_msg.poses)} waypoints')
    
    def set_goal(self, x, y, theta=0.0):
        """Set a navigation goal for the robot (API for direct goal setting)."""
        self.get_logger().info(f'Setting goal to position ({x}, {y}, {theta})')
        
        # Create goal pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Set position
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        q = tf_transformations.quaternion_from_euler(0, 0, theta)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]
        
        # Store current goal
        self.current_goal = goal_pose
        
        # Send the goal
        self.send_goal()
        
    def send_goal(self):
        """Send the current goal to the navigation stack."""
        if self.current_goal is None:
            self.get_logger().warn('No goal to send!')
            return
            
        # Wait for action server
        if not self._nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return
            
        # Create and send goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.current_goal
        
        self.get_logger().info('Sending goal to navigation stack...')
        self.goal_in_progress = True
        
        # Also publish to topic for visualization
        self.goal_pub.publish(self.current_goal)
        
        # Send goal and register callbacks
        self._send_goal_future = self._nav_to_pose_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Callback when goal is accepted or rejected."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected!')
            self.goal_in_progress = False
            return
            
        self.get_logger().info('Goal accepted by server, waiting for result...')
        
        # Request the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Callback for when the navigation action is completed."""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # Succeeded
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().warn(f'Goal failed with status: {status}')
            
            # If we were stopped due to obstacles, we might want to try a recovery
            if self.obstacles_detected:
                self.get_logger().info('Obstacles detected, triggering recovery...')
                self.trigger_recovery()
        
        self.goal_in_progress = False
    
    def feedback_callback(self, feedback_msg):
        """Process feedback from the navigation stack."""
        feedback = feedback_msg.feedback
        # Can log progress, distance remaining, etc.
    
    def scan_callback(self, msg):
        """Process laser scan data for obstacle detection and tracking."""
        # Convert min/max angles from degrees to radians
        angle_min_rad = math.radians(self.scan_angle_min)
        angle_max_rad = math.radians(self.scan_angle_max)
        
        # Find indices corresponding to angle range
        angle_min_idx = max(0, int((angle_min_rad - msg.angle_min) / msg.angle_increment))
        angle_max_idx = min(len(msg.ranges) - 1, int((angle_max_rad - msg.angle_min) / msg.angle_increment))
        
        # Check for obstacles in the specified range
        obstacles_in_range = False
        min_distance = float('inf')
        
        # Clear previous obstacle points
        self.obstacle_points = []
        
        # Process all scan points
        for i in range(len(msg.ranges)):
            range_val = msg.ranges[i]
            # Filter invalid readings
            if range_val > msg.range_min and range_val < msg.range_max:
                # Calculate the angle for this reading
                angle = msg.angle_min + i * msg.angle_increment
                
                # Check if this is an obstacle (closer than threshold)
                if range_val < self.obstacle_threshold:
                    # Convert from polar to cartesian coordinates (relative to robot)
                    x = range_val * math.cos(angle)
                    y = range_val * math.sin(angle)
                    
                    # Store obstacle point
                    self.obstacle_points.append((x, y))
                    
                    # Update minimum distance
                    if i >= angle_min_idx and i <= angle_max_idx:
                        min_distance = min(min_distance, range_val)
                        obstacles_in_range = True
        
        # Update obstacle state
        if obstacles_in_range:
            if not self.obstacles_detected:
                self.get_logger().info(f'Obstacle detected! Min distance: {min_distance:.2f}m')
                
                # Adjust safety parameters when obstacles detected
                self.update_safety_parameters(True)
                
            self.obstacles_detected = True
            self.last_obstacle_time = self.get_clock().now()
        else:
            # Check if obstacles have been cleared for a while
            elapsed = (self.get_clock().now() - self.last_obstacle_time).nanoseconds / 1e9
            if self.obstacles_detected and elapsed > 2.0:  # 2 seconds without obstacles
                self.get_logger().info('Path clear of obstacles')
                self.obstacles_detected = False
                
                # Restore normal parameters
                self.update_safety_parameters(False)
    
    def publish_visualization(self):
        """Publish visualization markers for detected obstacles."""
        if not self.obstacle_points:
            return
            
        # Create marker array
        marker_array = MarkerArray()
        
        # Create a marker for each obstacle point
        for i, (x, y) in enumerate(self.obstacle_points):
            marker = Marker()
            marker.header.frame_id = 'base_link'  # Relative to robot
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'obstacles'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Set position
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.1  # Slightly above ground
            
            # Set orientation (identity quaternion)
            marker.pose.orientation.w = 1.0
            
            # Set scale
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            # Set color (red for obstacles)
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 0.8  # Mostly opaque
            
            # How long to show the marker
            marker.lifetime.sec = 1
            
            # Add to array
            marker_array.markers.append(marker)
        
        # Publish marker array
        self.marker_pub.publish(marker_array)
    
    def update_safety_parameters(self, obstacles_present):
        """Update navigation safety parameters based on obstacle presence."""
        # When obstacles detected, increase inflation radius and footprint padding
        if obstacles_present:
            new_inflation_radius = 0.25  # Increased radius for more caution
            new_footprint_padding = 0.10  # Increased padding
        else:
            new_inflation_radius = self.inflation_radius  # Original value
            new_footprint_padding = self.footprint_padding  # Original value
        
        # Update costmap parameters
        self.update_costmap_params('local_costmap', new_inflation_radius, new_footprint_padding)
        self.update_costmap_params('global_costmap', new_inflation_radius, new_footprint_padding)
    
    def update_costmap_params(self, costmap_name, inflation_radius, footprint_padding):
        """Update parameters for a specific costmap."""
        client = self.create_client(
            SetParameters, 
            f'/{costmap_name}/{costmap_name}/set_parameters'
        )
        
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'{costmap_name} service not available')
            return
        
        # Create parameter list
        params = [
            Parameter(
                name='inflation_layer.inflation_radius',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=inflation_radius)
            ),
            Parameter(
                name='footprint_padding',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=footprint_padding)
            )
        ]
        
        # Send request
        request = SetParameters.Request()
        request.parameters = params
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self.parameter_callback(f, costmap_name)
        )
    
    def parameter_callback(self, future, name):
        """Handle the response from parameter update requests."""
        try:
            response = future.result()
            success = True
            
            for result in response.results:
                if not result.successful:
                    success = False
                    self.get_logger().error(f'Failed to set {name} parameter: {result.reason}')
            
            if success:
                self.get_logger().info(f'Successfully updated {name} parameters')
                    
        except Exception as e:
            self.get_logger().error(f'Service call failed for {name}: {str(e)}')
    
    def trigger_recovery(self):
        """Trigger a recovery behavior when stuck."""
        recovery_msg = Bool()
        recovery_msg.data = True
        self.recovery_pub.publish(recovery_msg)
        
        # Could add specific recovery behaviors here
        # For example, clear costmaps, rotate in place, etc.
        
        # After recovery, we could retry the navigation
        if self.current_goal is not None and not self.goal_in_progress:
            self.get_logger().info('Retrying navigation after recovery...')
            self.send_goal()

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNavigator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()