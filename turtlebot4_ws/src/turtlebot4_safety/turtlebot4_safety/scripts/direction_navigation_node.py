#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
import math
import tf2_ros
from tf2_ros import Buffer, TransformListener

class DirectionNavigationNode(Node):
    def __init__(self):
        super().__init__('direction_navigation_node')
        
        # Subscribe to direction commands
        self.direction_sub = self.create_subscription(
            String,
            'marker_direction',
            self.direction_callback,
            10
        )
        
        # Set up Nav2 Action Client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # TF buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Parameters
        self.declare_parameter('turn_angle', 1.5708)  # Default: 90 degrees in radians
        self.declare_parameter('forward_distance', 2.0)  # Meters to move forward
        
        self.turn_angle = self.get_parameter('turn_angle').value
        self.forward_distance = self.get_parameter('forward_distance').value
        
        # Keep track of the last command received to avoid repeated execution
        self.last_command = None
        self.command_timestamp = None
        
        self.get_logger().info('Direction navigation node initialized')
    
    def direction_callback(self, msg):
        """Process direction commands from markers"""
        current_time = self.get_clock().now()
        
        # Check if this is a repeated command (debounce)
        if (self.last_command == msg.data and 
            self.command_timestamp is not None and
            (current_time - self.command_timestamp).nanoseconds / 1e9 < 5.0):  # 5 second debounce
            return
        
        self.last_command = msg.data
        self.command_timestamp = current_time
        
        self.get_logger().info(f'Executing direction command: {msg.data}')
        
        if msg.data in ['go_left', 'go_right']:
            # Get current robot pose in map frame
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                current_pose = PoseStamped()
                current_pose.header.frame_id = 'map'
                current_pose.header.stamp = self.get_clock().now().to_msg()
                current_pose.pose.position.x = transform.transform.translation.x
                current_pose.pose.position.y = transform.transform.translation.y
                current_pose.pose.position.z = 0.0
                current_pose.pose.orientation = transform.transform.rotation
                
                # Calculate target pose based on direction
                target_pose = self.calculate_target_pose(current_pose, msg.data)
                
                # Send navigation goal
                self.send_navigation_goal(target_pose)
                
            except Exception as e:
                self.get_logger().error(f'Error getting robot pose: {str(e)}')
    
    def calculate_target_pose(self, current_pose, direction):
        """Calculate target pose based on direction command"""
        # Extract yaw from current orientation
        qx = current_pose.pose.orientation.x
        qy = current_pose.pose.orientation.y
        qz = current_pose.pose.orientation.z
        qw = current_pose.pose.orientation.w
        
        # Convert quaternion to euler angles
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Calculate new yaw based on direction
        new_yaw = yaw
        if direction == 'go_left':
            new_yaw += self.turn_angle
        elif direction == 'go_right':
            new_yaw -= self.turn_angle
        
        # Calculate target position (forward after turn)
        target_pose = PoseStamped()
        target_pose.header = current_pose.header
        target_pose.pose.position.x = current_pose.pose.position.x + self.forward_distance * math.cos(new_yaw)
        target_pose.pose.position.y = current_pose.pose.position.y + self.forward_distance * math.sin(new_yaw)
        target_pose.pose.position.z = 0.0
        
        # Set orientation
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = math.sin(new_yaw / 2.0)
        target_pose.pose.orientation.w = math.cos(new_yaw / 2.0)
        
        return target_pose
    
    def send_navigation_goal(self, pose):
        """Send navigation goal to Nav2"""
        self.get_logger().info('Sending navigation goal')
        
        # Wait for action server
        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return
        
        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        
        # Send goal
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Handle response from Nav2 action server"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            return
        
        self.get_logger().info('Navigation goal accepted')
        
        # Get result future
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """Handle goal result"""
        result = future.result().result
        self.get_logger().info('Navigation goal completed')

def main(args=None):
    rclpy.init(args=args)
    node = DirectionNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()