#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf_transformations

class ResetPoseService(Node):
    def __init__(self):
        super().__init__('reset_pose_service')
        
        # Parameters
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('default_x', 0.0)
        self.declare_parameter('default_y', 0.0)
        self.declare_parameter('default_yaw', 0.0)
        
        # Get parameters
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.default_x = self.get_parameter('default_x').value
        self.default_y = self.get_parameter('default_y').value
        self.default_yaw = self.get_parameter('default_yaw').value
        
        # Initial pose publisher
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            'initialpose',
            10
        )
        
        # Reset pose service
        self.srv = self.create_service(
            Empty,
            'reset_pose',
            self.reset_pose_callback
        )
        
        # Initial pose reset on startup
        self.timer = self.create_timer(2.0, self.initial_reset)
        self.initial_reset_done = False
        
        self.get_logger().info('Reset Pose Service initialized')
    
    def initial_reset(self):
        """Perform an initial pose reset after startup."""
        if not self.initial_reset_done:
            self.publish_reset_pose()
            self.initial_reset_done = True
            self.timer.cancel()  # Cancel the timer after reset
            self.get_logger().info('Initial pose reset performed')
    
    def reset_pose_callback(self, request, response):
        """Handle requests to reset the robot's pose."""
        self.get_logger().info('Reset pose service called')
        self.publish_reset_pose()
        return response
    
    def publish_reset_pose(self):
        """Publish the reset pose to the initialpose topic."""
        # Create initial pose message
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        # Set position
        pose_msg.pose.pose.position.x = self.default_x
        pose_msg.pose.pose.position.y = self.default_y
        pose_msg.pose.pose.position.z = 0.0
        
        # Set orientation
        q = tf_transformations.quaternion_from_euler(0, 0, self.default_yaw)
        pose_msg.pose.pose.orientation.x = q[0]
        pose_msg.pose.pose.orientation.y = q[1]
        pose_msg.pose.pose.orientation.z = q[2]
        pose_msg.pose.pose.orientation.w = q[3]
        
        # Set covariance (small covariance = high certainty)
        covariance = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
        pose_msg.pose.covariance = covariance
        
        # Publish the pose
        self.initial_pose_pub.publish(pose_msg)
        self.get_logger().info(f'Published reset pose: x={self.default_x}, y={self.default_y}, yaw={self.default_yaw}')

def main(args=None):
    rclpy.init(args=args)
    node = ResetPoseService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()