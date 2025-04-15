#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class RobotLocalizationNode(Node):
    def __init__(self):
        super().__init__('robot_localization_node')
        
        # Parameters
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('frequency', 30.0)
        self.declare_parameter('publish_tf', True)
        
        # Get parameters
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.frequency = self.get_parameter('frequency').value
        self.publish_tf = self.get_parameter('publish_tf').value
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10
        )
        
        # Publishers
        self.filtered_odom_pub = self.create_publisher(Odometry, 'filtered_odom', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'filtered_pose', 10)
        
        # Timer for regular processing
        self.timer = self.create_timer(1.0/self.frequency, self.filter_callback)
        
        # State variables
        self.latest_odom = None
        self.latest_imu = None
        self.filtered_x = 0.0
        self.filtered_y = 0.0
        self.filtered_yaw = 0.0
        
        # Simple covariance for the filter
        self.pose_covariance = np.diag([0.1, 0.1, 0.0, 0.0, 0.0, 0.1]).flatten()
        
        self.get_logger().info('Robot Localization Node initialized')
    
    def odom_callback(self, msg):
        """Process odometry data."""
        self.latest_odom = msg
    
    def imu_callback(self, msg):
        """Process IMU data."""
        self.latest_imu = msg
    
    def filter_callback(self):
        """Run the filter process at regular intervals."""
        if self.latest_odom is None:
            return
            
        # Get current odometry position and orientation
        odom_x = self.latest_odom.pose.pose.position.x
        odom_y = self.latest_odom.pose.pose.position.y
        
        q = self.latest_odom.pose.pose.orientation
        _, _, odom_yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        # If we have IMU data, use it for orientation
        imu_yaw = None
        if self.latest_imu is not None:
            q_imu = self.latest_imu.orientation
            _, _, imu_yaw = tf_transformations.euler_from_quaternion([q_imu.x, q_imu.y, q_imu.z, q_imu.w])
        
        # Simple complementary filter for position and orientation
        if self.filtered_x == 0.0 and self.filtered_y == 0.0:
            # Initialize filter with first measurement
            self.filtered_x = odom_x
            self.filtered_y = odom_y
            self.filtered_yaw = odom_yaw if imu_yaw is None else imu_yaw
        else:
            # Apply the filter
            alpha = 0.7  # Weight for odometry (0.7 means 70% from odometry, 30% from previous estimate)
            
            self.filtered_x = alpha * odom_x + (1 - alpha) * self.filtered_x
            self.filtered_y = alpha * odom_y + (1 - alpha) * self.filtered_y
            
            # For yaw, prefer IMU data if available
            if imu_yaw is not None:
                beta = 0.8  # Weight for IMU (0.8 means 80% from IMU, 20% from previous estimate)
                self.filtered_yaw = beta * imu_yaw + (1 - beta) * self.filtered_yaw
            else:
                self.filtered_yaw = alpha * odom_yaw + (1 - alpha) * self.filtered_yaw
        
        # Publish filtered odometry
        self.publish_filtered_odometry()
        
        # Publish tf if enabled
        if self.publish_tf:
            self.publish_transform()
    
    def publish_filtered_odometry(self):
        """Publish the filtered odometry and pose."""
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Set position
        odom_msg.pose.pose.position.x = self.filtered_x
        odom_msg.pose.pose.position.y = self.filtered_y
        odom_msg.pose.pose.position.z = 0.0
        
        # Set orientation
        q = tf_transformations.quaternion_from_euler(0, 0, self.filtered_yaw)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        
        # Set covariance
        odom_msg.pose.covariance = self.pose_covariance
        
        # Publish odometry
        self.filtered_odom_pub.publish(odom_msg)
        
        # Create and publish pose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose = odom_msg.pose
        self.pose_pub.publish(pose_msg)
    
    def publish_transform(self):
        """Publish the transform from odom to base_link."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Set translation
        t.transform.translation.x = self.filtered_x
        t.transform.translation.y = self.filtered_y
        t.transform.translation.z = 0.0
        
        # Set rotation
        q = tf_transformations.quaternion_from_euler(0, 0, self.filtered_yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = RobotLocalizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()