#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
import numpy as np
import tf2_ros
import math
from tf2_ros import TransformBroadcaster

class FiducialDetectionNode(Node):
    def __init__(self):
        super().__init__('fiducial_detection_node')
        
        # Parameters
        self.declare_parameter('marker_size', 0.2)  # Size in meters
        self.declare_parameter('dictionary_id', 5)  # DICT_5X5_100
        
        self.marker_size = self.get_parameter('marker_size').value
        dict_id = self.get_parameter('dictionary_id').value
        
        # Create ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Map marker IDs to direction commands
        self.marker_commands = {
            10: "go_left",
            20: "go_right"
        }
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image, 
            'oakd/rgb/preview/image_raw', 
            self.image_callback, 
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            'oakd/rgb/preview/camera_info',
            self.camera_info_callback,
            10
        )
        
        # Publishers
        self.direction_pub = self.create_publisher(String, 'marker_direction', 10)
        self.debug_image_pub = self.create_publisher(Image, 'debug_image', 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # CV bridge
        self.bridge = CvBridge()
        
        # Camera calibration parameters
        self.camera_matrix = None
        self.dist_coeffs = None
        
        self.get_logger().info('Fiducial detection node initialized')
    
    def camera_info_callback(self, msg):
        """Process camera calibration information"""
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info('Camera calibration received')
    
    def image_callback(self, msg):
        """Process incoming camera images"""
        if self.camera_matrix is None:
            return
        
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to grayscale for ArUco detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect markers
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, 
                                                           parameters=self.aruco_params)
            
            # Draw detection results on the image
            debug_image = cv_image.copy()
            cv2.aruco.drawDetectedMarkers(debug_image, corners, ids)
            
            if ids is not None and len(ids) > 0:
                # Process each detected marker
                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    
                    # Estimate pose of the marker
                    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners[i], self.marker_size, self.camera_matrix, self.dist_coeffs
                    )
                    
                    # Draw axes on the marker
                    cv2.drawFrameAxes(debug_image, self.camera_matrix, self.dist_coeffs, 
                                     rvecs[0], tvecs[0], self.marker_size/2)
                    
                    # Broadcast marker transform
                    self.broadcast_marker_tf(marker_id, rvecs[0], tvecs[0], msg.header)
                    
                    # Check if this marker has a command
                    if marker_id in self.marker_commands:
                        # Publish command
                        direction_msg = String()
                        direction_msg.data = self.marker_commands[marker_id]
                        self.direction_pub.publish(direction_msg)
                        
                        # Add text to debug image
                        position = (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 10)
                        cv2.putText(debug_image, self.marker_commands[marker_id], 
                                   position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        self.get_logger().info(f'Detected marker {marker_id}: {self.marker_commands[marker_id]}')
            
            # Publish debug image
            self.get_logger().info("Publishing debug image")
            self.debug_image_pub.publish(self.bridge.cv2_to_imgmsg(debug_image, encoding='bgr8'))
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')
    
    def broadcast_marker_tf(self, marker_id, rvec, tvec, header):
        """Broadcast the marker's transform"""
        transform = TransformStamped()
        transform.header.stamp = header.stamp
        transform.header.frame_id = header.frame_id
        transform.child_frame_id = f'marker_{marker_id}'
        
        # Fill in translation
        transform.transform.translation.x = tvec[0]
        transform.transform.translation.y = tvec[1]
        transform.transform.translation.z = tvec[2]
        
        # Convert rotation vector to quaternion
        rot_matrix = np.eye(4)
        rot_matrix[:3, :3], _ = cv2.Rodrigues(rvec)
        
        # Get quaternion from rotation matrix
        q = self.rotation_matrix_to_quaternion(rot_matrix[:3, :3])
        
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(transform)
    
    def rotation_matrix_to_quaternion(self, rotation_matrix):
        """Convert a rotation matrix to quaternion"""
        q = np.zeros(4)
        
        trace = rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) * s
            q[1] = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) * s
            q[2] = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) * s
        else:
            if rotation_matrix[0, 0] > rotation_matrix[1, 1] and rotation_matrix[0, 0] > rotation_matrix[2, 2]:
                s = 2.0 * np.sqrt(1.0 + rotation_matrix[0, 0] - rotation_matrix[1, 1] - rotation_matrix[2, 2])
                q[3] = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                q[2] = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
            elif rotation_matrix[1, 1] > rotation_matrix[2, 2]:
                s = 2.0 * np.sqrt(1.0 + rotation_matrix[1, 1] - rotation_matrix[0, 0] - rotation_matrix[2, 2])
                q[3] = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / s
                q[0] = (rotation_matrix[0, 1] + rotation_matrix[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + rotation_matrix[2, 2] - rotation_matrix[0, 0] - rotation_matrix[1, 1])
                q[3] = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / s
                q[0] = (rotation_matrix[0, 2] + rotation_matrix[2, 0]) / s
                q[1] = (rotation_matrix[1, 2] + rotation_matrix[2, 1]) / s
                q[2] = 0.25 * s
        
        return q

def main(args=None):
    rclpy.init(args=args)
    node = FiducialDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()