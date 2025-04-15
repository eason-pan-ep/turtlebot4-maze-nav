#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
import math
import time
from std_srvs.srv import Trigger

class AMCLTuner(Node):
    def __init__(self):
        super().__init__('amcl_tuner')
        
        # Parameters
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('check_interval', 5.0)  # Seconds between checks
        self.declare_parameter('drift_threshold', 0.5)  # Meters of drift to trigger adjustment
        self.declare_parameter('rotation_drift_threshold', 0.3)  # Radians of rotation drift
        
        # Get parameters
        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.check_interval = self.get_parameter('check_interval').value
        self.drift_threshold = self.get_parameter('drift_threshold').value
        self.rotation_drift_threshold = self.get_parameter('rotation_drift_threshold').value
        
        # TF buffer for transform checking
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Odometry subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        
        # Create clients for AMCL parameter services
        self.amcl_get_params_client = self.create_client(
            GetParameters,
            'amcl/get_parameters'
        )
        
        self.amcl_set_params_client = self.create_client(
            SetParameters,
            'amcl/set_parameters'
        )
        
        # Client for clearing costmaps
        self.clear_costmaps_client = self.create_client(
            Trigger,
            'local_costmap/clear_entirely_local_costmap'
        )
        
        # Create timer for regular checks
        self.timer = self.create_timer(self.check_interval, self.check_localization)
        
        # Track odometry drift
        self.last_odom_x = None
        self.last_odom_y = None
        self.last_odom_yaw = None
        self.odom_distance = 0.0
        self.last_check_time = self.get_clock().now()
        
        # Current AMCL parameters
        self.current_min_particles = 500
        self.current_recovery_alpha_slow = 0.001
        self.current_recovery_alpha_fast = 0.1
        
        self.get_logger().info('AMCL Tuner initialized')
        
        # Wait for services
        self.wait_for_services()
    
    def wait_for_services(self):
        """Wait for all required services to be available."""
        services = [
            (self.amcl_get_params_client, 'amcl/get_parameters'),
            (self.amcl_set_params_client, 'amcl/set_parameters'),
            (self.clear_costmaps_client, 'local_costmap/clear_entirely_local_costmap')
        ]
        
        for client, service_name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f'Waiting for {service_name} service...')
        
        self.get_logger().info('All services are available')
    
    def odom_callback(self, msg):
        """Process odometry data to track drift."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Extract yaw from quaternion
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Calculate yaw (simplified)
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        
        # Initialize on first callback
        if self.last_odom_x is None:
            self.last_odom_x = x
            self.last_odom_y = y
            self.last_odom_yaw = yaw
            return
        
        # Calculate distance traveled since last callback
        dx = x - self.last_odom_x
        dy = y - self.last_odom_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Accumulate distance traveled
        self.odom_distance += distance
        
        # Update last position
        self.last_odom_x = x
        self.last_odom_y = y
        self.last_odom_yaw = yaw
    
    def check_localization(self):
        """Check localization quality and adjust parameters if needed."""
        # Check if enough time has passed
        now = self.get_clock().now()
        if (now - self.last_check_time).nanoseconds / 1e9 < self.check_interval:
            return
        
        self.last_check_time = now
        
        try:
            # Check transform from map to base_link (success indicates good localization)
            # If transform fails, we may have poor localization
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                transform_age = (now - rclpy.time.Time.from_msg(transform.header.stamp)).nanoseconds / 1e9
                
                if transform_age > 1.0:  # Transform is old
                    self.get_logger().warn(f'Transform is old: {transform_age:.2f} seconds')
                    self.adjust_amcl_parameters(True)
                else:
                    # Check if we've moved enough to evaluate localization
                    if self.odom_distance > 0.1:  # Moved at least 10cm
                        self.get_logger().info(f'Robot has moved {self.odom_distance:.2f}m since last check')
                        self.adjust_amcl_parameters(False)  # Normal adjustment
                        self.odom_distance = 0.0  # Reset distance counter
            
            except Exception as e:
                self.get_logger().error(f'Transform lookup failed: {str(e)}')
                self.adjust_amcl_parameters(True)  # Error indicates poor localization
                
        except Exception as e:
            self.get_logger().error(f'Error in check_localization: {str(e)}')
    
    def adjust_amcl_parameters(self, aggressive=False):
        """Adjust AMCL parameters based on localization quality."""
        try:
            # Get current parameters
            self.get_current_parameters()
            
            # Determine new parameters based on localization quality
            if aggressive:
                # For poor localization, make more aggressive adjustments
                new_min_particles = min(5000, self.current_min_particles * 2)
                new_recovery_alpha_slow = min(0.01, self.current_recovery_alpha_slow * 5)
                new_recovery_alpha_fast = min(0.5, self.current_recovery_alpha_fast * 2)
                
                self.get_logger().warn('Applying aggressive AMCL parameter adjustments')
                
                # Also clear costmaps when localization is poor
                self.clear_costmaps()
            else:
                # For good localization, make small adjustments or revert to defaults
                new_min_particles = max(500, self.current_min_particles)
                new_recovery_alpha_slow = 0.001  # Default
                new_recovery_alpha_fast = 0.1    # Default
                
                self.get_logger().info('Localization seems good, maintaining standard parameters')
            
            # Only update if parameters changed
            if (new_min_particles != self.current_min_particles or
                new_recovery_alpha_slow != self.current_recovery_alpha_slow or
                new_recovery_alpha_fast != self.current_recovery_alpha_fast):
                
                self.update_parameters(
                    new_min_particles,
                    new_recovery_alpha_slow,
                    new_recovery_alpha_fast
                )
        
        except Exception as e:
            self.get_logger().error(f'Error adjusting parameters: {str(e)}')
    
    def get_current_parameters(self):
        """Get current AMCL parameters."""
        try:
            request = GetParameters.Request()
            request.names = ['min_particles', 'recovery_alpha_slow', 'recovery_alpha_fast']
            
            future = self.amcl_get_params_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.result() is not None:
                response = future.result()
                
                # Extract parameter values
                for i, name in enumerate(request.names):
                    value = response.values[i]
                    if name == 'min_particles':
                        self.current_min_particles = value.integer_value
                    elif name == 'recovery_alpha_slow':
                        self.current_recovery_alpha_slow = value.double_value
                    elif name == 'recovery_alpha_fast':
                        self.current_recovery_alpha_fast = value.double_value
                
                self.get_logger().info(f'Current AMCL parameters: min_particles={self.current_min_particles}, '
                                     f'recovery_alpha_slow={self.current_recovery_alpha_slow}, '
                                     f'recovery_alpha_fast={self.current_recovery_alpha_fast}')
            else:
                self.get_logger().warn('Failed to get current AMCL parameters')
        
        except Exception as e:
            self.get_logger().error(f'Error getting parameters: {str(e)}')
    
    def update_parameters(self, min_particles, recovery_alpha_slow, recovery_alpha_fast):
        """Update AMCL parameters."""
        try:
            request = SetParameters.Request()
            
            # Create parameter messages
            min_particles_param = ParameterMsg()
            min_particles_param.name = 'min_particles'
            min_particles_param.value.type = ParameterType.PARAMETER_INTEGER
            min_particles_param.value.integer_value = min_particles
            
            recovery_alpha_slow_param = ParameterMsg()
            recovery_alpha_slow_param.name = 'recovery_alpha_slow'
            recovery_alpha_slow_param.value.type = ParameterType.PARAMETER_DOUBLE
            recovery_alpha_slow_param.value.double_value = recovery_alpha_slow
            
            recovery_alpha_fast_param = ParameterMsg()
            recovery_alpha_fast_param.name = 'recovery_alpha_fast'
            recovery_alpha_fast_param.value.type = ParameterType.PARAMETER_DOUBLE
            recovery_alpha_fast_param.value.double_value = recovery_alpha_fast
            
            # Add parameters to request
            request.parameters = [min_particles_param, recovery_alpha_slow_param, recovery_alpha_fast_param]
            
            # Send request
            future = self.amcl_set_params_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.result() is not None:
                response = future.result()
                success = True
                
                for result in response.results:
                    if not result.successful:
                        success = False
                        self.get_logger().error(f'Failed to set parameter: {result.reason}')
                
                if success:
                    self.current_min_particles = min_particles
                    self.current_recovery_alpha_slow = recovery_alpha_slow
                    self.current_recovery_alpha_fast = recovery_alpha_fast
                    
                    self.get_logger().info(f'Updated AMCL parameters: min_particles={min_particles}, '
                                         f'recovery_alpha_slow={recovery_alpha_slow}, '
                                         f'recovery_alpha_fast={recovery_alpha_fast}')
            else:
                self.get_logger().warn('Failed to update AMCL parameters')
        
        except Exception as e:
            self.get_logger().error(f'Error updating parameters: {str(e)}')
    
    def clear_costmaps(self):
        """Clear the costmaps when localization issues are detected."""
        try:
            request = Trigger.Request()
            
            future = self.clear_costmaps_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.get_logger().info('Successfully cleared costmaps')
                else:
                    self.get_logger().warn(f'Failed to clear costmaps: {response.message}')
            else:
                self.get_logger().warn('Failed to call clear costmaps service')
        
        except Exception as e:
            self.get_logger().error(f'Error clearing costmaps: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = AMCLTuner()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()