#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rcl_interfaces.srv import SetParameters
import time

class SafetyParameterNode(Node):
    def __init__(self):
        super().__init__('safety_parameter_node')
        
        # Allow user to configure parameters via ROS params
        self.declare_parameter('inflation_radius', 0.10)
        self.declare_parameter('footprint_padding', 0.01)
        
        # Get parameter values from node configuration
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.footprint_padding = self.get_parameter('footprint_padding').value
        
        self.get_logger().info(f'Starting parameter configuration...')
        self.get_logger().info(f'Using inflation_radius: {self.inflation_radius}')
        self.get_logger().info(f'Using footprint_padding: {self.footprint_padding}')
        
        # Wait a moment for navigation to start
        time.sleep(5.0)
        
        # Update parameters regularly
        self.create_timer(10.0, self.update_parameters)
        self.update_parameters()
        
    def update_parameters(self):
        """Attempt to update all navigation parameters related to safety"""
        self.get_logger().info('Attempting to update safety parameters...')
        
        # Update the basic costmap parameters
        self.update_local_costmap_params()
        self.update_global_costmap_params()
        
        # Also try to update the footprint directly
        self.update_footprints()
    
    def update_local_costmap_params(self):
        """Update local costmap parameters for obstacle avoidance"""
        local_costmap_client = self.create_client(
            SetParameters, 
            '/local_costmap/local_costmap/set_parameters'
        )
        
        if not local_costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Local costmap service not available yet')
            return
        
        # Create parameter list - just focus on inflation radius
        params = [
            Parameter(
                name='inflation_layer.inflation_radius',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=self.inflation_radius)
            ),
            Parameter(
                name='footprint_padding',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=self.footprint_padding)
            )
        ]
        
        # Send request
        request = SetParameters.Request()
        request.parameters = params
        future = local_costmap_client.call_async(request)
        future.add_done_callback(
            lambda f: self.parameter_callback(f, 'local_costmap')
        )
    
    def update_global_costmap_params(self):
        """Update global costmap parameters"""
        global_costmap_client = self.create_client(
            SetParameters, 
            '/global_costmap/global_costmap/set_parameters'
        )
        
        if not global_costmap_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Global costmap service not available yet')
            return
        
        # Create parameter list - just focus on inflation radius
        params = [
            Parameter(
                name='inflation_layer.inflation_radius',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=self.inflation_radius)
            ),
            Parameter(
                name='footprint_padding',
                value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=self.footprint_padding)
            )
        ]
        
        # Send request
        request = SetParameters.Request()
        request.parameters = params
        future = global_costmap_client.call_async(request)
        future.add_done_callback(
            lambda f: self.parameter_callback(f, 'global_costmap')
        )
    
    def update_footprints(self):
        """Try to set a custom footprint directly"""
        footprint_client1 = self.create_client(
            SetParameters, 
            '/local_costmap/local_costmap/set_parameters'
        )
        
        footprint_client2 = self.create_client(
            SetParameters, 
            '/global_costmap/global_costmap/set_parameters'
        )
        
        if not footprint_client1.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Local costmap service not available yet')
            return
            
        if not footprint_client2.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Global costmap service not available yet')
            return
        
        # Create a smaller footprint - much smaller than default
        min_footprint = '[[-0.08, -0.08], [-0.08, 0.08], [0.08, 0.08], [0.08, -0.08]]'
        
        footprint_param = Parameter(
            name='footprint',
            value=ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=min_footprint)
        )
        
        # Try setting for local costmap
        request1 = SetParameters.Request()
        request1.parameters = [footprint_param]
        future1 = footprint_client1.call_async(request1)
        future1.add_done_callback(
            lambda f: self.parameter_callback(f, 'local_footprint')
        )
        
        # Try setting for global costmap
        request2 = SetParameters.Request()
        request2.parameters = [footprint_param]
        future2 = footprint_client2.call_async(request2)
        future2.add_done_callback(
            lambda f: self.parameter_callback(f, 'global_footprint')
        )
            
    def parameter_callback(self, future, component_name):
        """Handle the response from parameter update requests"""
        try:
            response = future.result()
            success = True
            
            for result in response.results:
                if not result.successful:
                    success = False
                    self.get_logger().error(f'Failed to set {component_name} parameter: {result.reason}')
            
            if success:
                self.get_logger().info(f'Successfully updated {component_name} safety parameters')
                    
        except Exception as e:
            self.get_logger().error(f'Service call failed for {component_name}: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = SafetyParameterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()