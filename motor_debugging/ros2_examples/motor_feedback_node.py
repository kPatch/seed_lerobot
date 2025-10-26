#!/usr/bin/env python3
"""
ROS2 Motor Feedback Monitor Node for LeKiwi

This node monitors motor feedback and publishes diagnostics.
Useful for debugging and monitoring motor health.
"""

import sys

try:
    import rclpy
    from rclpy.node import Node
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    from sensor_msgs.msg import Temperature
except ImportError:
    print("Error: ROS2 not installed or not sourced")
    print("Please install ROS2 and source the setup:")
    print("  source /opt/ros/humble/setup.bash  # or your ROS2 distro")
    sys.exit(1)


class LeKiwiFeedbackMonitor(Node):
    """ROS2 node for monitoring LeKiwi motor feedback."""
    
    def __init__(self, port='/dev/ttyUSB0', motor_ids=[7, 8, 9]):
        super().__init__('lekiwi_feedback_monitor')
        
        self.port = port
        self.motor_ids = motor_ids
        
        # Declare parameters
        self.declare_parameter('port', port)
        self.declare_parameter('motor_ids', motor_ids)
        self.declare_parameter('update_rate', 10.0)  # Hz
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.motor_ids = self.get_parameter('motor_ids').value
        update_rate = self.get_parameter('update_rate').value
        
        self.get_logger().info(f'Initializing feedback monitor on {self.port}')
        
        # Publisher for diagnostics
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )
        
        # Publishers for individual motor temperatures
        self.temp_pubs = {}
        for motor_id in self.motor_ids:
            topic = f'motor_{motor_id}/temperature'
            self.temp_pubs[motor_id] = self.create_publisher(
                Temperature,
                topic,
                10
            )
        
        # Timer for publishing feedback
        self.timer = self.create_timer(1.0 / update_rate, self.publish_diagnostics)
        
        # Connect to motors
        self.connect_motors()
        
        self.get_logger().info('Feedback monitor node started')
    
    def connect_motors(self):
        """Connect to motors."""
        try:
            # Placeholder for actual motor connection
            self.get_logger().info('Connected to motors')
        except Exception as e:
            self.get_logger().error(f'Failed to connect: {e}')
            raise
    
    def read_motor_feedback(self, motor_id):
        """Read feedback from a motor."""
        # Placeholder - would read actual motor data
        # Simulated data for demonstration
        return {
            'position': 2048,
            'temperature': 35.0 + motor_id,
            'voltage': 12.0,
            'current': 150,
            'load': 20
        }
    
    def check_motor_health(self, motor_id, feedback):
        """Check motor health and return diagnostic status."""
        status = DiagnosticStatus()
        status.name = f'Motor {motor_id}'
        status.hardware_id = f'motor_{motor_id}'
        
        # Determine status level
        level = DiagnosticStatus.OK
        messages = []
        
        # Temperature check
        if feedback['temperature'] > 70:
            level = DiagnosticStatus.ERROR
            messages.append('Critical temperature')
        elif feedback['temperature'] > 60:
            level = max(level, DiagnosticStatus.WARN)
            messages.append('Elevated temperature')
        
        # Voltage check
        if feedback['voltage'] < 10.5 or feedback['voltage'] > 13.0:
            level = DiagnosticStatus.ERROR
            messages.append('Voltage out of range')
        
        # Current check
        if feedback['current'] > 1000:
            level = DiagnosticStatus.ERROR
            messages.append('Excessive current')
        elif feedback['current'] > 900:
            level = max(level, DiagnosticStatus.WARN)
            messages.append('High current')
        
        # Load check
        if feedback['load'] > 80:
            level = max(level, DiagnosticStatus.WARN)
            messages.append('High load')
        
        status.level = level
        status.message = '; '.join(messages) if messages else 'OK'
        
        # Add detailed values
        status.values = [
            KeyValue(key='Position', value=str(feedback['position'])),
            KeyValue(key='Temperature', value=f"{feedback['temperature']:.1f} °C"),
            KeyValue(key='Voltage', value=f"{feedback['voltage']:.2f} V"),
            KeyValue(key='Current', value=f"{feedback['current']} mA"),
            KeyValue(key='Load', value=f"{feedback['load']} %"),
        ]
        
        return status
    
    def publish_diagnostics(self):
        """Read motor feedback and publish diagnostics."""
        try:
            # Diagnostics array
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()
            
            # Process each motor
            for motor_id in self.motor_ids:
                # Read feedback
                feedback = self.read_motor_feedback(motor_id)
                
                # Check health and create diagnostic status
                status = self.check_motor_health(motor_id, feedback)
                diag_array.status.append(status)
                
                # Publish temperature separately
                temp_msg = Temperature()
                temp_msg.header.stamp = diag_array.header.stamp
                temp_msg.header.frame_id = f'motor_{motor_id}'
                temp_msg.temperature = feedback['temperature']
                temp_msg.variance = 0.1  # Temperature sensor variance
                
                self.temp_pubs[motor_id].publish(temp_msg)
            
            # Publish diagnostics
            self.diagnostics_pub.publish(diag_array)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing diagnostics: {e}')


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    try:
        node = LeKiwiFeedbackMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()

