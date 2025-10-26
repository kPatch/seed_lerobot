#!/usr/bin/env python3
"""
ROS2 Motor Controller Node for LeKiwi

This node provides ROS2 interface to control LeKiwi motors.
Subscribes to velocity commands and publishes motor feedback.
"""

import sys

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import JointState
except ImportError:
    print("Error: ROS2 not installed or not sourced")
    print("Please install ROS2 and source the setup:")
    print("  source /opt/ros/humble/setup.bash  # or your ROS2 distro")
    sys.exit(1)


class LeKiwiMotorController(Node):
    """ROS2 node for LeKiwi motor control."""
    
    def __init__(self, port='/dev/ttyUSB0', motor_ids=[7, 8, 9]):
        super().__init__('lekiwi_motor_controller')
        
        self.port = port
        self.motor_ids = motor_ids
        
        # Declare parameters
        self.declare_parameter('port', port)
        self.declare_parameter('motor_ids', motor_ids)
        self.declare_parameter('update_rate', 50.0)  # Hz
        
        # Get parameters
        self.port = self.get_parameter('port').value
        self.motor_ids = self.get_parameter('motor_ids').value
        update_rate = self.get_parameter('update_rate').value
        
        self.get_logger().info(f'Initializing LeKiwi controller on {self.port}')
        self.get_logger().info(f'Motor IDs: {self.motor_ids}')
        
        # Subscribe to velocity commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Publisher for motor feedback
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # Timer for publishing feedback
        self.timer = self.create_timer(1.0 / update_rate, self.publish_feedback)
        
        # Initialize motor connection
        self.connect_motors()
        
        self.get_logger().info('LeKiwi motor controller node started')
    
    def connect_motors(self):
        """Connect to motors."""
        try:
            # Placeholder for actual motor connection
            # In real implementation:
            # self.motor_bus = FeetechBus(self.port)
            # self.motors = [Motor(id) for id in self.motor_ids]
            
            self.get_logger().info('Motors connected successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to motors: {e}')
            raise
    
    def cmd_vel_callback(self, msg):
        """Handle velocity command messages."""
        # Extract velocity commands
        linear_x = msg.linear.x   # Forward/backward
        linear_y = msg.linear.y   # Left/right (strafe)
        angular_z = msg.angular.z  # Rotation
        
        self.get_logger().debug(
            f'Received cmd_vel: linear=[{linear_x:.2f}, {linear_y:.2f}], '
            f'angular={angular_z:.2f}'
        )
        
        # Convert to motor velocities for omnidirectional drive
        # LeKiwi uses 3-wheel omnidirectional configuration
        motor_velocities = self.calculate_motor_velocities(linear_x, linear_y, angular_z)
        
        # Send commands to motors
        self.send_motor_commands(motor_velocities)
    
    def calculate_motor_velocities(self, vx, vy, omega):
        """
        Calculate individual motor velocities for omnidirectional drive.
        
        For 3-wheel omnidirectional base with 120° spacing:
        Motor 7 (front): 0°
        Motor 8 (back-left): 120°
        Motor 9 (back-right): 240°
        """
        import math
        
        # Wheel positions (in radians)
        angles = [0, 2*math.pi/3, 4*math.pi/3]  # 0°, 120°, 240°
        
        # Calculate velocity for each wheel
        velocities = []
        for angle in angles:
            # Omnidirectional kinematics
            v_wheel = (
                -vx * math.sin(angle) + 
                vy * math.cos(angle) + 
                omega * 0.15  # Robot radius (meters)
            )
            velocities.append(v_wheel)
        
        return velocities
    
    def send_motor_commands(self, velocities):
        """Send velocity commands to motors."""
        try:
            # Placeholder for actual motor commands
            # In real implementation:
            # for motor_id, velocity in zip(self.motor_ids, velocities):
            #     self.motor_bus.set_velocity(motor_id, velocity)
            
            self.get_logger().debug(f'Motor velocities: {velocities}')
        except Exception as e:
            self.get_logger().error(f'Failed to send motor commands: {e}')
    
    def publish_feedback(self):
        """Publish motor state feedback."""
        try:
            # Read motor states
            # Placeholder - would read actual motor data
            
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [f'motor_{id}' for id in self.motor_ids]
            
            # Placeholder data
            msg.position = [0.0] * len(self.motor_ids)
            msg.velocity = [0.0] * len(self.motor_ids)
            msg.effort = [0.0] * len(self.motor_ids)
            
            self.joint_state_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish feedback: {e}')
    
    def shutdown(self):
        """Cleanup on shutdown."""
        self.get_logger().info('Shutting down motor controller')
        # Stop all motors
        self.send_motor_commands([0.0] * len(self.motor_ids))


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    try:
        node = LeKiwiMotorController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

