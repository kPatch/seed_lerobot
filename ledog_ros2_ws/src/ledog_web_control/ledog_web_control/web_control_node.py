#!/usr/bin/env python3
"""
WebSocket Robot Control Node for LeRobot/Go2 System.

This node connects to a signaling server as a producer in a specific room and translates
control messages from web clients into ROS2 messages for robot control.

It supports controlling:
1. Go2 mobile platform via cmd_vel (Twist messages)
2. LeRobot arm via joint position commands

The node also sends robot feedback (IMU, odometry, joint states) back to web clients in the same room.

Usage:
    ros2 run ledog_web_control web_control_node
    
Parameters:
    signalling_server: WebSocket URL for the signalling server (default: wss://localhost:8000/ws)
    room_id: Room ID to join (default: "default")
    robot_name: Friendly name for this robot (default: hostname)
    cmd_vel_topic: Topic for velocity commands (default: /cmd_vel)
    arm_joint_topic: Topic for arm joint commands (default: /arm/joint_commands)
    feedback_rate: Rate to send feedback in Hz (default: 10.0)
    heartbeat_frequency: Frequency to send heartbeat in seconds (default: 3.0)
    command_timeout: Timeout for velocity commands in seconds (default: 0.5)
    max_linear_speed: Maximum linear speed in m/s (default: 0.5)
    max_angular_speed: Maximum angular speed in rad/s (default: 1.0)
"""

import asyncio
import json
import logging
import socket
import threading
import time
from typing import Dict, Any, Optional, Set

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

try:
    import websockets
except ImportError:
    websockets = None

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("web_control_node")


class WebControlNode(Node):
    """
    ROS2 node for WebSocket-based robot control via signaling server.
    
    This node:
    1. Connects to the signalling server as a producer in a specific room
    2. Receives control messages from web clients in the same room
    3. Translates messages into Twist and JointState commands
    4. Publishes commands to ROS2 topics
    5. Subscribes to robot state topics
    6. Sends sensor feedback to web clients in the same room
    """
    
    def __init__(self):
        super().__init__('ledog_web_control')
        
        # Check websockets dependency
        if websockets is None:
            self.get_logger().error(
                "websockets library not found! Install with: pip install websockets"
            )
            raise ImportError("websockets library is required")
        
        # Get hostname for default robot name
        hostname = socket.gethostname()
        
        # Declare parameters
        self._declare_parameters()
        
        # Get parameters
        self.signalling_server = self.get_parameter('signalling_server').value
        self.room_id = self.get_parameter('room_id').value
        self.robot_name = self.get_parameter('robot_name').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.arm_joint_topic = self.get_parameter('arm_joint_topic').value
        self.feedback_rate = self.get_parameter('feedback_rate').value
        self.heartbeat_frequency = self.get_parameter('heartbeat_frequency').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.ticks_per_second = self.get_parameter('ticks_per_second').value
        
        # Get arm joint configuration
        self.arm_joint_limits_min = self.get_parameter('arm_joint_limits_min').value
        self.arm_joint_limits_max = self.get_parameter('arm_joint_limits_max').value
        self.arm_joint_names = self.get_parameter('arm_joint_names').value
        
        # Create callback group for parallel execution
        self.callback_group = ReentrantCallbackGroup()
        
        # Create publishers
        self.vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.arm_joint_pub = self.create_publisher(
            JointState, 
            self.arm_joint_topic, 
            10
        )
        
        # Create subscribers for feedback
        self.imu_sub = self.create_subscription(
            Imu,
            'imu/data',
            self.imu_callback,
            10,
            callback_group=self.callback_group
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10,
            callback_group=self.callback_group
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Initialize state storage
        self.latest_imu = None
        self.latest_odom = None
        self.latest_joint_state = None
        self.last_feedback_time = 0.0
        
        # Initialize command state
        self.twist = Twist()
        self.last_velocity_command_time = 0.0
        self.last_arm_command_time = 0.0
        
        # WebSocket and room management
        self.websocket = None
        self.clients_in_room: Set[str] = set()
        self.loop = None
        
        # Timestamp for synchronization
        self.timestamp = time.time()
        self.tick_time = 1.0 / float(self.ticks_per_second)
        
        # Create timer for main control loop
        self.global_tick_timer = self.create_timer(
            self.tick_time, 
            self.global_tick_timer_callback,
            callback_group=self.callback_group
        )
        
        # Create timer for heartbeat
        self.heartbeat_timer = self.create_timer(
            self.heartbeat_frequency,
            self.send_heartbeat,
            callback_group=self.callback_group
        )
        
        # Log initialization
        self.get_logger().info(f"Starting LeRobot Web Control with signalling server: {self.signalling_server}")
        self.get_logger().info(f"Robot will join room: {self.room_id} as '{self.robot_name}'")
        self.get_logger().info(f"Publishing cmd_vel to: {self.cmd_vel_topic}")
        self.get_logger().info(f"Publishing arm joints to: {self.arm_joint_topic}")
        self.get_logger().info(f"Max speeds - Linear: {self.max_linear_speed} m/s, Angular: {self.max_angular_speed} rad/s")
        self.get_logger().info(f"Feedback rate: {self.feedback_rate} Hz")
        self.get_logger().info(f"Tick time: {self.tick_time} seconds")
        self.get_logger().info(f"Ticks per second: {self.ticks_per_second} TPS")
        
        # Start WebSocket client in separate thread
        self.ws_thread = threading.Thread(target=self.run_websocket_client, daemon=True)
        self.ws_thread.start()
    
    def _declare_parameters(self):
        """Declare all ROS2 parameters."""
        hostname = socket.gethostname()
        
        self.declare_parameter(
            'signalling_server',
            'wss://localhost:8000/ws',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='WebSocket URL for the signalling server'
            )
        )
        self.declare_parameter(
            'room_id',
            'default',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Room ID to join'
            )
        )
        self.declare_parameter(
            'robot_name',
            hostname,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Friendly name for this robot'
            )
        )
        self.declare_parameter(
            'cmd_vel_topic',
            '/cmd_vel',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Topic for velocity commands'
            )
        )
        self.declare_parameter(
            'arm_joint_topic',
            '/arm/joint_commands',
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description='Topic for arm joint position commands'
            )
        )
        self.declare_parameter(
            'feedback_rate',
            10.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Rate to send feedback data in Hz'
            )
        )
        self.declare_parameter(
            'heartbeat_frequency',
            3.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Frequency to send heartbeat messages in seconds'
            )
        )
        self.declare_parameter(
            'command_timeout',
            0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Timeout for velocity commands in seconds'
            )
        )
        self.declare_parameter(
            'max_linear_speed',
            0.5,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Maximum linear speed in m/s'
            )
        )
        self.declare_parameter(
            'max_angular_speed',
            1.0,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description='Maximum angular speed in rad/s'
            )
        )
        self.declare_parameter(
            'ticks_per_second',
            30,
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description='Control loop frequency'
            )
        )
        self.declare_parameter(
            'arm_joint_limits_min',
            [0, 0, 0, 0, 0],
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER_ARRAY,
                description='Minimum joint positions (0-4095 range)'
            )
        )
        self.declare_parameter(
            'arm_joint_limits_max',
            [4095, 4095, 4095, 4095, 4095],
            ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER_ARRAY,
                description='Maximum joint positions (0-4095 range)'
            )
        )
        self.declare_parameter(
            'arm_joint_names',
            ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5'],
            ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING_ARRAY,
                description='Names of arm joints'
            )
        )
    
    def global_tick_timer_callback(self):
        """Main timer callback that handles all periodic tasks."""
        # Update timestamp
        self.timestamp = time.time()
        
        # Publish velocity (with timeout safety)
        self.publish_velocity()
        
        # Send feedback to clients
        self.send_feedback()
    
    def imu_callback(self, msg: Imu):
        """Store latest IMU data."""
        self.latest_imu = msg
    
    def odom_callback(self, msg: Odometry):
        """Store latest odometry data."""
        self.latest_odom = msg
    
    def joint_state_callback(self, msg: JointState):
        """Store latest joint state data."""
        self.latest_joint_state = msg
    
    def publish_velocity(self):
        """Publish velocity command with timeout safety."""
        current_time = time.time()
        
        # Check for command timeout
        if (self.last_velocity_command_time > 0 and 
            current_time - self.last_velocity_command_time > self.command_timeout):
            # Stop the robot if no recent command
            self.twist.linear.x = 0.0
            self.twist.linear.y = 0.0
            self.twist.linear.z = 0.0
            self.twist.angular.x = 0.0
            self.twist.angular.y = 0.0
            self.twist.angular.z = 0.0
            self.get_logger().debug("Velocity command timeout - stopping robot")
        
        # Publish current twist
        self.vel_pub.publish(self.twist)
    
    def send_heartbeat(self):
        """Send a heartbeat message to the signalling server."""
        if self.websocket and self.websocket.open:
            asyncio.run_coroutine_threadsafe(
                self.websocket.send(json.dumps({
                    "type": "heartbeat",
                    "status": "active",
                    "timestamp": self.timestamp,
                    "room_id": self.room_id,
                    "robot_name": self.robot_name
                })),
                self.loop
            )
    
    def send_feedback(self):
        """Send robot feedback to all connected clients in the room."""
        if not self.websocket:
            return
        
        # Rate limiting
        if self.timestamp - self.last_feedback_time < 1.0 / self.feedback_rate:
            return
        
        self.last_feedback_time = self.timestamp
        
        # Prepare feedback message
        feedback_data = {
            "type": "robot_feedback",
            "timestamp": self.timestamp,
            "room_id": self.room_id,
            "robot_name": self.robot_name
        }
        
        # Add IMU data if available
        if self.latest_imu:
            feedback_data["imu"] = {
                "orientation": {
                    "x": self.latest_imu.orientation.x,
                    "y": self.latest_imu.orientation.y,
                    "z": self.latest_imu.orientation.z,
                    "w": self.latest_imu.orientation.w
                },
                "angular_velocity": {
                    "x": self.latest_imu.angular_velocity.x,
                    "y": self.latest_imu.angular_velocity.y,
                    "z": self.latest_imu.angular_velocity.z
                },
                "linear_acceleration": {
                    "x": self.latest_imu.linear_acceleration.x,
                    "y": self.latest_imu.linear_acceleration.y,
                    "z": self.latest_imu.linear_acceleration.z
                }
            }
        
        # Add odometry data if available
        if self.latest_odom:
            feedback_data["velocity"] = {
                "linear": {
                    "x": self.latest_odom.twist.twist.linear.x,
                    "y": self.latest_odom.twist.twist.linear.y,
                    "z": self.latest_odom.twist.twist.linear.z
                },
                "angular": {
                    "x": self.latest_odom.twist.twist.angular.x,
                    "y": self.latest_odom.twist.twist.angular.y,
                    "z": self.latest_odom.twist.twist.angular.z
                }
            }
            feedback_data["position"] = {
                "x": self.latest_odom.pose.pose.position.x,
                "y": self.latest_odom.pose.pose.position.y,
                "z": self.latest_odom.pose.pose.position.z
            }
        
        # Add joint state data if available
        if self.latest_joint_state:
            feedback_data["joint_states"] = {
                "names": list(self.latest_joint_state.name),
                "positions": list(self.latest_joint_state.position),
                "velocities": list(self.latest_joint_state.velocity) if self.latest_joint_state.velocity else [],
                "efforts": list(self.latest_joint_state.effort) if self.latest_joint_state.effort else []
            }
        
        # Send feedback data via WebSocket if we have data to send
        if self.websocket and (self.latest_imu or self.latest_odom or self.latest_joint_state):
            asyncio.run_coroutine_threadsafe(
                self.websocket.send(json.dumps(feedback_data)),
                self.loop
            )
            self.get_logger().debug(f"Sent robot feedback data to clients in room {self.room_id}")
    
    def process_velocity_command(self, data: Dict[str, Any], client_id: str = "unknown") -> Dict[str, Any]:
        """
        Process velocity command from WebSocket.
        
        Args:
            data: Command data containing linear and angular velocities
            client_id: ID of the client sending the command
            
        Returns:
            Response dictionary
        """
        # Verify the command came from a client in our room
        if client_id != "unknown" and client_id not in self.clients_in_room:
            self.get_logger().warning(f"Ignoring velocity command from client {client_id} not in our room")
            return {"status": "error", "message": "Client not in room"}
        
        try:
            linear = data.get('linear', {})
            angular = data.get('angular', {})
            
            # Update twist with clamping
            self.twist.linear.x = self._clamp(
                linear.get('x', 0.0),
                -self.max_linear_speed,
                self.max_linear_speed
            )
            self.twist.linear.y = self._clamp(
                linear.get('y', 0.0),
                -self.max_linear_speed,
                self.max_linear_speed
            )
            self.twist.linear.z = self._clamp(
                linear.get('z', 0.0),
                -self.max_linear_speed,
                self.max_linear_speed
            )
            
            self.twist.angular.x = self._clamp(
                angular.get('x', 0.0),
                -self.max_angular_speed,
                self.max_angular_speed
            )
            self.twist.angular.y = self._clamp(
                angular.get('y', 0.0),
                -self.max_angular_speed,
                self.max_angular_speed
            )
            self.twist.angular.z = self._clamp(
                angular.get('z', 0.0),
                -self.max_angular_speed,
                self.max_angular_speed
            )
            
            # Update command timestamp
            self.last_velocity_command_time = time.time()
            
            self.get_logger().info(
                f"Processing velocity command from client {client_id}: "
                f"linear=({self.twist.linear.x:.2f}, {self.twist.linear.y:.2f}, {self.twist.linear.z:.2f}), "
                f"angular=({self.twist.angular.x:.2f}, {self.twist.angular.y:.2f}, {self.twist.angular.z:.2f})"
            )
            
            return {"status": "success", "message": "Velocity command received"}
            
        except Exception as e:
            self.get_logger().error(f"Error processing velocity command: {e}")
            return {"status": "error", "message": str(e)}
    
    def process_arm_command(self, data: Dict[str, Any], client_id: str = "unknown") -> Dict[str, Any]:
        """
        Process arm joint command from WebSocket.
        
        Args:
            data: Command data containing joint positions
            client_id: ID of the client sending the command
            
        Returns:
            Response dictionary
        """
        # Verify the command came from a client in our room
        if client_id != "unknown" and client_id not in self.clients_in_room:
            self.get_logger().warning(f"Ignoring arm command from client {client_id} not in our room")
            return {"status": "error", "message": "Client not in room"}
        
        try:
            joint_positions = data.get('joint_positions', [])
            joint_names = data.get('joint_names', self.arm_joint_names)
            
            if not joint_positions:
                return {"status": "error", "message": "No joint positions provided"}
            
            # Validate and clamp positions
            clamped_positions = []
            for i, pos in enumerate(joint_positions):
                if i < len(self.arm_joint_limits_min):
                    min_val = self.arm_joint_limits_min[i]
                    max_val = self.arm_joint_limits_max[i]
                    clamped_pos = self._clamp(pos, min_val, max_val)
                    clamped_positions.append(clamped_pos)
                else:
                    clamped_positions.append(pos)
            
            # Create and publish JointState message
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = joint_names[:len(clamped_positions)]
            joint_state.position = clamped_positions
            
            self.arm_joint_pub.publish(joint_state)
            
            # Update command timestamp
            self.last_arm_command_time = time.time()
            
            self.get_logger().info(f"Processing arm command from client {client_id}: positions={clamped_positions}")
            
            return {"status": "success", "message": "Arm command received"}
            
        except Exception as e:
            self.get_logger().error(f"Error processing arm command: {e}")
            return {"status": "error", "message": str(e)}
    
    def process_stop_command(self, data: Dict[str, Any], client_id: str = "unknown") -> Dict[str, Any]:
        """
        Process stop command - stops velocity.
        
        Args:
            data: Command data
            client_id: ID of the client sending the command
            
        Returns:
            Response dictionary
        """
        # Verify the command came from a client in our room
        if client_id != "unknown" and client_id not in self.clients_in_room:
            self.get_logger().warning(f"Ignoring stop command from client {client_id} not in our room")
            return {"status": "error", "message": "Client not in room"}
        
        # Stop velocity
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.last_velocity_command_time = 0.0
        
        self.get_logger().info(f"Stop command received from client {client_id}")
        
        return {"status": "success", "message": "Robot stopped"}
    
    def _clamp(self, value: float, min_val: float, max_val: float) -> float:
        """Clamp a value between min and max."""
        return max(min_val, min(max_val, value))
    
    def run_websocket_client(self):
        """Run the WebSocket client in a separate thread."""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self.producer_client())
    
    async def producer_client(self):
        """
        WebSocket client that connects to the signalling server as a producer.
        
        Based on hurricane_teleop pattern but with arm control support.
        """
        uri = self.signalling_server
        
        while rclpy.ok():
            try:
                self.get_logger().info(f"Connecting to signalling server at {uri}")
                async with websockets.connect(uri) as websocket:
                    # Save the websocket connection for sending feedback later
                    self.websocket = websocket
                    
                    # Create room metadata
                    room_metadata = {
                        "robot_type": "LeRobot Go2",
                        "robot_name": self.robot_name,
                        "capabilities": ["cmd_vel", "arm_joints"]
                    }
                    
                    # Identify as producer in a specific room
                    await websocket.send(json.dumps({
                        "role": "producer",
                        "room_id": self.room_id,
                        "robot_name": self.robot_name,
                        "metadata": room_metadata
                    }))
                    
                    # Wait for server to acknowledge connection
                    response = await websocket.recv()
                    response_data = json.loads(response)
                    
                    # Verify we got a proper acknowledgment
                    if response_data.get("type") != "ack" or response_data.get("role") != "producer":
                        self.get_logger().error(f"Unexpected response from server: {response}")
                        self.websocket = None
                        await asyncio.sleep(5)
                        continue
                    
                    # Verify the room ID
                    if response_data.get("room_id") != self.room_id:
                        self.get_logger().warning(
                            f"Server assigned a different room: {response_data.get('room_id')} instead of {self.room_id}")
                        self.room_id = response_data.get("room_id")
                    
                    self.get_logger().info(f"Successfully connected to room {self.room_id} as producer")
                    
                    # Clear the client list and reset state
                    self.clients_in_room = set()
                    
                    # Send a heartbeat message to confirm connection
                    await self.websocket.send(json.dumps({
                        "type": "heartbeat",
                        "status": "connected",
                        "room_id": self.room_id,
                        "timestamp": self.timestamp,
                        "robot_name": self.robot_name
                    }))
                    
                    # Send an additional heartbeat after a short delay
                    await asyncio.sleep(0.2)
                    await self.websocket.send(json.dumps({
                        "type": "heartbeat",
                        "status": "active",
                        "room_id": self.room_id,
                        "timestamp": self.timestamp,
                        "robot_name": self.robot_name
                    }))
                    
                    # Request room status to force an update to all clients
                    await self.websocket.send(json.dumps({
                        "type": "room_status_check",
                        "room_id": self.room_id
                    }))
                    
                    # Process incoming messages
                    while rclpy.ok():
                        message = await websocket.recv()
                        data = json.loads(message)
                        
                        # Skip messages not intended for our room
                        msg_room_id = data.get("room_id")
                        if msg_room_id and msg_room_id != self.room_id:
                            self.get_logger().debug(f"Ignoring message for room {msg_room_id}")
                            continue
                        
                        msg_type = data.get("type", "")
                        client_id = data.get("client_id", "unknown")
                        
                        if msg_type == "cmd_vel":
                            # Process velocity command
                            response = self.process_velocity_command(data, client_id)
                            
                            # Send acknowledgment back to the client
                            await websocket.send(json.dumps({
                                "type": "cmd_vel_ack",
                                "target_id": client_id,
                                "status": response["status"],
                                "message": response["message"],
                                "room_id": self.room_id
                            }))
                        
                        elif msg_type == "arm_joints":
                            # Process arm joint command
                            response = self.process_arm_command(data, client_id)
                            
                            # Send acknowledgment back to the client
                            await websocket.send(json.dumps({
                                "type": "arm_joints_ack",
                                "target_id": client_id,
                                "status": response["status"],
                                "message": response["message"],
                                "room_id": self.room_id
                            }))
                        
                        elif msg_type == "stop_all":
                            # Process stop command
                            response = self.process_stop_command(data, client_id)
                            
                            # Send acknowledgment back to the client
                            await websocket.send(json.dumps({
                                "type": "stop_all_ack",
                                "target_id": client_id,
                                "status": response["status"],
                                "message": response["message"],
                                "room_id": self.room_id
                            }))
                        
                        elif msg_type == "connect":
                            # New client connected to our room
                            client_id = data.get("client_id")
                            self.clients_in_room.add(client_id)
                            self.get_logger().info(f"New client {client_id} connected to room {self.room_id}")
                        
                        elif msg_type == "disconnect":
                            # Client disconnected from our room
                            client_id = data.get("client_id")
                            if client_id in self.clients_in_room:
                                self.clients_in_room.remove(client_id)
                            self.get_logger().info(f"Client {client_id} disconnected from room {self.room_id}")
                        
                        else:
                            self.get_logger().debug(f"Received other message: {data}")
            
            except websockets.exceptions.ConnectionClosed:
                self.get_logger().warning("Connection to server closed")
            except Exception as e:
                self.get_logger().error(f"Error in producer client: {e}")
            
            # Reset the websocket connection reference and client list
            self.websocket = None
            self.clients_in_room = set()
            
            # Wait before reconnecting
            if rclpy.ok():
                self.get_logger().info("Waiting 5 seconds before reconnecting...")
                await asyncio.sleep(5)
            else:
                break


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)
    
    try:
        node = WebControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"Error in web control node: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
