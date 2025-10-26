# ROS2 Integration Examples for LeKiwi

These examples demonstrate how to integrate the LeKiwi mobile base with ROS2.

## Prerequisites

1. ROS2 installed (Humble or later recommended)
2. LeRobot installed with Feetech support
3. Python packages: `rclpy`, `geometry_msgs`, `sensor_msgs`, `diagnostic_msgs`

## Installation

```bash
# Source ROS2
source /opt/ros/humble/setup.bash  # or your ROS2 distro

# Install ROS2 Python dependencies (if not already installed)
sudo apt install ros-humble-rclpy ros-humble-geometry-msgs \
                 ros-humble-sensor-msgs ros-humble-diagnostic-msgs
```

## Available Nodes

### 1. Motor Controller Node

Controls the LeKiwi motors via ROS2 cmd_vel messages.

**Run:**
```bash
python ros2_examples/motor_controller_node.py
```

**With parameters:**
```bash
ros2 run python_executor motor_controller_node.py \
  --ros-args \
  -p port:=/dev/ttyUSB0 \
  -p motor_ids:=[7,8,9] \
  -p update_rate:=50.0
```

**Topics:**
- Subscribes: `/cmd_vel` (geometry_msgs/Twist)
- Publishes: `/joint_states` (sensor_msgs/JointState)

### 2. Feedback Monitor Node

Monitors motor health and publishes diagnostics.

**Run:**
```bash
python ros2_examples/motor_feedback_node.py
```

**Topics:**
- Publishes: `/diagnostics` (diagnostic_msgs/DiagnosticArray)
- Publishes: `/motor_<id>/temperature` (sensor_msgs/Temperature)

## Usage Examples

### Basic Teleoperation with ROS2

1. Start the motor controller:
```bash
python ros2_examples/motor_controller_node.py
```

2. In another terminal, use keyboard teleoperation:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

3. Monitor diagnostics:
```bash
python ros2_examples/motor_feedback_node.py
```

### View Topics

```bash
# List all topics
ros2 topic list

# Echo velocity commands
ros2 topic echo /cmd_vel

# Echo joint states
ros2 topic echo /joint_states

# Echo diagnostics
ros2 topic echo /diagnostics
```

### Send Manual Commands

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# Rotate
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"

# Stop
ros2 topic pub /cmd_vel geometry_msgs/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Visualization with RViz

```bash
# Launch RViz
rviz2

# Add displays:
# - RobotModel (if you have a URDF)
# - TF (for coordinate frames)
# - DiagnosticArray (for motor diagnostics)
```

### Record Data

```bash
# Record all topics
ros2 bag record -a

# Record specific topics
ros2 bag record /cmd_vel /joint_states /diagnostics

# Playback
ros2 bag play <bag_file>
```

## Creating a Launch File

Create `lekiwi_bringup.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lekiwi',
            executable='motor_controller_node.py',
            name='motor_controller',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'motor_ids': [7, 8, 9],
                'update_rate': 50.0
            }]
        ),
        Node(
            package='lekiwi',
            executable='motor_feedback_node.py',
            name='feedback_monitor',
            parameters=[{
                'port': '/dev/ttyUSB0',
                'motor_ids': [7, 8, 9],
                'update_rate': 10.0
            }]
        )
    ])
```

Launch with:
```bash
ros2 launch lekiwi lekiwi_bringup.launch.py
```

## Troubleshooting

### Node fails to start
- Ensure ROS2 is sourced: `source /opt/ros/humble/setup.bash`
- Check if port exists: `ls -l /dev/ttyUSB*`
- Verify permissions: `sudo chmod 666 /dev/ttyUSB0`

### No messages on topics
- Check if node is running: `ros2 node list`
- Verify topic names: `ros2 topic list`
- Check network: `ros2 doctor`

### Motors not responding
- Test motors with standalone scripts first
- Verify motor IDs are correct
- Check power supply (12V)

## Integration with Navigation Stack

For autonomous navigation, integrate with:
- Nav2 (navigation stack)
- SLAM Toolbox (mapping)
- Robot Localization (odometry fusion)

See ROS2 Navigation documentation for details.

