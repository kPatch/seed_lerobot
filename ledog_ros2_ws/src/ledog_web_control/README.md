# LeRobot Web Control

WebSocket-based remote control interface for the LeRobot system via signaling server, enabling web clients to control both the Unitree Go2 mobile platform and the LeRobot manipulator arm.

## Overview

This ROS2 package provides a **WebSocket bridge** that connects to a signaling server as a **producer** in a specific room. Web clients connect to the same server as **consumers** and can send commands to control the robot. The architecture is based on `hurricane_teleop` with added support for arm control.

### Dual Control Support:
- **Go2 Mobile Platform**: Velocity commands via `geometry_msgs/Twist`
- **LeRobot Arm**: Joint position commands via `sensor_msgs/JointState`

The node provides bidirectional communication, sending robot feedback (IMU, odometry, joint states) back to connected clients in the same room.

## Features

✅ **Signaling Server Architecture** - Room-based communication via external server  
✅ **Producer/Consumer Pattern** - Robot as producer, web clients as consumers  
✅ **Dual Control Streams** - Simultaneous control of mobile base and arm  
✅ **Safety Features** - Command timeouts, velocity limits, joint position clamping  
✅ **Real-time Feedback** - IMU, odometry, and joint state data sent to clients  
✅ **Room Isolation** - Multiple robots can operate in different rooms  
✅ **NAT/Firewall Friendly** - Works behind firewalls via cloud signaling server  
✅ **Automatic Reconnection** - Handles server disconnections gracefully  

## Architecture

```
                  ┌──────────────────────────┐
                  │   Signaling Server       │
                  │   (Heroku/Cloud)         │
                  │ wss://server.com/ws      │
                  └────────────┬─────────────┘
                               │
              ┌────────────────┼────────────────┐
              │ Room: "robot1" │                │
              ▼                ▼                ▼
    ┌──────────────┐  ┌──────────────┐  ┌──────────────┐
    │   Robot      │  │ Web Client   │  │ Web Client   │
    │  (Producer)  │  │ (Consumer)   │  │ (Consumer)   │
    │              │  │              │  │              │
    │  ┌────────┐  │  │  ┌────────┐  │  │  ┌────────┐  │
    │  │cmd_vel │◄─┼──┼──┤Commands│  │  │  │Commands│  │
    │  │ /arm   │◄─┼──┼──┤        │  │  │  │        │  │
    │  └────────┘  │  │  └────────┘  │  │  └────────┘  │
    │              │  │              │  │              │
    │  ┌────────┐  │  │  ┌────────┐  │  │  ┌────────┐  │
    │  │Feedback├──┼──┼─►│Display │  │  │  │Display │  │
    │  │ IMU    │  │  │  │        │  │  │  │        │  │
    │  │ Odom   │  │  │  │        │  │  │  │        │  │
    │  └────────┘  │  │  └────────┘  │  │  └────────┘  │
    └──────────────┘  └──────────────┘  └──────────────┘
         ▲                                      
         │                                      
    ┌────┴─────┐                               
    │ Go2 Base │                               
    │LeRobot   │                               
    │  Arm     │                               
    └──────────┘                               
```

## Installation

### Prerequisites

- ROS2 (Humble or later)
- Python 3.8+
- Go2 robot SDK/drivers
- LeRobot arm interface
- Access to a signaling server (or deploy your own)

### Install Dependencies

```bash
# Install Python dependencies in your venv
source /path/to/your/venv/bin/activate
pip install websockets

# Or add to requirements.txt
echo "websockets>=10.0" >> requirements.txt
pip install -r requirements.txt
```

### Build the Package

```bash
# Navigate to workspace
cd ~/ledog_ros2_ws

# Build go2_interfaces first (dependency)
colcon build --packages-select go2_interfaces

# Build ledog_web_control
colcon build --packages-select ledog_web_control

# Source the workspace
source install/setup.bash
```

## Quick Start

### 1. Start the Node

```bash
# Activate your virtual environment (REQUIRED for websockets)
source /home/kpatch/development/seed_lerobot/motor_debugging/venv/bin/activate

# Source ROS2 workspace
source ~/ledog_ros2_ws/install/setup.bash

# Launch with default settings (localhost server)
ros2 launch ledog_web_control web_control.launch.py

# Launch with custom signaling server
ros2 launch ledog_web_control web_control.launch.py \
    signalling_server:=wss://your-server.herokuapp.com/ws \
    room_id:=my_robot_room
```

### 2. Web Client Connection

Web clients connect to the **same signaling server** as consumers in the same room:

```javascript
// Example JavaScript WebSocket client
const signalingServer = 'wss://your-server.herokuapp.com/ws';
const roomId = 'my_robot_room';

const ws = new WebSocket(signalingServer);

ws.onopen = () => {
    console.log('Connected to signaling server');
    
    // Identify as consumer in the room
    ws.send(JSON.stringify({
        role: 'consumer',
        room_id: roomId,
        client_name: 'WebClient1'
    }));
};

ws.onmessage = (event) => {
    const data = JSON.parse(event.data);
    
    if (data.type === 'robot_feedback') {
        console.log('Robot position:', data.position);
        console.log('Joint states:', data.joint_states);
    }
};

// Send velocity command
function sendVelocity(linear_x, angular_z) {
    ws.send(JSON.stringify({
        type: 'cmd_vel',
        room_id: roomId,
        linear: {x: linear_x, y: 0.0, z: 0.0},
        angular: {x: 0.0, y: 0.0, z: angular_z}
    }));
}

// Send arm joint command
function sendArmCommand(positions) {
    ws.send(JSON.stringify({
        type: 'arm_joints',
        room_id: roomId,
        joint_positions: positions
    }));
}

// Emergency stop
function emergencyStop() {
    ws.send(JSON.stringify({
        type: 'stop_all',
        room_id: roomId
    }));
}
```

## WebSocket API

### Message Format

All messages are JSON and include a `room_id` field for routing.

### Inbound Messages (Client → Robot)

#### 1. Velocity Command (`cmd_vel`)

Control the Go2 mobile platform velocity.

```json
{
    "type": "cmd_vel",
    "room_id": "my_robot_room",
    "client_id": "client_123",
    "linear": {
        "x": 0.5,   // Forward/backward (m/s)
        "y": 0.0,   // Left/right (m/s)
        "z": 0.0    // Up/down (m/s)
    },
    "angular": {
        "x": 0.0,   // Roll (rad/s)
        "y": 0.0,   // Pitch (rad/s)
        "z": 0.3    // Yaw/rotation (rad/s)
    }
}
```

**Response:**
```json
{
    "type": "cmd_vel_ack",
    "target_id": "client_123",
    "room_id": "my_robot_room",
    "status": "success",
    "message": "Velocity command received"
}
```

#### 2. Arm Joint Command (`arm_joints`)

Control LeRobot arm joint positions.

```json
{
    "type": "arm_joints",
    "room_id": "my_robot_room",
    "client_id": "client_123",
    "joint_positions": [2048, 1500, 3000, 2048, 2048],
    "joint_names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"]
}
```

**Notes:**
- Positions are in motor encoder units (0-4095 for Feetech STS3215)
- Values are automatically clamped to configured limits
- `joint_names` is optional (uses default names if omitted)

**Response:**
```json
{
    "type": "arm_joints_ack",
    "target_id": "client_123",
    "room_id": "my_robot_room",
    "status": "success",
    "message": "Arm command received"
}
```

#### 3. Stop All (`stop_all`)

Emergency stop for platform velocity.

```json
{
    "type": "stop_all",
    "room_id": "my_robot_room",
    "client_id": "client_123"
}
```

**Response:**
```json
{
    "type": "stop_all_ack",
    "target_id": "client_123",
    "room_id": "my_robot_room",
    "status": "success",
    "message": "Robot stopped"
}
```

### Outbound Messages (Robot → Clients)

#### Robot Feedback

Sent periodically (10 Hz default) to all clients in the room.

```json
{
    "type": "robot_feedback",
    "timestamp": 1234567890.123,
    "room_id": "my_robot_room",
    "robot_name": "ledog-001",
    "imu": {
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
        "angular_velocity": {"x": 0.0, "y": 0.0, "z": 0.0},
        "linear_acceleration": {"x": 0.0, "y": 0.0, "z": 9.81}
    },
    "velocity": {
        "linear": {"x": 0.2, "y": 0.0, "z": 0.0},
        "angular": {"x": 0.0, "y": 0.0, "z": 0.3}
    },
    "position": {
        "x": 1.5,
        "y": 0.8,
        "z": 0.0
    },
    "joint_states": {
        "names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5"],
        "positions": [2048, 1600, 2900, 2048, 2048],
        "velocities": [0.0, 0.0, 0.0, 0.0, 0.0],
        "efforts": [0.0, 0.0, 0.0, 0.0, 0.0]
    }
}
```

#### Heartbeat

Sent periodically (3 seconds default) to confirm connection.

```json
{
    "type": "heartbeat",
    "status": "active",
    "timestamp": 1234567890.123,
    "room_id": "my_robot_room",
    "robot_name": "ledog-001"
}
```

## Configuration

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `signalling_server` | string | `wss://localhost:8000/ws` | Signaling server WebSocket URL |
| `room_id` | string | `default` | Room ID to join |
| `robot_name` | string | hostname | Robot friendly name |
| `cmd_vel_topic` | string | `/cmd_vel` | Velocity command topic |
| `arm_joint_topic` | string | `/arm/joint_commands` | Arm joint command topic |
| `feedback_rate` | double | `10.0` | Feedback rate (Hz) |
| `heartbeat_frequency` | double | `3.0` | Heartbeat frequency (seconds) |
| `command_timeout` | double | `0.5` | Velocity timeout (seconds) |
| `max_linear_speed` | double | `0.5` | Max linear speed (m/s) |
| `max_angular_speed` | double | `1.0` | Max angular speed (rad/s) |
| `ticks_per_second` | int | `30` | Control loop frequency |
| `arm_joint_limits_min` | int[] | `[0, 0, 0, 0, 0]` | Min joint positions |
| `arm_joint_limits_max` | int[] | `[4095, ...]` | Max joint positions |
| `arm_joint_names` | string[] | `[joint_1, ...]` | Joint names |

### Launch File Examples

#### Connect to Hurricane Labs Server

```bash
ros2 launch ledog_web_control web_control.launch.py \
    signalling_server:=wss://hurricane-laboratories.herokuapp.com/ws \
    room_id:=ledog_lab_01
```

#### Custom Room with Conservative Speeds

```bash
ros2 launch ledog_web_control web_control.launch.py \
    signalling_server:=wss://your-server.com/ws \
    room_id:=my_custom_room \
    robot_name:=LeRobot-Alpha \
    max_linear_speed:=0.3 \
    max_angular_speed:=0.5
```

#### High Performance Mode

```bash
ros2 launch ledog_web_control web_control.launch.py \
    feedback_rate:=20.0 \
    ticks_per_second:=50
```

### Parameter File

Edit `config/web_control_params.yaml`:

```yaml
web_control_node:
  ros__parameters:
    signalling_server: "wss://your-server.herokuapp.com/ws"
    room_id: "production_robot_01"
    robot_name: "LeRobot-Production"
    max_linear_speed: 0.5
    max_angular_speed: 1.0
    # ... other parameters
```

Then launch with:

```bash
ros2 launch ledog_web_control web_control.launch.py \
    params_file:=/path/to/custom_params.yaml
```

## Safety Features

### 1. Command Timeout

Velocity commands automatically time out after 0.5 seconds (configurable) to prevent runaway robot if connection is lost.

### 2. Speed Limiting

All velocity commands are clamped to configured maximum speeds:
- Default linear: 0.5 m/s
- Default angular: 1.0 rad/s

### 3. Joint Position Clamping

Arm joint positions are automatically clamped to safe ranges (0-4095 for Feetech motors).

### 4. Room Isolation

Only clients in the same room can control the robot. Commands from other rooms are ignored.

### 5. Automatic Reconnection

If connection to signaling server is lost, the node automatically attempts to reconnect every 5 seconds.

## Complete Launch Script

Create a convenience script that handles venv and ROS2 sourcing:

```bash
#!/bin/bash
# File: ~/ledog_ros2_ws/launch_web_control.sh

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}=== LeRobot Web Control Launcher ===${NC}"

# Source ROS2 workspace
echo -e "${GREEN}[1/3]${NC} Sourcing ROS2 workspace..."
source ~/ledog_ros2_ws/install/setup.bash

# Activate virtual environment
echo -e "${GREEN}[2/3]${NC} Activating virtual environment..."
source /home/kpatch/development/seed_lerobot/motor_debugging/venv/bin/activate

# Verify dependencies
echo -e "${GREEN}[3/3]${NC} Verifying dependencies..."
python3 -c "import websockets; print(f'  ✓ websockets {websockets.__version__}')" || {
    echo "  ✗ websockets not found in venv!"
    exit 1
}

# Launch node
echo -e "${BLUE}Starting web control node...${NC}"
ros2 launch ledog_web_control web_control.launch.py "$@"
```

Make it executable:
```bash
chmod +x ~/ledog_ros2_ws/launch_web_control.sh
```

Use it:
```bash
# Basic launch
~/ledog_ros2_ws/launch_web_control.sh

# With parameters
~/ledog_ros2_ws/launch_web_control.sh \
    signalling_server:=wss://your-server.com/ws \
    room_id:=my_room
```

## Troubleshooting

### Cannot Connect to Signaling Server

```bash
# Check server URL is correct
curl -I https://your-server.herokuapp.com

# Check websockets library
python3 -c "import websockets; print('OK')"

# Check node logs
ros2 node info /web_control_node
```

### No Velocity Commands

```bash
# Check topic
ros2 topic echo /cmd_vel

# Verify room_id matches between robot and clients
ros2 param get /web_control_node room_id
```

### Arm Not Moving

```bash
# Check joint command topic
ros2 topic echo /arm/joint_commands

# Verify arm controller is running
ros2 topic list | grep arm
```

## Signaling Server Setup

This node requires a signaling server. You can:

1. **Use Hurricane Labs server** (if available):
   ```
   wss://hurricane-laboratories.herokuapp.com/ws
   ```

2. **Deploy your own** using the signaling server from hurricane-laboratories repo

3. **Run locally for testing**:
   ```bash
   # Clone and run signaling server locally
   python signalling_server.py
   ```

## Integration Examples

### Python Client

```python
import asyncio
import websockets
import json

async def control_robot():
    server = "wss://your-server.com/ws"
    room_id = "my_robot_room"
    
    async with websockets.connect(server) as ws:
        # Join as consumer
        await ws.send(json.dumps({
            "role": "consumer",
            "room_id": room_id,
            "client_name": "PythonController"
        }))
        
        # Wait for ack
        ack = await ws.recv()
        print(f"Connected: {ack}")
        
        # Send velocity command
        await ws.send(json.dumps({
            "type": "cmd_vel",
            "room_id": room_id,
            "linear": {"x": 0.2, "y": 0.0, "z": 0.0},
            "angular": {"x": 0.0, "y": 0.0, "z": 0.0}
        }))
        
        # Listen for feedback
        while True:
            msg = await ws.recv()
            data = json.loads(msg)
            if data.get('type') == 'robot_feedback':
                print(f"Position: {data.get('position')}")

asyncio.run(control_robot())
```

## Development

### Running Tests

```bash
# Run all tests
colcon test --packages-select ledog_web_control

# Run specific test
python3 -m pytest test/test_flake8.py
```

## Related Packages

- `hurricane_teleop` - Base architecture pattern
- `go2_interfaces` - Custom message types for Go2 robot
- `ledog_policy_controller` - LeRobot policy execution
- `dog_voice_agent` - Voice control interface

## License

MIT License - See LICENSE file for details.

## Maintainer

LeRobot Developer - dev@lerobot.local

---

**Version:** 0.0.0  
**Last Updated:** October 2025  
**ROS2 Version:** Humble+  
**Architecture:** Signaling Server (Producer/Consumer Pattern)
