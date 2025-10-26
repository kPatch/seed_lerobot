# LeRobot Policy Controller

A ROS2 package for managing LeRobot policy execution on the LeKiwi robot dog platform. This package provides a clean service-based interface to start, stop, and monitor long-running LeRobot policy recording commands.

## Overview

The LeRobot Policy Controller solves the problem of managing the `lerobot-record` CLI command, which runs indefinitely until manually stopped. This package wraps the command execution in a ROS2 node and provides services for lifecycle management.

### Key Features

- ✅ **Service-based control** - Start, stop, and query policy status via ROS2 services
- ✅ **Process lifecycle management** - Proper handling of long-running processes
- ✅ **Conda environment support** - Activates the correct conda environment before execution
- ✅ **Graceful shutdown** - Attempts graceful termination before force-killing
- ✅ **Status monitoring** - Always know if the policy is running, idle, or stopped
- ✅ **Thread-safe** - Safe concurrent access to process state
- ✅ **Logging** - Streams LeRobot output to ROS2 logs

## Architecture

```
┌─────────────────────────────────────────┐
│   Policy Controller Node                │
│                                         │
│  ┌─────────────────────────────────┐   │
│  │  Services:                       │   │
│  │  • /ledog/start_policy          │   │
│  │  • /ledog/stop_policy           │   │
│  │  • /ledog/get_policy_status     │   │
│  └─────────────────────────────────┘   │
│                                         │
│  ┌─────────────────────────────────┐   │
│  │  Process Manager                 │   │
│  │  • Conda activation              │   │
│  │  • Subprocess management         │   │
│  │  • Output streaming              │   │
│  └─────────────────────────────────┘   │
└─────────────────────────────────────────┘
            ↓
    ┌───────────────┐
    │  lerobot-record│
    │  (subprocess)  │
    └───────────────┘
```

## Installation

### Prerequisites

1. **ROS2** (Humble or later)
2. **Conda** with LeRobot installed
3. **LeRobot environment** configured and working

### Build Instructions

```bash
# Navigate to your workspace
cd ~/ledog_ros2_ws

# Build the package
colcon build --packages-select ledog_policy_controller

# Source the workspace
source install/setup.bash
```

## Usage

### Starting the Node

#### Method 1: Using ros2 run
```bash
ros2 run ledog_policy_controller policy_controller_node
```

#### Method 2: Using the launch file
```bash
ros2 launch ledog_policy_controller policy_controller.launch.py
```

#### Method 3: With custom parameters
```bash
ros2 launch ledog_policy_controller policy_controller.launch.py \
    conda_env_name:=my_lerobot_env \
    conda_base_path:=~/miniconda3 \
    shutdown_timeout:=10.0
```

### Controlling the Policy

Once the node is running, use these services to control the policy:

#### Start the Policy
```bash
ros2 service call /ledog/start_policy std_srvs/srv/Trigger
```

**Response:**
```
success: True
message: 'LeRobot policy started successfully (PID: 12345)'
```

#### Stop the Policy
```bash
ros2 service call /ledog/stop_policy std_srvs/srv/Trigger
```

**Response:**
```
success: True
message: 'Policy stopped gracefully (PID: 12345, exit code: 0)'
```

#### Check Policy Status
```bash
ros2 service call /ledog/get_policy_status std_srvs/srv/Trigger
```

**Possible responses:**
```
# When idle
success: True
message: 'idle (no policy has been started)'

# When running
success: True
message: 'running (PID: 12345)'

# When finished successfully
success: True
message: 'finished successfully (exit code: 0)'

# When finished with error
success: False
message: 'finished with error (exit code: 1)'
```

## Services

### `/ledog/start_policy`

**Type:** `std_srvs/srv/Trigger`

**Description:** Starts the LeRobot policy recording process.

**Behavior:**
- Returns immediately after starting the subprocess
- Validates that no policy is already running
- Streams process output to ROS2 logs
- Returns success/failure with descriptive message

**Example:**
```bash
ros2 service call /ledog/start_policy std_srvs/srv/Trigger
```

---

### `/ledog/stop_policy`

**Type:** `std_srvs/srv/Trigger`

**Description:** Stops the currently running policy process.

**Behavior:**
- First attempts graceful termination (SIGTERM)
- Waits for `shutdown_timeout` seconds
- Force kills (SIGKILL) if graceful shutdown fails
- Returns success/failure with exit code

**Example:**
```bash
ros2 service call /ledog/stop_policy std_srvs/srv/Trigger
```

---

### `/ledog/get_policy_status`

**Type:** `std_srvs/srv/Trigger`

**Description:** Queries the current status of the policy process.

**Behavior:**
- Non-blocking status check using `poll()`
- Returns current state: idle, running, or finished
- Provides PID for running processes
- Provides exit code for finished processes

**Example:**
```bash
ros2 service call /ledog/get_policy_status std_srvs/srv/Trigger
```

## Configuration

### ROS2 Parameters

The node accepts the following parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `conda_env_name` | string | `lerobot` | Name of the conda environment containing LeRobot |
| `conda_base_path` | string | `~/anaconda3` | Path to conda base installation |
| `shutdown_timeout` | double | `5.0` | Seconds to wait for graceful shutdown before force kill |
| `robot_id` | string | `follower_1` | Robot ID for LeRobot recording |
| `robot_port` | string | `/dev/ttyACM0` | Serial port for robot communication |
| `robot_type` | string | `so100_follower` | Type of robot for LeRobot |

### Setting Parameters

#### Via launch file:
```bash
ros2 launch ledog_policy_controller policy_controller.launch.py \
    conda_env_name:=my_env \
    conda_base_path:=~/miniconda3
```

#### Via parameter file:
```yaml
# policy_controller_params.yaml
policy_controller_node:
  ros__parameters:
    conda_env_name: "lerobot"
    conda_base_path: "~/anaconda3"
    shutdown_timeout: 5.0
    robot_id: "follower_1"
    robot_port: "/dev/ttyACM0"
    robot_type: "so100_follower"
```

```bash
ros2 run ledog_policy_controller policy_controller_node \
    --ros-args --params-file policy_controller_params.yaml
```

## LeRobot Command

The node executes the following hardcoded LeRobot command:

```bash
lerobot-record \
    --robot.id=follower_1 \
    --robot.port=/dev/ttyACM0 \
    --robot.type=so100_follower \
    --dataset.repo_id=zhoumiaosen/eval_policy_act1 \
    --dataset.num_episodes=2 \
    --dataset.single_task="Scoop the poop" \
    --display_data=true \
    --robot.cameras='{ top: {"type": "opencv", "index_or_path": 2, "width": 640, "height": 480, "fps": 30}, left: {"type": "opencv", "index_or_path": 0, "width": 640, "height": 480, "fps": 30} }' \
    --policy.device=cuda \
    --policy.path=ishandotsh/poopascoopa_act \
    --dataset.fps=30 \
    --dataset.episode_time_s=120
```

**Note:** To modify the command, edit the `_run_policy_thread()` method in `policy_controller_node.py`.

## Workflow Examples

### Basic Workflow
```bash
# Terminal 1: Start the controller node
ros2 launch ledog_policy_controller policy_controller.launch.py

# Terminal 2: Check initial status
ros2 service call /ledog/get_policy_status std_srvs/srv/Trigger
# Response: idle (no policy has been started)

# Start the policy
ros2 service call /ledog/start_policy std_srvs/srv/Trigger
# Response: LeRobot policy started successfully (PID: 12345)

# Check status while running
ros2 service call /ledog/get_policy_status std_srvs/srv/Trigger
# Response: running (PID: 12345)

# Stop the policy when done
ros2 service call /ledog/stop_policy std_srvs/srv/Trigger
# Response: Policy stopped gracefully (PID: 12345, exit code: 0)

# Check final status
ros2 service call /ledog/get_policy_status std_srvs/srv/Trigger
# Response: finished successfully (exit code: 0)
```

### Integration with Other Nodes

```python
# Example: Triggering policy from another ROS2 node
from std_srvs.srv import Trigger

class MyControllerNode(Node):
    def __init__(self):
        super().__init__('my_controller')
        
        # Create service clients
        self.start_client = self.create_client(
            Trigger, '/ledog/start_policy')
        self.stop_client = self.create_client(
            Trigger, '/ledog/stop_policy')
        self.status_client = self.create_client(
            Trigger, '/ledog/get_policy_status')
    
    async def start_policy(self):
        """Start the LeRobot policy"""
        request = Trigger.Request()
        response = await self.start_client.call_async(request)
        
        if response.success:
            self.get_logger().info(f'Policy started: {response.message}')
        else:
            self.get_logger().error(f'Failed to start: {response.message}')
        
        return response.success
```

## Troubleshooting

### Policy fails to start

**Symptom:** Service returns "Policy process started but exited immediately"

**Solutions:**
1. Check conda environment name:
   ```bash
   conda env list  # Verify 'lerobot' exists
   ```

2. Check conda base path:
   ```bash
   ls ~/anaconda3/etc/profile.d/conda.sh  # Should exist
   # or
   ls ~/miniconda3/etc/profile.d/conda.sh
   ```

3. Test LeRobot command manually:
   ```bash
   conda activate lerobot
   lerobot-record --help
   ```

4. Check node logs:
   ```bash
   ros2 topic echo /rosout
   ```

### Hardware not found

**Symptom:** Process starts but reports hardware errors in logs

**Solutions:**
1. Check serial port permissions:
   ```bash
   ls -l /dev/ttyACM0
   sudo usermod -a -G dialout $USER
   # Log out and back in
   ```

2. Verify robot is connected:
   ```bash
   ls /dev/ttyACM*
   ```

3. Check camera availability:
   ```bash
   ls /dev/video*
   v4l2-ctl --list-devices
   ```

### Cannot stop policy

**Symptom:** Stop service returns error or process continues running

**Solutions:**
1. Check if process is actually running:
   ```bash
   ros2 service call /ledog/get_policy_status std_srvs/srv/Trigger
   ```

2. Find and manually kill if needed:
   ```bash
   ps aux | grep lerobot-record
   kill <PID>
   ```

3. Increase shutdown timeout:
   ```bash
   ros2 launch ledog_policy_controller policy_controller.launch.py \
       shutdown_timeout:=10.0
   ```

## Development

### File Structure

```
ledog_policy_controller/
├── ledog_policy_controller/
│   ├── __init__.py
│   └── policy_controller_node.py    # Main node implementation
├── launch/
│   └── policy_controller.launch.py  # Launch file with parameters
├── resource/
│   └── ledog_policy_controller      # ament resource marker
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   └── test_pep257.py
├── package.xml                       # Package dependencies
├── setup.py                          # Package setup
├── setup.cfg                         # Setup configuration
├── LICENSE                           # MIT License
└── README.md                         # This file
```

### Extending the Package

To modify the LeRobot command:

1. Edit `ledog_policy_controller/policy_controller_node.py`
2. Locate the `_run_policy_thread()` method
3. Modify the `cmd` string with your desired command
4. Rebuild the package:
   ```bash
   colcon build --packages-select ledog_policy_controller
   ```

To make the command configurable via parameters, you would need to:
1. Declare additional parameters in `__init__()`
2. Build the command string dynamically using parameter values
3. Update the launch file with new parameter declarations

## Testing

### Unit Tests

Run the included tests:
```bash
colcon test --packages-select ledog_policy_controller
colcon test-result --verbose
```

### Manual Testing

Test with a simple mock command first:
```python
# In _run_policy_thread(), temporarily replace cmd with:
cmd = "bash -c 'for i in {1..10}; do echo \"Test $i\"; sleep 1; done'"
```

Then test the lifecycle:
1. Start the policy
2. Check status (should be running)
3. Stop the policy
4. Check status (should be finished)

## Integration

This package integrates with the broader LeKiwi ecosystem:

- **lekiwi_base**: Motor control for the mobile base
- **lekiwi_bringup**: System startup and configuration
- **dog_voice_agent**: Voice control and AI integration
- **lekiwi_teleop**: Manual control interface

Policy recording can be triggered automatically based on voice commands, sensor inputs, or scheduled tasks from other nodes in the system.

## License

MIT License - See LICENSE file for details

## Authors

- **kpatch** - Initial work - [irvsteve@gmail.com](mailto:irvsteve@gmail.com)

## Acknowledgments

- LeRobot team for the excellent robotics toolkit
- ROS2 community for the robust framework
- LeKiwi project for the hardware platform

## Support

For issues, questions, or contributions:
1. Check the troubleshooting section above
2. Review ROS2 logs: `ros2 topic echo /rosout`
3. Test LeRobot command independently
4. Verify conda environment and hardware setup

---

**Version:** 0.0.0  
**Last Updated:** 2025-10-26  
**ROS2 Version:** Humble+

