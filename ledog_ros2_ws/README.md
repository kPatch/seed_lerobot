# LeKiwi ROS2 Workspace

ROS2 workspace for LeKiwi mobile base control and integration.

## 📦 Packages

### 1. **lekiwi_base** - Core Motor Control
Low-level motor control and hardware interface for Feetech STS3215 motors.

**Features:**
- Motor controller node (subscribes to `cmd_vel`)
- Motor state publisher (publishes `joint_states`, `odom`, `diagnostics`)
- Omnidirectional kinematics (3-wheel, 120° spacing)
- Safety monitoring (temperature, current, voltage)
- Odometry estimation from wheel encoders

**Topics:**
- Subscribes: `/cmd_vel` (geometry_msgs/Twist)
- Publishes: `/joint_states`, `/odom`, `/diagnostics`, `/tf`

### 2. **lekiwi_bringup** - Launch & Configuration
Launch files and parameter configurations for the LeKiwi system.

**Contents:**
- `launch/` - Launch files for different scenarios
- `config/` - YAML configuration files

### 3. **lekiwi_description** - Robot Model
URDF/XACRO robot description and visualization files.

**Contents:**
- `urdf/` - Robot URDF/XACRO files
- `meshes/` - STL/DAE mesh files for visualization
- `rviz/` - RViz configuration files
- `launch/` - Launch files for visualization

### 4. **lekiwi_teleop** - Teleoperation
Human control interfaces for the mobile base.

**Nodes:**
- Keyboard teleoperation
- Joystick teleoperation

## 🚀 Quick Start

### Prerequisites

- ROS2 Humble (or later) installed
- LeKiwi hardware with Feetech STS3215 motors
- Hardware validation completed (use `motor_debugging/` package first)

### Build Workspace

```bash
cd ledog_ros2_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build

# Source the workspace
source install/setup.bash
```

### Run Basic Test

```bash
# Launch the base controller
ros2 launch lekiwi_bringup base_only.launch.py

# In another terminal, send velocity commands
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Run with Teleoperation

```bash
# Launch base + teleoperation
ros2 launch lekiwi_bringup teleop.launch.py

# Use keyboard to control (WASD keys)
```

## 📋 Hardware Specifications

- **Motors:** Feetech STS3215 Servos
- **Voltage:** 12V
- **Baud Rate:** 1,000,000
- **Motor IDs:** 7, 8, 9 (for wheels)
- **Communication:** UART over USB

## 🔧 Development Workflow

### 1. Hardware Validation (Use `motor_debugging/` first)
```bash
cd ../motor_debugging
python scripts/01_find_port.py
python scripts/03_test_single_motor.py --port /dev/ttyUSB0 --motor_id 7
```

### 2. ROS2 Development
```bash
cd ledog_ros2_ws
colcon build --packages-select lekiwi_base
source install/setup.bash
ros2 run lekiwi_base motor_controller_node
```

### 3. Testing
```bash
# Run tests
colcon test

# Check results
colcon test-result --verbose
```

## 📁 Workspace Structure

```
ledog_ros2_ws/
├── README.md                          # This file
├── src/
│   ├── lekiwi_base/                   # Motor control package
│   │   ├── lekiwi_base/
│   │   │   ├── motor_controller_node.py
│   │   │   ├── motor_interface.py
│   │   │   └── kinematics.py
│   │   └── config/
│   │       └── motor_params.yaml
│   │
│   ├── lekiwi_bringup/                # Launch & config
│   │   ├── launch/
│   │   │   ├── robot.launch.py
│   │   │   ├── base_only.launch.py
│   │   │   ├── teleop.launch.py
│   │   │   └── test.launch.py
│   │   └── config/
│   │       ├── motor_config.yaml
│   │       ├── robot_params.yaml
│   │       └── safety_limits.yaml
│   │
│   ├── lekiwi_description/            # URDF & visualization
│   │   ├── urdf/
│   │   │   └── lekiwi.urdf.xacro
│   │   ├── meshes/
│   │   ├── rviz/
│   │   │   └── lekiwi.rviz
│   │   └── launch/
│   │       └── display.launch.py
│   │
│   └── lekiwi_teleop/                 # Teleoperation
│       ├── lekiwi_teleop/
│       │   ├── keyboard_teleop_node.py
│       │   └── joy_teleop_node.py
│       └── config/
│           └── teleop_params.yaml
│
├── build/                             # Build artifacts (gitignored)
├── install/                           # Install space (gitignored)
└── log/                               # Build logs (gitignored)
```

## 🛠️ Common Commands

```bash
# Build all packages
colcon build

# Build specific package
colcon build --packages-select lekiwi_base

# Build with symlink install (for faster iteration)
colcon build --symlink-install

# Clean build
rm -rf build install log

# List all packages
colcon list

# Run specific node
ros2 run lekiwi_base motor_controller_node

# Launch file
ros2 launch lekiwi_bringup robot.launch.py

# View topics
ros2 topic list

# Echo cmd_vel
ros2 topic echo /cmd_vel

# Publish cmd_vel
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2}}"
```

## 🐛 Troubleshooting

### Build Errors
```bash
# Clean and rebuild
rm -rf build install log
colcon build
```

### Missing Dependencies
```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Can't Find Motors
```bash
# Check USB port
ls -l /dev/ttyUSB*

# Fix permissions
sudo chmod 666 /dev/ttyUSB0
```

### Motors Not Responding
1. Verify hardware with `motor_debugging/` scripts first
2. Check motor IDs are configured (7, 8, 9)
3. Verify 12V power supply
4. Check baud rate (1,000,000)

## 📚 Resources

- [ROS2 Documentation](https://docs.ros.org/)
- [LeRobot Documentation](https://huggingface.co/docs/lerobot)
- [LeKiwi Assembly Guide](https://huggingface.co/docs/lerobot/en/lekiwi)
- [Colcon Documentation](https://colcon.readthedocs.io/)

## 🔗 Related Packages

- **motor_debugging/** - Hardware testing and debugging tools (../motor_debugging/)

## 📝 Development Status

- ✅ Workspace structure created
- 🚧 lekiwi_base - In development
- 🚧 lekiwi_bringup - In development
- 📋 lekiwi_description - Planned
- 📋 lekiwi_teleop - Planned

## 📄 License

MIT

---

**Note:** This ROS2 workspace is independent from `motor_debugging/`. Use motor_debugging for hardware validation, then use this workspace for production robot control.

