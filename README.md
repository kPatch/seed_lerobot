# Seed LeRobot - LeKiwi Development

Development repository for LeKiwi mobile base robotics projects.

## 📦 Packages

### 🔧 [motor_debugging/](motor_debugging/)

**Hardware Testing & Validation Tool**

Test, debug, and validate LeKiwi mobile base motors (Feetech STS3215).

**Quick Start:**
```bash
cd motor_debugging
./setup.sh
source venv/bin/activate
python scripts/01_find_port.py
```

**Use When:** Setting up hardware, debugging motor issues, configuring motor IDs

**Documentation**: See [motor_debugging/README.md](motor_debugging/README.md)

---

### 🤖 [ledog_ros2_ws/](ledog_ros2_ws/)

**Production ROS2 Control System**

Full ROS2 workspace for LeKiwi mobile base control and integration.

**Packages:**
- `lekiwi_base` - Motor control & hardware interface
- `lekiwi_bringup` - Launch files & configuration
- `lekiwi_description` - URDF robot model & visualization
- `lekiwi_teleop` - Keyboard & joystick teleoperation

**Quick Start:**
```bash
cd ledog_ros2_ws
colcon build
source install/setup.bash
ros2 launch lekiwi_bringup robot.launch.py
```

**Use When:** Running the robot, integrating with navigation, production deployment

**Documentation**: See [ledog_ros2_ws/README.md](ledog_ros2_ws/README.md)

---

## Repository Structure

```
seed_lerobot/
├── README.md                    # This file
├── .gitignore                   # Git ignore patterns
│
├── motor_debugging/             # 🔧 Hardware testing tool
│   ├── README.md               # Complete documentation
│   ├── QUICKSTART.md           # 5-minute setup guide
│   ├── requirements.txt        # Dependencies
│   ├── setup.sh                # Automated setup
│   ├── config/                 # Configurations
│   ├── scripts/                # Testing scripts (01-06)
│   └── ros2_examples/          # ROS2 integration examples
│
└── ledog_ros2_ws/             # 🤖 ROS2 workspace
    ├── README.md               # Workspace documentation
    └── src/                    # ROS2 packages
        ├── lekiwi_base/        # Motor control
        ├── lekiwi_bringup/     # Launch & config
        ├── lekiwi_description/ # URDF & visualization
        └── lekiwi_teleop/      # Teleoperation
```

## 🔄 Workflow

### Phase 1: Hardware Validation
```bash
cd motor_debugging
./setup.sh && source venv/bin/activate
python scripts/01_find_port.py
python scripts/02_configure_motors.py --port /dev/ttyUSB0 --auto
python scripts/04_test_all_motors.py --port /dev/ttyUSB0
```
✅ Verify motors work, IDs configured

### Phase 2: ROS2 Development & Operation
```bash
cd ledog_ros2_ws
colcon build && source install/setup.bash
ros2 launch lekiwi_bringup robot.launch.py
```
✅ Production robot control

## 📋 Key Points

- **Independent Packages**: No dependencies between motor_debugging and ledog_ros2_ws
- **Different Purposes**: motor_debugging = testing tool, ledog_ros2_ws = production system
- **Both Use Same SDK**: `lerobot[feetech]` installed independently in each
- **Sequential Workflow**: Validate hardware first, then use ROS2 for operation

## Resources

- [LeRobot Documentation](https://huggingface.co/docs/lerobot)
- [LeKiwi Assembly Guide](https://huggingface.co/docs/lerobot/en/lekiwi)
- [Feetech STS3215 Specs](https://openelab.io/products/seeed-studio-lekiwi-kit12v-version-mobile-base-with-3d-printed-parts-and-battery)
- [ROS2 Documentation](https://docs.ros.org/)
