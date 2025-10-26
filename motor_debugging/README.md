# LeKiwi Motor Testing & Debugging

> **Part of the [Seed LeRobot Monorepo](../README.md)**

This package provides tools and scripts to test, debug, and control the LeKiwi mobile base with Feetech STS3215 motors.

## Hardware Specifications

- **Motors:** Feetech STS3215 Servos
- **Voltage:** 12V
- **Torque:** 10 kg·cm
- **Communication:** UART at 1,000,000 baud
- **Wheel Motor IDs:** 7, 8, 9

## Package Structure

```
motor_debugging/
├── README.md                          # This file
├── QUICKSTART.md                      # 5-minute getting started
├── requirements.txt                   # Python dependencies
├── setup.sh                           # Installation script
├── scripts/
│   ├── 01_find_port.py               # Find motor controller USB port
│   ├── 02_configure_motors.py        # Configure motor IDs
│   ├── 03_test_single_motor.py       # Test individual motor
│   ├── 04_test_all_motors.py         # Test all wheel motors
│   ├── 05_basic_teleop.py            # Basic teleoperation control
│   └── 06_motor_diagnostics.py       # Motor health diagnostics
├── config/
│   └── lekiwi_config.yaml            # Robot configuration
└── ros2_examples/                     # Optional ROS2 integration
    ├── motor_controller_node.py
    ├── motor_feedback_node.py
    └── README.md
```

## Quick Start

### 1. Initial Setup (One-time)

```bash
# Make setup script executable
chmod +x setup.sh

# Run setup (creates venv, installs dependencies)
./setup.sh
```

### 2. Activate Virtual Environment

```bash
source venv/bin/activate
```

### 3. Testing Sequence

Follow these steps in order:

#### Step 1: Find Motor Controller Port
```bash
python scripts/01_find_port.py
```
Note the port (e.g., `/dev/ttyUSB0`) for next steps.

#### Step 2: Configure Motor IDs
```bash
python scripts/02_configure_motors.py --port /dev/ttyUSB0
```

#### Step 3: Test Single Motor
```bash
python scripts/03_test_single_motor.py --port /dev/ttyUSB0 --motor_id 7
```

#### Step 4: Test All Motors
```bash
python scripts/04_test_all_motors.py --port /dev/ttyUSB0
```

#### Step 5: Run Diagnostics
```bash
python scripts/06_motor_diagnostics.py --port /dev/ttyUSB0
```

#### Step 6: Basic Teleoperation
```bash
python scripts/05_basic_teleop.py --port /dev/ttyUSB0
```

## Troubleshooting

### No motors detected
- Check power supply (12V battery charged?)
- Verify USB connection
- Try different USB port
- Check cable connections to motors

### Motors not responding
- Verify baud rate (should be 1000000)
- Check motor IDs are correctly set
- Ensure torque is enabled
- Verify adequate power supply

### Permission denied on port
```bash
sudo chmod 666 /dev/ttyUSB0
# Or add user to dialout group:
sudo usermod -a -G dialout $USER
# Then logout and login again
```

### Motor overheating
- Check load on motors
- Verify voltage is correct (12V)
- Allow motors to cool down
- Check for mechanical obstructions

## Using ROS2 (Optional)

If you have ROS2 installed:

```bash
# Source ROS2
source /opt/ros/humble/setup.bash  # or your ROS2 distro

# Run ROS2 motor controller
python ros2_examples/motor_controller_node.py --port /dev/ttyUSB0
```

## Additional Resources

- [LeRobot Documentation](https://huggingface.co/docs/lerobot)
- [LeKiwi Assembly Guide](https://huggingface.co/docs/lerobot/en/lekiwi)
- [Feetech STS3215 Specs](https://openelab.io/products/seeed-studio-lekiwi-kit12v-version-mobile-base-with-3d-printed-parts-and-battery)

## Notes

- Always ensure 12V battery is charged before testing
- Start with individual motor tests before coordinated movement
- Monitor motor temperature during extended use
- Keep motor current below 900mA rated limit

