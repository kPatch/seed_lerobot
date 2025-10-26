# LeKiwi Quick Start Guide

> **Part of the [Seed LeRobot Monorepo](../README.md)**

Get your LeKiwi mobile base up and running in 5 minutes!

## Prerequisites

- ✅ LeKiwi mobile base with Feetech STS3215 motors
- ✅ 12V battery (charged)
- ✅ USB cable connection to motor controller
- ✅ Python 3.8+ installed
- ✅ Linux, macOS, or Windows with WSL

## 5-Minute Setup

### Step 1: Navigate to Motor Debugging Package (10 seconds)

```bash
cd /home/kpatch/development/seed_lerobot/motor_debugging
```

### Step 2: Run Setup Script (2-3 minutes)

```bash
chmod +x setup.sh
./setup.sh
```

This will:
- Create a virtual environment
- Install all dependencies (lerobot, feetech-servo-sdk, etc.)
- Set up project directories

### Step 3: Activate Virtual Environment (5 seconds)

```bash
source venv/bin/activate
```

### Step 4: Connect Hardware (30 seconds)

1. Ensure 12V battery is connected and charged
2. Connect motor controller to your computer via USB
3. Power on the robot

### Step 5: Find USB Port (30 seconds)

```bash
python scripts/01_find_port.py
```

Note the port (e.g., `/dev/ttyUSB0`)

## First Test

### Option A: Test Individual Motor (Recommended First)

```bash
# Configure motor ID 7 (first wheel)
python scripts/02_configure_motors.py --port /dev/ttyUSB0 --motor_id 7

# Test it
python scripts/03_test_single_motor.py --port /dev/ttyUSB0 --motor_id 7
```

### Option B: Configure All Motors

```bash
python scripts/02_configure_motors.py --port /dev/ttyUSB0 --auto
```

### Option C: Test All Motors

```bash
python scripts/04_test_all_motors.py --port /dev/ttyUSB0
```

## Common Issues

### "Permission denied" on port

```bash
sudo chmod 666 /dev/ttyUSB0
# OR add yourself to dialout group:
sudo usermod -a -G dialout $USER
# Then logout and login again
```

### "No module named 'lerobot'"

```bash
# Make sure virtual environment is activated
source venv/bin/activate

# If still fails, reinstall
pip install 'lerobot[feetech]'
```

### Motors not responding

1. Check 12V power supply
2. Verify USB connection
3. Check motor IDs are configured (7, 8, 9)
4. Test with single motor first

## Next Steps

Once basic tests work:

1. **Run Diagnostics**: Monitor motor health
   ```bash
   python scripts/06_motor_diagnostics.py --port /dev/ttyUSB0 --log
   ```

2. **Try Teleoperation**: Control with keyboard
   ```bash
   python scripts/05_basic_teleop.py --port /dev/ttyUSB0
   ```

3. **Use ROS2** (if installed): Advanced control
   ```bash
   python ros2_examples/motor_controller_node.py
   ```

## Getting Help

- 📖 Full documentation: See `README.md`
- 🔧 Troubleshooting: See `README.md` troubleshooting section
- 🌐 LeRobot docs: https://huggingface.co/docs/lerobot
- 🤖 LeKiwi guide: https://huggingface.co/docs/lerobot/en/lekiwi

## Safety Reminders

- ⚠️ Always ensure clear space for robot movement
- ⚠️ Keep emergency stop accessible
- ⚠️ Monitor motor temperature during extended use
- ⚠️ Never exceed 12V power supply
- ⚠️ Test single motors before coordinated movement

## Test Sequence Checklist

- [ ] Setup script completed
- [ ] Virtual environment activated
- [ ] USB port identified
- [ ] Motor 7 configured and tested
- [ ] Motor 8 configured and tested
- [ ] Motor 9 configured and tested
- [ ] All motors tested together
- [ ] Diagnostics running successfully
- [ ] Teleoperation working

**Status: Ready to go!** 🚀

