# Motor Testing Scripts - Now With Real Control!

These scripts now use `scservo_sdk` (via the motor_interface module) to actually communicate with and control Feetech STS3215 motors.

## What Changed

✅ **Now using `scservo_sdk`** for actual motor communication
✅ **Reads real motor data** (position, temperature, voltage, current, load)
✅ **Sends actual movement commands** to motors
✅ **Enables torque** automatically
✅ **Proper connection/disconnection** handling

## Usage

### 1. Find Port
```bash
python scripts/01_find_port.py
```

### 2. Test Single Motor (UPDATED - NOW WORKS!)
```bash
python scripts/03_test_single_motor.py --port /dev/ttyACM0 --motor_id 7
```

**This will:**
- Connect to the serial port
- Ping motor to verify communication
- Enable torque on the motor
- Read actual motor state (position, temp, voltage, etc.)
- **Actually move the motor** through a test sequence
- Display real diagnostics

### Skip Movement Test
```bash
python scripts/03_test_single_motor.py --port /dev/ttyACM0 --motor_id 7 --skip_movement
```

## Motor Interface Module

The `motor_interface.py` module provides a clean interface to control Feetech motors:

### Key Features:
- `connect()` / `disconnect()` - Manage serial connection
- `ping(motor_id)` - Check if motor responds
- `enable_torque(motor_id)` - Enable motor control
- `disable_torque(motor_id)` - Make motor limp
- `read_position(motor_id)` - Get current position
- `write_position(motor_id, position)` - Move motor
- `read_all_data(motor_id)` - Get complete motor state

### Control Table Addresses (STS3215):
- `40` - Torque Enable (0=off, 1=on)
- `42` - Goal Position (0-4095)
- `56` - Present Position
- `65` - Present Voltage (÷10 for actual volts)
- `66` - Present Temperature (°C)
- `69` - Present Current (mA)

## Testing Checklist

✅ Port detected: `/dev/ttyACM0`
✅ Motor responds to ping
✅ Torque enabled
✅ Can read position
✅ Can read temperature
✅ Can read voltage
✅ Can write position
✅ Motor actually moves
✅ Position feedback matches command

## Troubleshooting

### Motor not responding
1. Check 12V power supply is connected
2. Verify motor is connected to bus
3. Try different motor ID (default: 7)
4. Check port permissions: `sudo chmod 666 /dev/ttyACM0`

### Motor doesn't move
1. Check torque is enabled (script does this automatically)
2. Ensure motor can move freely (no mechanical obstruction)
3. Verify motor ID is correct
4. Check voltage is adequate (>10.5V)

### Permission denied
```bash
sudo usermod -a -G dialout $USER
# Then logout and login again
```

## Next Steps

- ✅ Script 03 (single motor) - **NOW FUNCTIONAL**
- 🚧 Script 04 (all motors) - Needs update
- 🚧 Script 05 (teleoperation) - Needs update
- 🚧 Script 06 (diagnostics) - Needs update

After testing, these scripts can be integrated into ROS2 nodes in `ledog_ros2_ws/`.

