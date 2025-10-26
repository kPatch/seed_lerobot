# LeRobot Policy Controller - Quick Start Guide

Get the policy controller up and running in 5 minutes!

## Prerequisites

- ✅ ROS2 installed and sourced
- ✅ Conda environment with LeRobot installed (`lerobot`)
- ✅ Robot hardware connected (`/dev/ttyACM0`)
- ✅ Cameras available (indexes 0 and 2)

## Step 1: Build the Package

```bash
cd ~/ledog_ros2_ws
colcon build --packages-select ledog_policy_controller
source install/setup.bash
```

## Step 2: Configure (Optional)

If your conda is in a different location:

```bash
# Check your conda location
which conda
# Should show: /home/user/anaconda3/bin/conda
# or: /home/user/miniconda3/bin/conda
```

Edit parameters if needed:
```bash
nano src/ledog_policy_controller/config/policy_controller_params.yaml
```

## Step 3: Start the Node

```bash
ros2 launch ledog_policy_controller policy_controller.launch.py
```

Or with custom parameters:
```bash
ros2 launch ledog_policy_controller policy_controller.launch.py \
    conda_env_name:=lerobot \
    conda_base_path:=~/anaconda3
```

## Step 4: Control the Policy

Open a new terminal and source the workspace:
```bash
source ~/ledog_ros2_ws/install/setup.bash
```

### Check Status
```bash
ros2 service call /ledog/get_policy_status std_srvs/srv/Trigger
```

### Start Recording
```bash
ros2 service call /ledog/start_policy std_srvs/srv/Trigger
```

### Stop Recording
```bash
ros2 service call /ledog/stop_policy std_srvs/srv/Trigger
```

## Complete Workflow Example

```bash
# Terminal 1: Start the controller
ros2 launch ledog_policy_controller policy_controller.launch.py

# Terminal 2: Control the policy
source ~/ledog_ros2_ws/install/setup.bash

# Check initial status
ros2 service call /ledog/get_policy_status std_srvs/srv/Trigger
# Response: "idle (no policy has been started)"

# Start the policy
ros2 service call /ledog/start_policy std_srvs/srv/Trigger
# Response: "LeRobot policy started successfully (PID: 12345)"

# Watch the logs in Terminal 1 for LeRobot output...

# When done, stop the policy
ros2 service call /ledog/stop_policy std_srvs/srv/Trigger
# Response: "Policy stopped gracefully (PID: 12345, exit code: 0)"
```

## Troubleshooting Quick Fixes

### "Policy process started but exited immediately"

Check conda environment:
```bash
conda env list | grep lerobot
```

If missing, create it:
```bash
conda create -n lerobot python=3.10
conda activate lerobot
pip install lerobot[feetech]
```

### "Permission denied" on /dev/ttyACM0

Add user to dialout group:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Cameras not found

Check available cameras:
```bash
ls /dev/video*
v4l2-ctl --list-devices
```

## Testing Without Hardware

To test the node without hardware, temporarily modify the command in:
```
ledog_policy_controller/policy_controller_node.py
```

Replace the `cmd` in `_run_policy_thread()` with:
```python
cmd = "bash -c 'for i in {1..10}; do echo \"Test iteration $i\"; sleep 1; done'"
```

Then rebuild and test:
```bash
colcon build --packages-select ledog_policy_controller
source install/setup.bash
ros2 launch ledog_policy_controller policy_controller.launch.py
```

## Next Steps

- Read the full [README.md](README.md) for detailed documentation
- Integrate with other LeKiwi packages
- Customize the LeRobot command for your use case
- Set up automated policy recording workflows

---

**Need more help?** See the full README.md or check the troubleshooting section.

