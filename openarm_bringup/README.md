# OpenArm Bringup

This package provides launch files to bring up the OpenArm robot system.

## Quick Start

Launch the OpenArm with v1.0 configuration and fake hardware:

```bash
ros2 launch openarm_bringup openarm.launch.py arm_type:=v10 use_fake_hardware:=true
```

## Launch Files

- `openarm.launch.py` - Single arm configuration
- `openarm.bimanual.launch.py` - Dual arm configuration

## Key Parameters

- `arm_type` - Arm type (default: v10)
- `use_fake_hardware` - Use fake hardware instead of real hardware (default: false)
- `can_interface` - CAN interface to use (default: can0)
- `robot_controller` - Controller type: `joint_trajectory_controller` or `forward_position_controller`

## What Gets Launched

- Robot state publisher
- Controller manager with ros2_control
- Joint state broadcaster
- Robot controller (joint trajectory or forward position)
- Gripper controller
- RViz2 visualization
