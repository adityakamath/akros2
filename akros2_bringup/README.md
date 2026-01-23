# akros2_bringup

![ROS 2 Humble](https://img.shields.io/badge/ROS%202%20Humble-Ubuntu%2022.04-blue)
![GitHub License](https://img.shields.io/github/license/adityakamath/akros2_bringup)
![X (formerly Twitter) Follow](https://img.shields.io/twitter/follow/kamathsblog)

System-level launch files and integration for the AKROS2 mecanum-wheeled robot.

## Documentation

📚 See the [**AKROS2 Architecture Documentation**](../docs/architecture.md#akros2_bringup) for complete details including:

- All launch files and their arguments
- Visualization configurations
- System integration details
- Foxglove Studio configuration
- Recommended launch practices

## Quick Reference

**Launch the robot:**
```bash
ros2 launch akros2_bringup bringup_launch.py
```

**Launch with custom configuration:**
```bash
ros2 launch akros2_bringup bringup_launch.py \
  joy_config:=steamdeck \
  viz_config:=foxglove \
  laser:=true \
  camera:=false
```

**Launch base station:**
```bash
ros2 launch akros2_bringup basestation_launch.py \
  joy_config:=steamdeck \
  viz_config:=foxglove
```

## Launch Files

- **bringup_launch.py** - Main robot bringup
- **bringup_local_launch.py** - Bringup with local teleoperation
- **basestation_launch.py** - Remote base station
- **viz_launch.py** - Visualization packages

## Common Launch Arguments

- `joy_config` - Controller configuration (ps4/stadia/sn30pro/steamdeck/none)
- `viz_config` - Visualization config (foxglove/rosbridge/rviz/none)
- `desc` - Enable URDF description
- `laser` - Enable LIDAR
- `camera` - Enable camera
- `control` - Enable low-level control
- `fusion` - Enable sensor fusion
- `teleop` - Enable teleoperation

**Note:** Launch `control` and `camera` separately in different terminals due to timing issues.

For detailed documentation, see [Architecture Documentation](../docs/architecture.md#akros2_bringup).
