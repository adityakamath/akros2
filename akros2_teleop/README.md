# akros2_teleop

![ROS 2 Humble](https://img.shields.io/badge/ROS%202%20Humble-Ubuntu%2022.04-blue)
![GitHub License](https://img.shields.io/github/license/adityakamath/akros2_teleop)
![X (formerly Twitter) Follow](https://img.shields.io/twitter/follow/kamathsblog)

Teleoperation nodes and command mixing for the AKROS2 robot.

## Documentation

📚 See the [**AKROS2 Architecture Documentation**](../docs/architecture.md#akros2_teleop) for complete details including:

- Node descriptions (joy_mode_handler, twist_mixer, drive_node)
- Supported controllers and configurations
- Launch file arguments
- Configuration file structure
- Data flow diagrams

## Quick Reference

**Launch teleoperation:**
```bash
ros2 launch akros2_teleop drive_launch.py joy_config:=steamdeck
```

**Launch arguments:**
- `joy` (Default: `True`) - Enable/disable joy-related packages
- `joy_config` (Default: `steamdeck`) - Controller configuration (ps4/stadia/sn30pro/steamdeck)
- `executor` - Run as composed node or separate nodes

## Supported Controllers

- Sony PS4 DualShock4
- Google Stadia Controller
- 8BitDo SN30 Pro
- Valve Steam Deck

## Key Nodes

- **joy_mode_handler** - Manages robot mode (stop/auto/teleop)
- **twist_mixer** - Mixes teleop and autonomous commands
- **drive_node** - Composed executable running both nodes

## Configuration

Each controller has three configuration files in `config/`:
- `<controller>_mapping.yaml` - Button/axis mappings
- `<controller>_mode_config.yaml` - Mode button mappings
- `<controller>_twist_config.yaml` - Twist command scaling

For detailed documentation, see [Architecture Documentation](../docs/architecture.md#akros2_teleop).
