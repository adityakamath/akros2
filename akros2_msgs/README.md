# akros2_msgs

![ROS 2 Galactic](https://img.shields.io/badge/ROS%202%20Galactic-Ubuntu%2020.04-blue)
![ROS 2 Humble](https://img.shields.io/badge/ROS%202%20Humble-Ubuntu%2022.04-blue)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202%20Jazzy-Ubuntu%2024.04-blue)
![GitHub License](https://img.shields.io/github/license/adityakamath/akros2_msgs)
![X (formerly Twitter) Follow](https://img.shields.io/twitter/follow/kamathsblog)

Custom ROS 2 message definitions for the AKROS2 robot.

## Documentation

📚 See the [**AKROS2 Architecture Documentation**](../docs/architecture.md#akros2_msgs) for complete details.

## Messages

### Mode.msg

Robot operation mode status:

```
bool estop      # Emergency stop status
bool auto_t     # Autonomous mode (True) vs Teleop mode (False)
```

**Usage:**
- Published by `joy_mode_handler` node
- Consumed by `twist_mixer` node to determine command routing

For detailed documentation, see [Architecture Documentation](../docs/architecture.md#akros2_msgs).
