# AKROS2

![ROS 2 Humble](https://img.shields.io/badge/ROS%202%20Humble-Ubuntu%2022.04-blue)
![GitHub License](https://img.shields.io/github/license/adityakamath/akros2)
![X (formerly Twitter) Follow](https://img.shields.io/twitter/follow/kamathsblog)

ROS 2 software stack for the AKROS mecanum-wheeled mobile robot platform, inspired by Linorobot2.

## Overview

AKROS2 is a comprehensive ROS 2 Humble-based system designed for mecanum-wheeled mobile robots. It provides a complete software stack including robot description, sensor fusion, teleoperation, and system integration capabilities. The platform supports distributed deployment across multiple devices and offers flexible visualization options through Foxglove Studio, RViz, and ROSBridge.

## Features

- **Mecanum Drive Control** - Full holonomic motion control for 4-wheeled mecanum platforms
- **Sensor Fusion** - IMU and wheel odometry fusion using Extended Kalman Filter
- **Multi-Controller Support** - PS4, Stadia, 8BitDo SN30 Pro, and Steam Deck controllers
- **Flexible Visualization** - Foxglove Studio, RViz2, and ROSBridge support
- **micro-ROS Integration** - Low-level control via serial communication with Teensy microcontroller
- **Modular Architecture** - Independent packages for easy customization and extension

## Packages

| Package | Description |
|---------|-------------|
| **akros2_description** | Robot URDF/Xacro descriptions, meshes, and kinematics |
| **akros2_base** | Core drivers, sensors, filters, and sensor fusion |
| **akros2_teleop** | Teleoperation nodes and command mixing |
| **akros2_msgs** | Custom ROS 2 message definitions |
| **akros2_bringup** | System-level launch files and integration |
| **setup** | System configuration, services, and development tools |

## Quick Start

### Installation

1. **Clone the repository:**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/adityakamath/akros2
   ```

2. **Install dependencies:**
   ```bash
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the workspace:**
   ```bash
   colcon build --symlink-install
   ```

4. **Source the workspace:**
   ```bash
   source ~/ros2_ws/install/setup.bash
   ```

### Basic Usage

**Launch the robot (on robot device):**
```bash
ros2 launch akros2_bringup bringup_launch.py
```

**Launch with specific configuration:**
```bash
ros2 launch akros2_bringup bringup_launch.py \
  joy_config:=steamdeck \
  viz_config:=foxglove \
  laser:=true \
  camera:=false
```

**Launch base station (on remote device):**
```bash
ros2 launch akros2_bringup basestation_launch.py \
  joy_config:=steamdeck \
  viz_config:=foxglove
```

### Configuration

Copy the bash configuration from setup:
```bash
cat src/akros2/setup/.bashrc >> ~/.bashrc
source ~/.bashrc
```

This provides convenient aliases:
- `bringup` - Launch robot with minimal sensors
- `bringup_local` - Launch robot with local control
- `basestation` - Launch base station
- `control` - Launch low-level control only
- `viz` - Launch visualization only

## Documentation

📚 **[Complete Architecture Documentation](docs/architecture.md)** - Detailed system architecture, data flow diagrams, and component descriptions

### Key Topics

- **[Robot Description](docs/architecture.md#akros2_description)** - URDF structure, meshes, and joint configuration
- **[Sensor Fusion](docs/architecture.md#sensor-fusion-pipeline)** - IMU filtering and EKF-based odometry fusion
- **[Teleoperation](docs/architecture.md#teleoperation-flow)** - Controller support and command mixing
- **[System Integration](docs/architecture.md#akros2_bringup)** - Launch file configuration and usage
- **[Data Flow](docs/architecture.md#data-flow)** - System-level data flow diagrams
- **[Hardware Integration](docs/architecture.md#hardware-integration)** - Microcontroller communication and sensor interfaces

## Hardware Requirements

### Minimum Configuration
- **Computer:** Raspberry Pi 4 (4GB) or equivalent
- **Microcontroller:** Teensy with [micro-ROS firmware](https://github.com/adityakamath/akros2_firmware)
- **Wheels:** 4x Mecanum wheels with encoders
- **IMU:** 9-DOF inertial measurement unit
- **Power:** Appropriate power system for motors and electronics

### Recommended Sensors
- **LIDAR:** LD06 or compatible
- **Camera:** USB camera (v4l2 compatible)

### Supported Controllers
- Sony PS4 DualShock4
- Google Stadia Controller
- 8BitDo SN30 Pro
- Valve Steam Deck

## Platform Support

**Tested On:**
- Raspberry Pi 4 (4GB/8GB)
- Raspberry Pi Zero 2 W
- Valve Steam Deck (Ubuntu 22.04 via Distrobox)
- WSL2 (partial support)

**Operating System:**
- Ubuntu 22.04 LTS (primary)
- Ubuntu 20.04 LTS (limited)

**ROS 2 Distribution:**
- Humble Hawksbill (recommended)

## Development Tools

The system includes integrations with popular robotics development tools:
- **Foxglove Studio** - Advanced web-based visualization
- **RViz2** - Traditional ROS visualization

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        akros2_bringup                           │
│                    (System Integration)                          │
└───────────┬─────────────────────────────────────────────────────┘
            │
    ┌───────┴───────┬──────────┬──────────┬──────────┬───────────┐
    │               │          │          │          │           │
┌───▼────┐  ┌───────▼──┐  ┌───▼─────┐ ┌─▼──────┐ ┌─▼──────┐ ┌──▼──────┐
│akros2_ │  │ akros2_  │  │akros2_  │ │akros2_ │ │akros2_ │ │ akros2_ │
│descrip-│  │   base   │  │teleop   │ │ msgs   │ │ setup  │ │firmware │
│tion    │  │          │  │         │ │        │ │        │ │(ext)    │
└────────┘  └──────────┘  └─────────┘ └────────┘ └────────┘ └─────────┘
```

See [Architecture Documentation](docs/architecture.md) for detailed component descriptions and data flow diagrams.

## Network Configuration

AKROS2 supports distributed deployment across multiple devices. Configure your ROS 2 network settings (DDS domain ID, RMW implementation, etc.) according to your deployment requirements.

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## License

Apache License 2.0

## Acknowledgments

- Inspired by [Linorobot2](https://github.com/linorobot/linorobot2)
- Built with ROS 2 Humble Hawksbill
- Utilizes micro-ROS for microcontroller integration

## Contact

- **Website:** [kamathrobotics.com](https://kamathrobotics.com)
- **Twitter:** [@kamathsblog](https://twitter.com/kamathsblog)
- **GitHub:** [adityakamath](https://github.com/adityakamath)

## Related Repositories

- [akros2_firmware](https://github.com/adityakamath/akros2_firmware) - micro-ROS firmware for Teensy microcontroller
- [akros_3d_assets](https://github.com/adityakamath/akros_3d_assets) - 3D models and meshes
