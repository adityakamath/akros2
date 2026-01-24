---
layout: page
title: Quick Start
subtitle: Getting started with AKROS2
---

![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20(Ubuntu%2022.04)-purple?style=flat&logo=ros&logoSize=auto)
![Python](https://img.shields.io/badge/Python-3.10+-blue?style=flat&logo=python&logoColor=white)
![C++](https://img.shields.io/badge/C++-17-blue?style=flat&logo=cplusplus&logoColor=white)
![GitHub License](https://img.shields.io/github/license/adityakamath/akros2)
![Website](https://img.shields.io/badge/Website-kamathrobotics.com-white?style=flat&logo=hashnode&logoSize=auto&link=https%3A%2F%2Fkamathrobotics.com&link=https%3A%2F%2Fkamathrobotics.com)
![GitHub License](https://img.shields.io/github/license/adityakamath/akros2)
![Project Status](https://img.shields.io/badge/Status-Archived-red)

> **⚠️ This project is archived and no longer in active development.**

ROS 2 stack for the AKROS mecanum-wheeled mobile robot

## Overview

AKROS2 is a comprehensive ROS 2 Humble-based system designed for the AKROS mecanum-wheeled mobile robot. It provides a complete software stack including robot description, sensor fusion, teleoperation, and system integration capabilities. The platform supports distributed deployment across multiple devices for remote control and visualization.

## Features

- **Mecanum Drive Control** - Full holonomic motion control for 4-wheeled mecanum platforms
- **Sensor Fusion** - IMU and wheel odometry fusion using Extended Kalman Filter
- **Multi-Controller Support** - PS4, Stadia, 8BitDo SN30 Pro, and Steam Deck controllers
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
| **akros2_firmware** | micro-ROS firmware for Teensy microcontroller |
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
  laser:=true \
  camera:=false
```

**Launch base station (on remote control/monitoring device):**
```bash
ros2 launch akros2_bringup basestation_launch.py \
  joy_config:=steamdeck
```

### System Configuration (EXPERIMENTAL)

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

## Documentation

📚 **[Complete Design Documentation](design.md)** - Detailed system design (including the micro-ROS firmware), data flow diagrams, and component descriptions

## Hardware Requirements

### Minimum Configuration
- **Computer:** Raspberry Pi 4 (4GB) or equivalent
- **Microcontroller:** Teensy 4.1 with [expansion board](https://www.tindie.com/products/cburgess129/arduino-teensy41-teensy-41-expansion-board/) running [akros2_firmware](../akros2_firmware/)
- **Wheels:** 4x Mecanum wheels and DC motors with quadrature encoders
- **IMU:** 9-DOF inertial measurement unit (accelerometer, gyroscope, magnetometer)
- **Motor Drivers:** 2x Cytron MDD3A motor drivers
- **Power:** Appropriate power system for motors and electronics

### Recommended (but optional) Sensors
- **LIDAR:** LDLidar LD06 or compatible
- **Camera:** USB camera (v4l2 compatible)

### Supported Controllers
- Sony PS4 DualShock4
- Google Stadia Controller
- 8BitDo SN30 Pro
- Valve Steam Deck

## Platform Support

**Tested On:** Raspberry Pi 4 (4GB/8GB)
**Operating System:** Ubuntu 22.04 LTS
**ROS 2 Distribution:** Humble Hawksbill (recommended)

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

See [Design Documentation](design.md) for detailed component descriptions and details (including firmware) and data flow diagrams.

## Network Configuration

AKROS2 supports distributed deployment across multiple devices. Configure your ROS 2 network settings (DDS domain ID, RMW implementation, etc.) according to your deployment requirements.

## Contributing

This project has been archived and is no longer actively maintained. If you wish to contribute, check out the [Linorobot2](https://github.com/linorobot/linorobot2) project which AKROS2 was based on, and consider submitting issues and PRs there.

## License

Apache License 2.0

## Contact

- **Website:** [kamathrobotics.com](https://kamathrobotics.com)
- **Twitter:** [@kamathsblog](https://twitter.com/kamathsblog)
- **GitHub:** [adityakamath](https://github.com/adityakamath)