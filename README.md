# AKROS2

![Project Status](https://img.shields.io/badge/Status-Archived-red)
![ROS 2 Humble](https://img.shields.io/badge/ROS%202%20Humble-Ubuntu%2022.04-blue)
![GitHub License](https://img.shields.io/github/license/adityakamath/akros2)
![X (formerly Twitter) Follow](https://img.shields.io/twitter/follow/kamathsblog)

> **⚠️ This project is archived and no longer in active development.**

ROS 2 stack for the AKROS mecanum-wheeled mobile robot

## Overview

AKROS2 is a comprehensive ROS 2 Humble-based system designed for mecanum-wheeled mobile robots. It provides a complete software stack including robot description, sensor fusion, teleoperation, and system integration capabilities. The platform supports distributed deployment across multiple devices for remote control and visualization.

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

**Launch base station (on remote device):**
```bash
ros2 launch akros2_bringup basestation_launch.py \
  joy_config:=steamdeck
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

## Documentation

📚 **[Complete Design Documentation](docs/design.md)** - Detailed system design, data flow diagrams, and component descriptions

### Key Topics

- **[Robot Description](docs/design.md#akros2_description)** - URDF structure, meshes, and joint configuration
- **[Sensor Fusion](docs/design.md#sensor-fusion-pipeline)** - IMU filtering and EKF-based odometry fusion
- **[Teleoperation](docs/design.md#teleoperation-flow)** - Controller support and command mixing
- **[System Integration](docs/design.md#akros2_bringup)** - Launch file configuration and usage
- **[Data Flow](docs/design.md#data-flow)** - System-level data flow diagrams
- **[Hardware Integration](docs/design.md#hardware-integration)** - Microcontroller communication and sensor interfaces

## Hardware Requirements

### Minimum Configuration
- **Computer:** Raspberry Pi 4 (4GB) or equivalent
- **Microcontroller:** Teensy 4.1 with [expansion board](https://www.tindie.com/products/cburgess129/arduino-teensy41-teensy-41-expansion-board/) running [akros2_firmware](akros2_firmware/)
- **Wheels:** 4x Mecanum wheels and DC motors with quadrature encoders
- **IMU:** 9-DOF inertial measurement unit (accelerometer, gyroscope, magnetometer)
- **Motor Drivers:** 2x Cytron MDD3A motor drivers
- **Power:** Appropriate power system for motors and electronics

### Recommended Sensors
- **LIDAR:** LDLidar LD06 or compatible
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

**Operating System:**
- Ubuntu 22.04 LTS

**ROS 2 Distribution:**
- Humble Hawksbill (recommended)

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

See [Design Documentation](docs/design.md) for detailed component descriptions and data flow diagrams.

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

## Firmware

The [akros2_firmware](akros2_firmware/) package contains micro-ROS firmware for the Teensy 4.1 microcontroller. Based on [linorobot2_hardware](https://github.com/linorobot/linorobot2_hardware), it provides low-level control for mecanum drive kinematics, motor control, and sensor data acquisition.

### Key Features

- **Dual Transport Support** - Configurable serial (USB/UART) or native ethernet (UDP4) communication
- **ROS Domain ID Configuration** - Set ROS_DOMAIN_ID for multi-robot deployments
- **Visual Status Indicators** - Neopixel LEDs show system status and operating mode using FastLED
- **Custom Message Support** - Implements `akros2_msgs/Mode` for operating mode control (stop/auto/teleop)
- **Dual Joint State Publishing** - Separate topics for measured vs. required joint states (velocities and positions)
- **Runtime PID Tuning** - Parameter server allows tuning PID gains (`kp`, `ki`, `kd`, `scale`) without recompilation
- **Coordinate Frame Conversion** - Optional NED to ENU IMU coordinate conversion (REP-103 compliant)

### Hardware Configuration

- **Microcontroller:** Teensy 4.1 with expansion board
- **Motors:** 4x DC motors with quadrature encoders (mecanum wheels)
- **Motor Drivers:** 2x Cytron MDD3A motor drivers
- **IMU:** 9-DOF sensor (accelerometer, gyroscope, magnetometer)
- **Connectivity:** Native ethernet or USB serial

### Communication

The firmware communicates with ROS 2 via micro-ROS agent:

**Published Topics:**

- `/joint_states` - Measured wheel positions and velocities
- `/req_states` - Required (commanded) wheel positions and velocities
- `/imu` - Raw IMU measurements
- `/odometry` - Wheel-based odometry

**Subscribed Topics:**

- `/cmd_vel` - Velocity commands (converted to individual wheel velocities)
- `/mode_status` - Operating mode commands (akros2_msgs/Mode)

**Parameters:**

- `kp`, `ki`, `kd` - PID gain values for motor control
- `scale` - Global velocity scaling factor [0.0-1.0]
- `ned_to_enu` - Enable IMU coordinate conversion (default: false)

See [akros2_firmware/README.md](akros2_firmware/README.md) for detailed firmware documentation and setup instructions.

## Related Repositories

- [akros2_firmware](https://github.com/adityakamath/akros2_firmware) - Upstream micro-ROS firmware repository
- [akros_3d_assets](https://github.com/adityakamath/akros_3d_assets) - 3D models and meshes
