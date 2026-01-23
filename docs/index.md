---
layout: default
title: AKROS2 - Interactive 3D Visualization
bigimg: /akros2/assets/img/header_image.png
---

# AKROS2

![ROS 2 Humble](https://img.shields.io/badge/ROS%202%20Humble-Ubuntu%2022.04-blue)
![GitHub License](https://img.shields.io/github/license/adityakamath/akros2)

ROS 2 software stack for the AKROS mecanum-wheeled mobile robot platform.

## Interactive 3D Visualization

Try out the AKROS2 mecanum drive kinematics in your browser:

<iframe height="600" style="width: 100%;" scrolling="no" title="AKROS2 - 3D Robot Visualization" src="https://codepen.io/adityakamath/embed/qENNvxe?default-tab=result" frameborder="no" loading="lazy" allowtransparency="true" allowfullscreen="true">
  See the Pen <a href="https://codepen.io/adityakamath/pen/qENNvxe">AKROS2 - 3D Robot Visualization</a> by Aditya Kamath (<a href="https://codepen.io/adityakamath">@adityakamath</a>) on <a href="https://codepen.io">CodePen</a>.
</iframe>

## About AKROS2

AKROS2 is a comprehensive ROS 2 Humble-based system designed for mecanum-wheeled mobile robots. It provides:

- **Mecanum Drive Control** - Full holonomic motion control for 4-wheeled mecanum platforms
- **Sensor Fusion** - IMU and wheel odometry fusion using Extended Kalman Filter
- **Multi-Controller Support** - PS4, Stadia, 8BitDo SN30 Pro, and Steam Deck controllers
- **Flexible Visualization** - Foxglove Studio, RViz2, and ROSBridge support
- **micro-ROS Integration** - Low-level control via serial communication with Teensy microcontroller
- **Modular Architecture** - Independent packages for easy customization and extension

## Documentation

- [Architecture Documentation](architecture.md) - Complete system architecture and technical details
- [GitHub Repository](https://github.com/adityakamath/akros2) - Source code and installation instructions

## Hardware

- **Microcontroller:** Teensy with [micro-ROS firmware](https://github.com/adityakamath/akros2_firmware)
- **Wheels:** 4x Mecanum wheels with encoders
- **IMU:** 9-DOF inertial measurement unit
- **LIDAR:** LD06 or compatible (optional)
- **Camera:** USB camera (optional)

## Links

- [Kamath Robotics](https://kamathrobotics.com)
- [GitHub: @adityakamath](https://github.com/adityakamath)
- [Twitter: @kamathsblog](https://twitter.com/kamathsblog)
