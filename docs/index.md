---
layout: page
title: AKROS2
subtitle: ROS 2 software stack for the AKROS mecanum-wheeled mobile robot
---

![ROS 2 Distro](https://img.shields.io/badge/ROS%202%20Distro-Humble%20(Ubuntu%2022.04)-purple?style=flat&logo=ros&logoSize=auto)
![ROS 2 Package](https://img.shields.io/badge/ROS%202%20Package-akros2-purple?style=flat&logo=ros&logoSize=auto&link=https%3A%2F%2Fgithub.com%2Fadityakamath%2Fakros2&link=https%3A%2F%2Fgithub.com%2Fadityakamath%2Fakros2)
![micro-ROS Firmware](https://img.shields.io/badge/micro--ROS-akros2__firmware-purple?style=flat&logo=ros&logoSize=auto&link=https%3A%2F%2Fgithub.com%2Fadityakamath%2Fakros2_firmware&link=https%3A%2F%2Fgithub.com%2Fadityakamath%2Fakros2_firmware)
![Documentation](https://img.shields.io/badge/Design-grey?style=flat&logo=githubpages&logoSize=auto&link=https%3A%2F%2Fadityakamath.github.io%2Fakros2%2Fdesign&link=https%3A%2F%2Fadityakamath.github.io%2Fakros2%2Fdesign)
![Website](https://img.shields.io/badge/Website-kamathrobotics.com-white?style=flat&logo=hashnode&logoSize=auto&link=https%3A%2F%2Fkamathrobotics.com&link=https%3A%2F%2Fkamathrobotics.com)
![GitHub License](https://img.shields.io/github/license/adityakamath/akros2)
![Project Status](https://img.shields.io/badge/Status-Archived-red)


## Capabilities

<div style="display: flex; flex-direction: column; gap: 1.2em; margin: 2em 0;">
  <div style="display: flex; align-items: flex-start; gap: 1em; background: #f8f9fa; padding: 1.2em; border-left: 3px solid #404040; border-radius: 6px;">
    <span style="font-size: 2em;">🛞</span>
    <div>
      <strong>Holonomic Drive Control</strong><br/>
      Mecanum wheel kinematics for omnidirectional motion. Converts twist commands to individual wheel velocities.
    </div>
  </div>
  <div style="display: flex; align-items: flex-start; gap: 1em; background: #f8f9fa; padding: 1.2em; border-left: 3px solid #404040; border-radius: 6px;">
    <span style="font-size: 2em;">🧭</span>
    <div>
      <strong>Sensor Fusion</strong><br/>
      EKF-based odometry using robot_localization package. Fuses IMU and wheel encoder data.
    </div>
  </div>
  <div style="display: flex; align-items: flex-start; gap: 1em; background: #f8f9fa; padding: 1.2em; border-left: 3px solid #404040; border-radius: 6px;">
    <span style="font-size: 2em;">🔌</span>
    <div>
      <strong>Hardware Abstraction</strong><br/>
      micro-ROS agent for Teensy microcontroller communication via serial.
    </div>
  </div>
  <div style="display: flex; align-items: flex-start; gap: 1em; background: #f8f9fa; padding: 1.2em; border-left: 3px solid #404040; border-radius: 6px;">
    <span style="font-size: 2em;">🎮</span>
    <div>
      <strong>Teleoperation</strong><br/>
      Standard ROS 2 joy/teleop nodes for gamepad control. Twist command mixing for teleop/auto mode switching.
    </div>
  </div>
  <div style="display: flex; align-items: flex-start; gap: 1em; background: #f8f9fa; padding: 1.2em; border-left: 3px solid #404040; border-radius: 6px;">
    <span style="font-size: 2em;">🧩</span>
    <div>
      <strong>Modular Architecture</strong><br/>
      Separate packages for description, base control, teleoperation, and system integration.
    </div>
  </div>
</div>

---

## Playground

<div style="position: relative; padding-bottom: 75%; height: 0; overflow: hidden; max-width: 100%; margin: 2em 0; box-shadow: 0 4px 6px rgba(0,0,0,0.1); border-radius: 8px;">
  <iframe style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; border-radius: 8px;" scrolling="no" title="AKROS2 - 3D Robot Visualization" src="https://codepen.io/adityakamath/embed/qENNvxe?default-tab=result" frameborder="no" loading="lazy" allowtransparency="true" allowfullscreen="true">
    See the Pen <a href="https://codepen.io/adityakamath/pen/qENNvxe">AKROS2 - 3D Robot Visualization</a> by Aditya Kamath (<a href="https://codepen.io/adityakamath">@adityakamath</a>) on <a href="https://codepen.io">CodePen</a>.
  </iframe>
</div>

---

## Requirements

**Software Configuration:**
- ROS 2 Humble Hawksbill
- Ubuntu 22.04 LTS
- Python 3.10+ and C++17

**Hardware Configuration:**
- Raspberry Pi 4 (4GB minimum) or equivalent ARM64/x86_64 system
- Teensy microcontroller running [micro-ROS firmware](https://github.com/adityakamath/akros2_firmware) and connected to the computer via serial (UART/USB)
- 4x mecanum wheels with quadrature encoders
- 9-DOF IMU (accelerometer, gyroscope, magnetometer)
- 2x Cytron MD33A motor drivers

**Optional Sensors:**
- LDLidar LD06 (or compatible 2D laser scanner)
- USB camera (v4l2 compatible)

## Architecture

| Package | Function |
|---------|----------|
| `akros2_description` | URDF/Xacro robot models, meshes, kinematics definitions |
| `akros2_base` | Motor controllers, sensor drivers, EKF localization |
| `akros2_teleop` | Joy node, teleop_twist_joy node, velocity command multiplexing |
| `akros2_bringup` | Launch file hierarchies, parameter configurations |
| `akros2_msgs` | Custom messagesinterfaces |
| `setup` | System configuration, systemd services, development tools, Steamdeck configuration |

---

<div style="margin: 2em 0; padding: 1em; background: #f0f0f0; border-left: 3px solid #666;">
  <p style="font-size: 0.95em; color: #555; margin: 0;">
    <strong>Project Status:</strong> Archived (2025) | Apache 2.0 License | Open source
  </p>
</div>
