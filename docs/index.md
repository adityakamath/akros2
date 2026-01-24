---
layout: page
title: AKROS2
subtitle: ROS 2 stack for the AKROS platform
---

![ROS 2 Distro](https://img.shields.io/badge/ROS%202%20Distro-Humble%20(Ubuntu%2022.04)-purple?style=flat&logo=ros&logoSize=auto)
![ROS 2 Package](https://img.shields.io/badge/ROS%202%20Package-akros2-purple?style=flat&logo=ros&logoSize=auto&link=https%3A%2F%2Fgithub.com%2Fadityakamath%2Fakros2&link=https%3A%2F%2Fgithub.com%2Fadityakamath%2Fakros2)
![micro-ROS Firmware](https://img.shields.io/badge/micro--ROS-akros2__firmware-purple?style=flat&logo=ros&logoSize=auto&link=https%3A%2F%2Fgithub.com%2Fadityakamath%2Fakros2_firmware&link=https%3A%2F%2Fgithub.com%2Fadityakamath%2Fakros2_firmware)
![Documentation](https://img.shields.io/badge/Design-grey?style=flat&logo=githubpages&logoSize=auto&link=https%3A%2F%2Fadityakamath.github.io%2Fakros2%2Fdesign&link=https%3A%2F%2Fadityakamath.github.io%2Fakros2%2Fdesign)
![Website](https://img.shields.io/badge/Website-kamathrobotics.com-white?style=flat&logo=hashnode&logoSize=auto&link=https%3A%2F%2Fkamathrobotics.com&link=https%3A%2F%2Fkamathrobotics.com)
![GitHub License](https://img.shields.io/github/license/adityakamath/akros2)
![Project Status](https://img.shields.io/badge/Status-Archived-red)

---

<div style="display: flex; flex-direction: column; gap: 0.9em; margin: 2em 0;">
  <div style="display: flex; align-items: flex-start; gap: 1em; background: #ffffff; padding: 0.9em 1.1em; border-left: 3px solid #666; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.1);">
    <span style="font-size: 1.8em;">🛞</span>
    <div style="flex: 1;">
      <strong>Holonomic Drive Control</strong><br/>
      <span style="font-size: 0.95em;">Mecanum wheel kinematics for omnidirectional motion. Motor controllers, sensor drivers, and EKF-based odometry using robot_localization package fuses IMU and wheel encoder data.</span>
      <br/><br/>
      <a href="https://github.com/adityakamath/akros2/tree/main/akros2_base" target="_blank" style="text-decoration: none; color: inherit;">
        <div style="background: #f8f9fa; padding: 0.6em 0.8em; border-radius: 4px; width: 100%; cursor: pointer;">
          <strong style="font-size: 0.9em;"><code>akros2_base</code></strong>
        </div>
      </a>
    </div>
  </div>
  <div style="display: flex; align-items: flex-start; gap: 1em; background: #ffffff; padding: 0.9em 1.1em; border-left: 3px solid #666; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.1);">
    <span style="font-size: 1.8em;">📐</span>
    <div style="flex: 1;">
      <strong>URDF Robot Model</strong><br/>
      <span style="font-size: 0.95em;">Unified Robot Description Format with Xacro macros, meshes, and kinematics definitions. Defines robot geometry, joints, links, sensors, and physical properties for visualization and simulation.</span>
      <br/><br/>
      <a href="https://github.com/adityakamath/akros2/tree/main/akros2_description" target="_blank" style="text-decoration: none; color: inherit;">
        <div style="background: #f8f9fa; padding: 0.6em 0.8em; border-radius: 4px; width: 100%; cursor: pointer;">
          <strong style="font-size: 0.9em;"><code>akros2_description</code></strong>
        </div>
      </a>
    </div>
  </div>
  <div style="display: flex; align-items: flex-start; gap: 1em; background: #ffffff; padding: 0.9em 1.1em; border-left: 3px solid #666; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.1);">
    <span style="font-size: 1.8em;">🔌</span>
    <div style="flex: 1;">
      <strong>Hardware Abstraction</strong><br/>
      <span style="font-size: 0.95em;">micro-ROS firmware for Teensy 4.1 microcontroller with motor control and sensor interfacing. Communicates with ROS 2 via micro-ROS agent over serial.</span>
      <br/><br/>
      <a href="https://github.com/adityakamath/akros2_firmware" target="_blank" style="text-decoration: none; color: inherit;">
        <div style="background: #f8f9fa; padding: 0.6em 0.8em; border-radius: 4px; width: 100%; cursor: pointer;">
          <strong style="font-size: 0.9em;"><code>akros2_firmware</code></strong>
        </div>
      </a>
    </div>
  </div>
  <div style="display: flex; align-items: flex-start; gap: 1em; background: #ffffff; padding: 0.9em 1.1em; border-left: 3px solid #666; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.1);">
    <span style="font-size: 1.8em;">🎮</span>
    <div style="flex: 1;">
      <strong>Teleoperation</strong><br/>
      <span style="font-size: 0.95em;">Joy node and teleop_twist_joy for gamepad control. Twist command multiplexing for switching between teleoperation and autonomous modes defined by custom Mode message.</span>
      <br/><br/>
      <div style="background: #f8f9fa; padding: 0.6em 0.8em; border-radius: 4px; width: 100%; display: flex; gap: 0.8em;">
        <a href="https://github.com/adityakamath/akros2/tree/main/akros2_teleop" target="_blank" style="text-decoration: none; color: inherit;">
          <strong style="font-size: 0.9em; cursor: pointer;"><code>akros2_teleop</code></strong>
        </a>
        <a href="https://github.com/adityakamath/akros2/tree/main/akros2_msgs" target="_blank" style="text-decoration: none; color: inherit;">
          <strong style="font-size: 0.9em; cursor: pointer;"><code>akros2_msgs</code></strong>
        </a>
      </div>
    </div>
  </div>
  <div style="display: flex; align-items: flex-start; gap: 1em; background: #ffffff; padding: 0.9em 1.1em; border-left: 3px solid #666; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.1);">
    <span style="font-size: 1.8em;">🧩</span>
    <div style="flex: 1;">
      <strong>Modular Architecture</strong><br/>
      <span style="font-size: 0.95em;">Launch file hierarchies, parameter configurations, and system integration packages for bringing up the complete robot stack.</span>
      <br/><br/>
      <div style="display: flex; flex-direction: column; gap: 0.6em; width: 100%;">
        <a href="https://github.com/adityakamath/akros2/tree/main/akros2_bringup" target="_blank" style="text-decoration: none; color: inherit;">
          <div style="background: #f8f9fa; padding: 0.6em 0.8em; border-radius: 4px; cursor: pointer;">
            <strong style="font-size: 0.9em;"><code>akros2_bringup</code></strong><br/>
            <span style="font-size: 0.85em; color: #666;">Launch file hierarchies, parameter configurations</span>
          </div>
        </a>
        <a href="https://github.com/adityakamath/akros2/tree/main/setup" target="_blank" style="text-decoration: none; color: inherit;">
          <div style="background: #f8f9fa; padding: 0.6em 0.8em; border-radius: 4px; cursor: pointer;">
            <strong style="font-size: 0.9em;"><code>setup</code></strong><br/>
            <span style="font-size: 0.85em; color: #666;">System configuration, systemd services, development tools, Steamdeck configuration</span>
          </div>
        </a>
      </div>
    </div>
  </div>
</div>

---

<iframe height="500" style="width: 100%; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.1);" scrolling="no" src="https://codepen.io/adityakamath/embed/qENNvxe?default-tab=result" frameborder="no" loading="lazy">
</iframe>

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

---

<div style="margin: 2em 0; padding: 1em; background: #f0f0f0; border-left: 3px solid #666;">
  <p style="font-size: 0.95em; color: #555; margin: 0;">
    <strong>Project Status:</strong> Archived (2025) | Apache 2.0 License | Open source
  </p>
</div>
