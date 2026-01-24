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
  <div style="display: flex; align-items: flex-start; gap: 1em; background: #ffffff; padding: 0.9em 1.1em; border-left: 3px solid #a855f7; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.1);">
    <span style="font-size: 1.8em;">🛞</span>
    <div>
      <strong>Holonomic Drive Control</strong><br/>
      <span style="font-size: 0.95em;">Mecanum wheel kinematics for omnidirectional motion. Converts twist commands to individual wheel velocities.</span>
    </div>
  </div>
  <div style="display: flex; align-items: flex-start; gap: 1em; background: #ffffff; padding: 0.9em 1.1em; border-left: 3px solid #06b6d4; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.1);">
    <span style="font-size: 1.8em;">🧭</span>
    <div>
      <strong>Sensor Fusion</strong><br/>
      <span style="font-size: 0.95em;">EKF-based odometry using robot_localization package. Fuses IMU and wheel encoder data.</span>
    </div>
  </div>
  <div style="display: flex; align-items: flex-start; gap: 1em; background: #ffffff; padding: 0.9em 1.1em; border-left: 3px solid #22c55e; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.1);">
    <span style="font-size: 1.8em;">🔌</span>
    <div>
      <strong>Hardware Abstraction</strong><br/>
      <span style="font-size: 0.95em;">micro-ROS agent for Teensy microcontroller communication via serial.</span>
    </div>
  </div>
  <div style="display: flex; align-items: flex-start; gap: 1em; background: #ffffff; padding: 0.9em 1.1em; border-left: 3px solid #ec4899; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.1);">
    <span style="font-size: 1.8em;">🎮</span>
    <div>
      <strong>Teleoperation</strong><br/>
      <span style="font-size: 0.95em;">Standard ROS 2 joy/teleop nodes for gamepad control. Twist command mixing for teleop/auto mode switching.</span>
    </div>
  </div>
  <div style="display: flex; align-items: flex-start; gap: 1em; background: #ffffff; padding: 0.9em 1.1em; border-left: 3px solid #f59e0b; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.1);">
    <span style="font-size: 1.8em;">🧩</span>
    <div>
      <strong>Modular Architecture</strong><br/>
      <span style="font-size: 0.95em;">Separate packages for description, base control, teleoperation, and system integration.</span>
      <br/><br/>
      <table style="width: 100%; font-size: 0.9em;">
        <tr>
          <td style="padding: 0.3em 0;"><code>akros2_description</code></td>
          <td style="padding: 0.3em 0;">URDF/Xacro robot models, meshes, kinematics definitions</td>
        </tr>
        <tr>
          <td style="padding: 0.3em 0;"><code>akros2_base</code></td>
          <td style="padding: 0.3em 0;">Motor controllers, sensor drivers, EKF localization</td>
        </tr>
        <tr>
          <td style="padding: 0.3em 0;"><code>akros2_teleop</code></td>
          <td style="padding: 0.3em 0;">Joy node, teleop_twist_joy node, velocity command multiplexing</td>
        </tr>
        <tr>
          <td style="padding: 0.3em 0;"><code>akros2_bringup</code></td>
          <td style="padding: 0.3em 0;">Launch file hierarchies, parameter configurations</td>
        </tr>
        <tr>
          <td style="padding: 0.3em 0;"><code>akros2_msgs</code></td>
          <td style="padding: 0.3em 0;">Custom messages/interfaces</td>
        </tr>
        <tr>
          <td style="padding: 0.3em 0;"><code>setup</code></td>
          <td style="padding: 0.3em 0;">System configuration, systemd services, development tools, Steamdeck configuration</td>
        </tr>
      </table>
    </div>
  </div>
</div>

---

<iframe height="600" style="width: 100%; border-radius: 8px; box-shadow: 0 4px 6px rgba(0,0,0,0.1);" scrolling="no" src="https://codepen.io/adityakamath/embed/qENNvxe?default-tab=result&editable=true" frameborder="no" loading="lazy">
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
