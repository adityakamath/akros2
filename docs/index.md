---
layout: page
title: AKROS2
subtitle: Next-generation ROS 2 stack for mecanum-wheeled mobile robots
cover-img: /akros2/assets/img/header_image.png
---

<div style="text-align: center; margin: 2em 0;">
  <p style="font-size: 1.3em; color: #555; margin-bottom: 1.5em;">
    A comprehensive, modular robotics platform built on ROS 2 Humble with full holonomic control, advanced sensor fusion, and seamless hardware integration.
  </p>
  <div style="margin: 2em 0;">
    <img src="https://img.shields.io/badge/ROS%202%20Humble-Ubuntu%2022.04-blue" alt="ROS 2 Humble" style="margin: 0.2em;">
    <img src="https://img.shields.io/github/license/adityakamath/akros2" alt="GitHub License" style="margin: 0.2em;">
  </div>
</div>

---

## See It In Action

<div style="text-align: center; margin: 2em 0;">
  <p style="font-size: 1.1em; margin-bottom: 1em;">
    Experience the mecanum drive kinematics with our interactive 3D visualization
  </p>
</div>

<div style="position: relative; padding-bottom: 75%; height: 0; overflow: hidden; max-width: 100%; margin: 2em 0; box-shadow: 0 4px 6px rgba(0,0,0,0.1); border-radius: 8px;">
  <iframe style="position: absolute; top: 0; left: 0; width: 100%; height: 100%; border-radius: 8px;" scrolling="no" title="AKROS2 - 3D Robot Visualization" src="https://codepen.io/adityakamath/embed/qENNvxe?default-tab=result" frameborder="no" loading="lazy" allowtransparency="true" allowfullscreen="true">
    See the Pen <a href="https://codepen.io/adityakamath/pen/qENNvxe">AKROS2 - 3D Robot Visualization</a> by Aditya Kamath (<a href="https://codepen.io/adityakamath">@adityakamath</a>) on <a href="https://codepen.io">CodePen</a>.
  </iframe>
</div>

---

## Why AKROS2?

<div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 1.5em; margin: 2em 0;">

<div style="padding: 1.5em; background: #f8f9fa; border-radius: 8px; border-left: 4px solid #007bff;">
  <h3 style="margin-top: 0; color: #007bff;">🎮 Holonomic Control</h3>
  <p>Full 4-directional mecanum drive control with precision kinematics. Move in any direction without rotating.</p>
</div>

<div style="padding: 1.5em; background: #f8f9fa; border-radius: 8px; border-left: 4px solid #28a745;">
  <h3 style="margin-top: 0; color: #28a745;">🔬 Advanced Fusion</h3>
  <p>Extended Kalman Filter fuses IMU and wheel odometry for accurate state estimation and navigation.</p>
</div>

<div style="padding: 1.5em; background: #f8f9fa; border-radius: 8px; border-left: 4px solid #ffc107;">
  <h3 style="margin-top: 0; color: #ffc107;">🔌 Plug & Play</h3>
  <p>Multi-controller support: PS4, Stadia, 8BitDo, Steam Deck. Connect your favorite gamepad and drive.</p>
</div>

<div style="padding: 1.5em; background: #f8f9fa; border-radius: 8px; border-left: 4px solid #17a2b8;">
  <h3 style="margin-top: 0; color: #17a2b8;">📊 Rich Visualization</h3>
  <p>Choose your workflow: Foxglove Studio for modern web-based viz or classic RViz2. ROSBridge included.</p>
</div>

<div style="padding: 1.5em; background: #f8f9fa; border-radius: 8px; border-left: 4px solid #6f42c1;">
  <h3 style="margin-top: 0; color: #6f42c1;">⚡ micro-ROS Ready</h3>
  <p>Seamless low-level control via Teensy microcontroller. Real-time performance where it matters.</p>
</div>

<div style="padding: 1.5em; background: #f8f9fa; border-radius: 8px; border-left: 4px solid #e83e8c;">
  <h3 style="margin-top: 0; color: #e83e8c;">🧩 Modular Design</h3>
  <p>Independent packages for description, control, teleop, and integration. Customize to your needs.</p>
</div>

</div>

---

## Get Started

<div style="text-align: center; margin: 2em 0;">
  <a href="https://github.com/adityakamath/akros2" style="display: inline-block; padding: 12px 30px; margin: 0.5em; background: #007bff; color: white; text-decoration: none; border-radius: 5px; font-weight: bold; font-size: 1.1em;">
    📦 View on GitHub
  </a>
  <a href="architecture.html" style="display: inline-block; padding: 12px 30px; margin: 0.5em; background: #28a745; color: white; text-decoration: none; border-radius: 5px; font-weight: bold; font-size: 1.1em;">
    📚 Architecture Docs
  </a>
</div>

---

## Platform Overview

AKROS2 is designed for **mecanum-wheeled mobile robots** running **ROS 2 Humble** on **Ubuntu 22.04**. The stack handles everything from low-level motor control through micro-ROS to high-level navigation and visualization.

**Hardware Foundation:**
- Raspberry Pi 4 (or equivalent) running the ROS 2 stack
- Teensy microcontroller with [micro-ROS firmware](https://github.com/adityakamath/akros2_firmware)
- 4x mecanum wheels with encoders for omnidirectional mobility
- 9-DOF IMU for orientation and acceleration sensing
- Optional: LD06 LIDAR and USB camera for perception

**Software Stack:**
- `akros2_description` - Robot models, URDF, meshes, and kinematics
- `akros2_base` - Core drivers, sensor fusion, and odometry
- `akros2_teleop` - Gamepad support and command mixing
- `akros2_bringup` - System launch files and configurations
- `akros2_msgs` - Custom message definitions
- `akros2_setup` - System services and development tools

<div style="text-align: center; margin: 2em 0; padding: 1.5em; background: #fffbea; border-radius: 8px; border: 2px solid #ffc107;">
  <p style="font-size: 1.1em; margin: 0;">
    <strong>💡 Built for makers, researchers, and robotics enthusiasts</strong><br>
    Open source, modular, and ready to deploy
  </p>
</div>
