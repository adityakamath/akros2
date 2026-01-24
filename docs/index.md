---
layout: page
title: AKROS
subtitle: A mecanum-wheel robot platform using ROS 2
---

<style>
  /* Ultra-aggressive override using @media all for universal application */
  @media all {
    html body .container,
    html body .main-content,
    html body .page-content,
    html body article,
    html body .post-content,
    html body main,
    html body .content,
    html body section,
    html body .container-fluid,
    html body .container-lg,
    html body .container-md,
    html body .container-sm,
    html body .container-xl,
    html body .container-xxl,
    html body div[class*="container"],
    html body .row,
    html body > div,
    html body #main-content,
    html body div.container,
    html body div.main-content {
      max-width: none !important;
      width: 100% !important;
      margin-left: auto !important;
      margin-right: auto !important;
      padding-left: 2.5% !important;
      padding-right: 2.5% !important;
      box-sizing: border-box !important;
    }
  }

  /* Larger screen padding */
  @media all and (min-width: 1200px) {
    html body .container,
    html body .main-content,
    html body .page-content,
    html body article,
    html body .post-content,
    html body main,
    html body .content,
    html body section,
    html body .container-fluid,
    html body .container-lg,
    html body .container-md,
    html body .container-sm,
    html body .container-xl,
    html body .container-xxl,
    html body div[class*="container"],
    html body .row,
    html body > div,
    html body #main-content,
    html body div.container,
    html body div.main-content {
      padding-left: 5% !important;
      padding-right: 5% !important;
    }
  }

  .capability-box {
    transition: all 0.2s ease;
  }

  .capability-box:hover {
    transform: translateY(-2px);
    box-shadow: 0 4px 12px rgba(0,0,0,0.2) !important;
  }
</style>

<div style="width: 100vw; margin-left: calc(-50vw + 50%); padding-left: 5%; padding-right: 5%; box-sizing: border-box;" markdown="1">

![Project Status](https://img.shields.io/badge/Status-Archived-red)
![ROS 2](https://img.shields.io/badge/ROS%202-Humble%20(Ubuntu%2022.04)-purple?style=flat&logo=ros&logoSize=auto)
![Repository](https://img.shields.io/badge/Repository-adityakamath%2Fakros2-purple?style=flat&logo=github&logoSize=auto&link=https%3A%2F%2Fgithub.com%2Fadityakamath%2Fakros2&link=https%3A%2F%2Fgithub.com%2Fadityakamath%2Fakros2)
![Python](https://img.shields.io/badge/Python-3.10+-blue?style=flat&logo=python&logoColor=white)
![C++](https://img.shields.io/badge/C++-17-blue?style=flat&logo=cplusplus&logoColor=white)
![Blog](https://img.shields.io/badge/Blog-AKROS%20Series-orange?style=flat&logo=hashnode&logoSize=auto&link=https%3A%2F%2Fkamathrobotics.com%2Fseries%2Fakros&link=https%3A%2F%2Fkamathrobotics.com%2Fseries%2Fakros)
![License](https://img.shields.io/github/license/adityakamath/akros2?label=License)

</div>

<div style="width: 100vw; margin-left: calc(-50vw + 50%); padding-left: 5%; padding-right: 5%; box-sizing: border-box;">
<div style="display: flex; flex-wrap: wrap; gap: 0.6em; margin: 2em 0; align-items: stretch;">
  <div class="capability-box" style="flex: 1 1 calc(33.333% - 0.4em); min-width: 280px; display: flex; align-items: flex-start; gap: 0.8em; background: #ffffff; padding: 0.7em 0.9em; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);">
    <span style="font-size: 1.8em;">🎛️</span>
    <div style="flex: 1;">
      <strong>Holonomic Drive Control</strong><br/>
      <a href="https://github.com/adityakamath/akros2/tree/main/akros2_base" target="_blank" style="text-decoration: none; color: #666; cursor: pointer;">
        <span style="font-size: 0.85em;"><code>akros2_base</code></span>
      </a>
      <br/><br/>
      <span style="font-size: 0.95em;">Mecanum wheel kinematics with motor controllers, sensor drivers, and odometry.</span>
    </div>
  </div>
  <div class="capability-box" style="flex: 1 1 calc(33.333% - 0.4em); min-width: 280px; display: flex; align-items: flex-start; gap: 0.8em; background: #ffffff; padding: 0.7em 0.9em; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);">
    <span style="font-size: 1.8em;">🤖</span>
    <div style="flex: 1;">
      <strong>Robot Description</strong><br/>
      <a href="https://github.com/adityakamath/akros2/tree/main/akros2_description" target="_blank" style="text-decoration: none; color: #666; cursor: pointer;">
        <span style="font-size: 0.85em;"><code>akros2_description</code></span>
      </a>
      <br/><br/>
      <span style="font-size: 0.95em;">URDF and Xacro models with meshes and kinematics for robot visualization and simulation.</span>
    </div>
  </div>
  <div class="capability-box" style="flex: 1 1 calc(33.333% - 0.4em); min-width: 280px; display: flex; align-items: flex-start; gap: 0.8em; background: #ffffff; padding: 0.7em 0.9em; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);">
    <span style="font-size: 1.8em;">📟</span>
    <div style="flex: 1;">
      <strong>Low-level Firmware</strong><br/>
      <a href="https://github.com/adityakamath/akros2_firmware" target="_blank" style="text-decoration: none; color: #666; cursor: pointer;">
        <span style="font-size: 0.85em;"><code>akros2_firmware</code></span>
      </a>
      <br/><br/>
      <span style="font-size: 0.95em;">micro-ROS firmware for Teensy 4.1 with motor control and sensor interfacing over serial.</span>
    </div>
  </div>
  <div class="capability-box" style="flex: 1 1 calc(33.333% - 0.4em); min-width: 280px; display: flex; align-items: flex-start; gap: 0.8em; background: #ffffff; padding: 0.7em 0.9em; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);">
    <span style="font-size: 1.8em;">🎮</span>
    <div style="flex: 1;">
      <strong>Teleoperation</strong><br/>
      <a href="https://github.com/adityakamath/akros2/tree/main/akros2_teleop" target="_blank" style="text-decoration: none; color: #666; cursor: pointer;">
        <span style="font-size: 0.85em;"><code>akros2_teleop</code></span>
      </a>
      <span style="font-size: 0.85em; color: #999;"> </span>
      <a href="https://github.com/adityakamath/akros2/tree/main/akros2_msgs" target="_blank" style="text-decoration: none; color: #666; cursor: pointer;">
        <span style="font-size: 0.85em;"><code>akros2_msgs</code></span>
      </a>
      <br/><br/>
      <span style="font-size: 0.95em;">Gamepad control with twist command multiplexing for switching between modes.</span>
    </div>
  </div>
  <div class="capability-box" style="flex: 1 1 calc(33.333% - 0.4em); min-width: 280px; display: flex; align-items: flex-start; gap: 0.8em; background: #ffffff; padding: 0.7em 0.9em; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);">
    <span style="font-size: 1.8em;">🧩</span>
    <div style="flex: 1;">
      <strong>Modular Architecture</strong><br/>
      <a href="https://github.com/adityakamath/akros2/tree/main/akros2_bringup" target="_blank" style="text-decoration: none; color: #666; cursor: pointer;">
        <span style="font-size: 0.85em;"><code>akros2_bringup</code></span>
      </a>
      <br/><br/>
      <span style="font-size: 0.95em;">Launch files and parameter configurations for system bringup.</span>
    </div>
  </div>
  <div class="capability-box" style="flex: 1 1 calc(33.333% - 0.4em); min-width: 280px; display: flex; align-items: flex-start; gap: 0.8em; background: #ffffff; padding: 0.7em 0.9em; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);">
    <span style="font-size: 1.8em;">⚙️</span>
    <div style="flex: 1;">
      <strong>System Configuration</strong><br/>
      <a href="https://github.com/adityakamath/akros2/tree/main/setup" target="_blank" style="text-decoration: none; color: #666; cursor: pointer;">
        <span style="font-size: 0.85em;"><code>setup</code></span>
      </a>
      <br/><br/>
      <span style="font-size: 0.95em;">Services, configs, scripts, and misc. development tools.</span>
    </div>
  </div>
</div>
</div>

<div style="width: 100vw; margin-left: calc(-50vw + 50%); padding-left: 5%; padding-right: 5%; box-sizing: border-box;">
<iframe height="500" style="width: 100%; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);" scrolling="no" src="https://codepen.io/adityakamath/embed/qENNvxe?default-tab=result" frameborder="no" loading="lazy">
</iframe>
</div>

<div style="width: 100vw; margin-left: calc(-50vw + 50%); padding-left: 5%; padding-right: 5%; box-sizing: border-box;">

<table style="width: 100%; border-collapse: separate; border-spacing: 0; margin: 2em auto; border-radius: 8px; overflow: hidden; box-shadow: 0 4px 12px rgba(0,0,0,0.2); border: none;">
  <thead>
    <tr>
      <th colspan="3" style="text-align: center; padding: 0.6em; background: #f8f9fa; border: none;">Hardware Configuration</th>
    </tr>
    <tr>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Component</th>
      <th style="text-align: center; padding: 0.6em; background: #e9ecef; border: none;">Qty</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;">Raspberry Pi 4</td>
      <td style="text-align: center; padding: 0.6em; border: none;">1</td>
      <td style="padding: 0.6em; border: none;">4GB minimum or equivalent ARM64/x86_64 system</td>
    </tr>
    <tr style="background: #f8f9fa;">
      <td style="padding: 0.6em; border: none;">Teensy Microcontroller</td>
      <td style="text-align: center; padding: 0.6em; border: none;">1</td>
      <td style="padding: 0.6em; border: none;">Running <a href="https://github.com/adityakamath/akros2_firmware" target="_blank">micro-ROS firmware</a> connected via serial (UART/USB)</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;">Mecanum Wheels</td>
      <td style="text-align: center; padding: 0.6em; border: none;">4</td>
      <td style="padding: 0.6em; border: none;">Omnidirectional drive wheels</td>
    </tr>
    <tr style="background: #f8f9fa;">
      <td style="padding: 0.6em; border: none;">DC Motors with Encoders</td>
      <td style="text-align: center; padding: 0.6em; border: none;">4</td>
      <td style="padding: 0.6em; border: none;">Quadrature encoders for odometry</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;">9-DOF IMU</td>
      <td style="text-align: center; padding: 0.6em; border: none;">1</td>
      <td style="padding: 0.6em; border: none;">Accelerometer, gyroscope, magnetometer</td>
    </tr>
    <tr style="background: #f8f9fa;">
      <td style="padding: 0.6em; border: none;">Cytron MDD3A Motor Drivers</td>
      <td style="text-align: center; padding: 0.6em; border: none;">2</td>
      <td style="padding: 0.6em; border: none;">Dual channel motor controllers</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;">Power Supply</td>
      <td style="text-align: center; padding: 0.6em; border: none;">1</td>
      <td style="padding: 0.6em; border: none;">Mini UPS with buck converters and monitoring circuitry</td>
    </tr>
    <tr style="background: #f8f9fa;">
      <td style="padding: 0.6em; border: none;">Indicator LEDs</td>
      <td style="text-align: center; padding: 0.6em; border: none;">-</td>
      <td style="padding: 0.6em; border: none;">Status indicators <em>(Optional)</em></td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;">LDLidar LD06</td>
      <td style="text-align: center; padding: 0.6em; border: none;">1</td>
      <td style="padding: 0.6em; border: none;">Or compatible 2D laser scanner <em>(Optional)</em></td>
    </tr>
    <tr style="background: #f8f9fa;">
      <td style="padding: 0.6em; border: none;">USB Camera</td>
      <td style="text-align: center; padding: 0.6em; border: none;">1</td>
      <td style="padding: 0.6em; border: none;">v4l2 compatible <em>(Optional)</em></td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;">Additional Sensors</td>
      <td style="text-align: center; padding: 0.6em; border: none;">-</td>
      <td style="padding: 0.6em; border: none;">Custom sensors via ROS 2 integration <em>(Optional)</em></td>
    </tr>
  </tbody>
</table>

</div>
