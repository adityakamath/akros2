# akros2_base

![ROS 2 Humble](https://img.shields.io/badge/ROS%202%20Humble-Ubuntu%2022.04-blue)
![GitHub License](https://img.shields.io/github/license/adityakamath/akros2_base)
![X (formerly Twitter) Follow](https://img.shields.io/twitter/follow/kamathsblog)

Core robot functionality including drivers, sensors, filters, and sensor fusion for AKROS2.

## Documentation

📚 See the [**AKROS2 Architecture Documentation**](../docs/architecture.md#akros2_base) for complete details including:

- All launch files and their arguments
- Sensor fusion pipeline (Madgwick filter + EKF)
- Configuration file descriptions
- Topic interfaces
- Node descriptions and functionality

## Quick Reference

**Launch sensor fusion:**
```bash
ros2 launch akros2_base sensor_fusion_launch.py
```

**Launch LIDAR:**
```bash
ros2 launch akros2_base laser_launch.py laser_filter:=true
```

**Launch camera:**
```bash
ros2 launch akros2_base camera_launch.py compose:=false
```

**Launch control:**
```bash
ros2 launch akros2_base control_launch.py
```

## Package Contents

### Launch Files
- `twist_mixer_launch.py` - Command velocity mixing
- `teleop_launch.py` - Teleoperation nodes
- `sensor_fusion_launch.py` - IMU filtering and EKF fusion
- `laser_launch.py` - LIDAR driver and filters
- `camera_launch.py` - Camera driver
- `control_launch.py` - micro-ROS agent

### Configuration Files
- `camera_config.yaml` - Camera parameters
- `camera_info.yaml` - Camera calibration
- `ekf_config.yaml` - EKF parameters
- `imu_filter_config.yaml` - Madgwick filter parameters
- `motion_detector_config.yaml` - Motion detection thresholds
- `laser_filter_config.yaml` - LIDAR filter chain

For detailed documentation, see [Architecture Documentation](../docs/architecture.md#akros2_base).
