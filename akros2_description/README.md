# akros2_description

![ROS 2 Humble](https://img.shields.io/badge/ROS%202%20Humble-Ubuntu%2022.04-blue)
![GitHub License](https://img.shields.io/github/license/adityakamath/akros2_description)
![X (formerly Twitter) Follow](https://img.shields.io/twitter/follow/kamathsblog)

Robot URDF/Xacro descriptions, meshes, and kinematics for the AKROS2 mecanum-wheeled robot.

## Documentation

📚 See the [**AKROS2 Architecture Documentation**](../docs/architecture.md#akros2_description) for complete details including:

- URDF structure and organization
- Mesh files and 3D models
- Joint definitions and kinematics
- Launch file configuration
- URDF generation for external tools
- Remote mesh path usage
- Integration with Unity and Foxglove

## Quick Reference

**Launch the description:**
```bash
ros2 launch akros2_description description_launch.py
```

**Launch arguments:**
- `js_ext` (Default: `True`) - Enable external joint states from micro-ROS
- `js_topic` (Default: `joint_states`) - Joint states topic name

**Generate URDF for external tools:**
```bash
cd src/akros2/akros2_description
xacro urdf/robot.urdf.xacro nopath:=False > urdf/robot.urdf
```

## Package Contents

- `launch/description_launch.py` - Main launch file
- `urdf/` - Robot URDF and xacro files
- `meshes/` - STL mesh files for visualization

For detailed documentation, see [Architecture Documentation](../docs/architecture.md#akros2_description).
