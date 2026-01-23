# akros2_setup

![ROS 2 Humble](https://img.shields.io/badge/ROS%202%20Humble-Ubuntu%2022.04-blue)
![GitHub License](https://img.shields.io/github/license/adityakamath/akros2_setup)

System configuration, services, and development tools for AKROS2 on Linux devices.

## Documentation

📚 See the [**AKROS2 Architecture Documentation**](../docs/architecture.md#akros2_setup) for complete details including:

- Bash environment configuration
- Convenience aliases
- systemd services
- udev rules
- Development tool integrations

## Quick Setup

1. **Clone to ~/Setup:**
   ```bash
   git clone https://github.com/adityakamath/akros2_setup Setup
   cd Setup
   ```

2. **Update bashrc:**
   ```bash
   cat .bashrc >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Copy udev rules:**
   ```bash
   sudo cp rules/*.rules /etc/udev/rules.d
   sudo udevadm control --reload-rules && sudo udevadm trigger
   ```

4. **Enable services:**
   ```bash
   sudo cp services/*.service /etc/systemd/system/
   sudo systemctl enable <service>.service
   sudo systemctl daemon-reload
   ```

## Bash Aliases

### Workspace
- `rosws` - Navigate to workspace
- `srs` - Source ROS setup
- `sls` - Source local setup

### Build
- `build_all` - Build all packages
- `build_only <pkg>` - Build specific package
- `dep_install` - Install dependencies

### Tools
- `fgb` - Foxglove Bridge

### Launch
- `bringup` - Main robot launch
- `bringup_local` - Robot with local control
- `basestation` - Base station
- `control` - Control only
- `viz` - Visualization only

## Platform Support

**Tested On:**
- Raspberry Pi 4 (4GB/8GB)
- Raspberry Pi Zero 2 W
- Valve Steam Deck (Ubuntu 22.04 via Distrobox)
- WSL2 (partial support)

For detailed documentation, see [Architecture Documentation](../docs/architecture.md#akros2_setup).
