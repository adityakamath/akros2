#!/bin/bash
export PATH=$HOME/.local/bin:$PATH
export PATH=$HOME/.local/podman/bin:$PATH
xhost +si:localuser:$USER

distrobox enter ros2 -- foxglove-studio

