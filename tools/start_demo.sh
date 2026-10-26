#!/usr/bin/env bash
# ==============================================================================
# CARLA ADAS Stack — 1-Click Live Demonstration Launcher
# ==============================================================================

docker exec -it ros_container bash -c "source /opt/ros/humble/setup.bash && source /root/ros-projects/install/setup.bash && python3 /root/ros-projects/src/carla_adas_stack/tools/live_demo.py"
