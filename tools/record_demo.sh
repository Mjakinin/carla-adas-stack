#!/usr/bin/env bash
# ==============================================================================
# CARLA ADAS Stack — Direct-Render HD Video & GIF Showcase Recorder
# ==============================================================================

set -e

SCENARIO="${1:-2}"

if [ "$SCENARIO" == "1" ]; then
    NAME="aeb_emergency_braking"
    DEMO_SCRIPT="/root/ros-projects/src/carla_adas_stack/tools/live_demo.py"
    TITLE="Euro NCAP AEB (CCRs)"
else
    NAME="dynamic_overtake_maneuver"
    DEMO_SCRIPT="/root/ros-projects/src/carla_adas_stack/tools/demo_overtake.py"
    TITLE="Level 3/4 Dynamic Overtake (Quintic S-Curve)"
fi

echo "=============================================================================="
echo "   RECORDER: $TITLE"
echo "=============================================================================="
echo "➜ Starte Demonstration mit nativer Unreal Engine HD-Kameraaufnahme..."

docker exec -it ros_container bash -c "source /opt/ros/humble/setup.bash && source /root/ros-projects/install/setup.bash && python3 $DEMO_SCRIPT --record"

echo "=============================================================================="
echo " ✓ AUFNAHME BEENDET! MP4 & GIF erfolgreich in docs/media/ generiert!"
echo "=============================================================================="
