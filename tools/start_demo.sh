#!/usr/bin/env bash
# ==============================================================================
# CARLA ADAS Stack — Interactive Demonstration Suite
# ==============================================================================

echo "=============================================================================="
echo "         CARLA AUTONOMOUS DRIVING & ADAS STACK — TEST SUITE"
echo "=============================================================================="
echo " Wähle das gewünschte Demonstrations-Szenario:"
echo ""
echo "   [1] Euro NCAP Notbremsung (AEB CCRs) — Vollbremsung vor Hindernis"
echo "   [2] Level 3/4 Dynamisches Überholen  — Ruckfreier S-Kurven-Spurwechsel"
echo ""

CHOICE="$1"
if [ -z "$CHOICE" ]; then
    read -p "Auswahl (1 oder 2) [Standard: 2]: " CHOICE
fi

if [[ "$CHOICE" == "1" ]]; then
    echo "➜ Starte Szenario 1: Euro NCAP Notbremsung (AEB)..."
    docker exec -it ros_container bash -c "source /opt/ros/humble/setup.bash && source /root/ros-projects/install/setup.bash && python3 /root/ros-projects/src/carla_adas_stack/tools/live_demo.py"
else
    echo "➜ Starte Szenario 2: Level 3/4 Dynamisches Überholen (Overtake)..."
    docker exec -it ros_container bash -c "source /opt/ros/humble/setup.bash && source /root/ros-projects/install/setup.bash && python3 /root/ros-projects/src/carla_adas_stack/tools/demo_overtake.py"
fi
