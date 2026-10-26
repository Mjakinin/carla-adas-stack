#!/usr/bin/env bash
# ==============================================================================
# CARLA ADAS Stack — Automated Video & GIF Showcase Recorder
# ==============================================================================

set -e

MEDIA_DIR="/home/mjakinin/ros-projects/src/carla_adas_stack/docs/media"
mkdir -p "$MEDIA_DIR"

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

# Detect CARLA window geometry
WIN_INFO=$(xwininfo -root -tree | grep -i "CarlaUE4" | head -n 1)
if [ -z "$WIN_INFO" ]; then
    echo "⚠ Warnung: CARLA-Fenster nicht gefunden. Stelle sicher, dass start_carla.sh läuft!"
    exit 1
fi

# Extract geometry: e.g. 1600x900+39+76
GEOM=$(echo "$WIN_INFO" | grep -oE '[0-9]+x[0-9]+\+[0-9]+\+[0-9]+' | head -n 1)
W=$(echo "$GEOM" | cut -d'x' -f1)
H=$(echo "$GEOM" | cut -d'x' -f2 | cut -d'+' -f1)
X=$(echo "$GEOM" | cut -d'+' -f2)
Y=$(echo "$GEOM" | cut -d'+' -f3)

echo "✓ Gefundenes CARLA-Fenster: ${W}x${H} an Position +${X},+${Y}"
RAW_MP4="$MEDIA_DIR/${NAME}.mp4"
FINAL_GIF="$MEDIA_DIR/${NAME}.gif"

echo "➜ Starte HD-Video-Aufnahme mit FFmpeg..."
ffmpeg -y -f x11grab -video_size "${W}x${H}" -framerate 30 -i ":0.0+${X},${Y}" -c:v libx264 -preset fast -pix_fmt yuv420p "$RAW_MP4" > /dev/null 2>&1 &
RECORDER_PID=$!

sleep 1

echo "➜ Führe Demonstration in ROS 2 aus..."
docker exec -it ros_container bash -c "source /opt/ros/humble/setup.bash && source /root/ros-projects/install/setup.bash && python3 $DEMO_SCRIPT" || true

echo "➜ Beende Videoaufnahme..."
kill -INT $RECORDER_PID 2>/dev/null || true
wait $RECORDER_PID 2>/dev/null || true

if [ -f "$RAW_MP4" ]; then
    echo "✓ Video gespeichert: $RAW_MP4 ($(du -h "$RAW_MP4" | cut -f1))"
    echo "➜ Erzeuge animiertes High-Quality GIF für GitHub README..."
    ffmpeg -y -i "$RAW_MP4" -vf "fps=15,scale=800:-1:flags=lanczos,split[s0][s1];[s0]palettegen[p];[s1][p]paletteuse" -loop 0 "$FINAL_GIF" > /dev/null 2>&1 || true
    if [ -f "$FINAL_GIF" ]; then
        echo "✓ GIF erfolgreich generiert: $FINAL_GIF ($(du -h "$FINAL_GIF" | cut -f1))"
    fi
fi

echo "=============================================================================="
echo " ✓ AUFNAHME BEENDET! Bereit für GitHub & Portfolio!"
echo "=============================================================================="
