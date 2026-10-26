#!/usr/bin/env bash
# ==============================================================================
# CARLA Simulator 0.9.15 Launcher with NVIDIA GPU Acceleration
# ==============================================================================

CONTAINER_NAME="carla_server"
CARLA_IMAGE="carlasim/carla:0.9.15"

MODE="windowed"
if [[ "$1" == "--offscreen" || "$1" == "-o" ]]; then
    MODE="offscreen"
fi

echo "========================================================"
echo " Starting CARLA 0.9.15 Simulator (${MODE} mode)"
echo " GPU: NVIDIA GeForce GTX 1060 (Low Quality Preset)"
echo "========================================================"

xhost +local:docker > /dev/null 2>&1

# Stop and remove existing container if running
if docker ps -a --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "➜ Removing previous ${CONTAINER_NAME} instance..."
    docker rm -f "${CONTAINER_NAME}" > /dev/null 2>&1
fi

CARLA_CMD="./CarlaUE4.sh -quality-level=Low -benchmark -fps=30"
if [[ "$MODE" == "offscreen" ]]; then
    CARLA_CMD="./CarlaUE4.sh -RenderOffScreen -quality-level=Low -benchmark -fps=30"
fi

docker run -d \
  --name "${CONTAINER_NAME}" \
  --privileged \
  --gpus all \
  --net=host \
  --ipc=host \
  -e DISPLAY="${DISPLAY:-:0}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  "${CARLA_IMAGE}" \
  /bin/bash -c "${CARLA_CMD}"

echo "➜ Waiting for CARLA Server to initialize on port 2000..."
for i in {1..30}; do
    if nc -z 127.0.0.1 2000 2>/dev/null; then
        echo "✓ CARLA Server is UP and listening on localhost:2000!"
        exit 0
    fi
    sleep 1
done

echo "⚠ Warning: CARLA server not yet responding on port 2000. Check logs with: docker logs -f ${CONTAINER_NAME}"
