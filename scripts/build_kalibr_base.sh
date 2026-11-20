#!/usr/bin/env bash
set -euo pipefail

IMAGE_NAME="kalibr"
KALIBR_REPO="https://github.com/ethz-asl/kalibr.git"
DOCKERFILE_NAME="Dockerfile_ros1_20_04"
FORCE_BUILD=false

if [[ "${1:-}" == "--force" ]]; then
    FORCE_BUILD=true
fi

if ! command -v docker &> /dev/null; then
    echo "[ERROR] Docker is not installed or not in PATH."
    exit 1
fi

if ! docker info &> /dev/null; then
    echo "[ERROR] Docker daemon is not running or current user lacks permission."
    echo "        Try: sudo usermod -aG docker \$USER && newgrp docker"
    exit 1
fi

if [[ "$FORCE_BUILD" == false ]] && docker image inspect "$IMAGE_NAME" &> /dev/null; then
    echo "[INFO] Docker image '$IMAGE_NAME' already exists locally."
    echo "       Run with '--force' to rebuild from scratch."
    exit 0
fi

TMP_DIR=$(mktemp -d)
trap 'rm -rf "$TMP_DIR"' EXIT

echo "[INFO] Cloning ETHZ-ASL Kalibr into temporary directory..."
git clone --depth 1 "$KALIBR_REPO" "$TMP_DIR/kalibr"

cd "$TMP_DIR/kalibr"

if [[ ! -f "$DOCKERFILE_NAME" ]]; then
    echo "[ERROR] Expected '$DOCKERFILE_NAME' not found in Kalibr repository."
    ls -la
    exit 1
fi

echo "[INFO] Building base Docker image '$IMAGE_NAME' using $DOCKERFILE_NAME..."
echo "[INFO] This compiles ROS 1 Noetic + Kalibr and may take 15-30 minutes on first run."
docker build -t "$IMAGE_NAME" -f "$DOCKERFILE_NAME" .

echo ""
echo "[SUCCESS] Base image '$IMAGE_NAME' is ready."
echo "[NEXT]    From the repository root, build the converter image:"
echo "          docker build -t stereo_camera_and_imu_to_rosbag_converter ."