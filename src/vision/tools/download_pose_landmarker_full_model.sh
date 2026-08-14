#!/usr/bin/env bash
# Section: Download the MediaPipe Pose Landmarker Full task model into the vision package.
set -euo pipefail

# Section: Resolve the script directory and package model destination.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
MODEL_DIR="${PACKAGE_DIR}/models"
MODEL_PATH="${MODEL_DIR}/pose_landmarker_full.task"
MODEL_URL="https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task"

# Section: Create the model directory and download the Full pose landmarker.
mkdir -p "${MODEL_DIR}"
curl -L "${MODEL_URL}" -o "${MODEL_PATH}"

# Section: Print the saved path for launch command reuse.
echo "Saved MediaPipe Pose Landmarker Full model to: ${MODEL_PATH}"
