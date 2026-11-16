#!/usr/bin/env sh
# Downloads the MediaPipe hand models (ONNX exports from the OpenCV Model Zoo).
# Usage: scripts/download_models.sh [dest-dir]
set -eu

DEST="${1:-$(dirname "$0")/../models}"
BASE="https://github.com/opencv/opencv_zoo/raw/main/models"
MIN_BYTES=3000000

mkdir -p "$DEST"
for path in \
  palm_detection_mediapipe/palm_detection_mediapipe_2023feb.onnx \
  handpose_estimation_mediapipe/handpose_estimation_mediapipe_2023feb.onnx
do
  file="$DEST/$(basename "$path")"
  if [ -f "$file" ] && [ "$(wc -c < "$file")" -ge "$MIN_BYTES" ]; then
    echo "ok        $file"
    continue
  fi
  echo "download  $path"
  curl -fsSL -o "$file" "$BASE/$path"
  if [ "$(wc -c < "$file")" -lt "$MIN_BYTES" ]; then
    echo "error: $file is too small (Git LFS pointer or error page?)" >&2
    exit 1
  fi
done
echo "Models are in $DEST"
