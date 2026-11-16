#!/usr/bin/env sh
# Downloads what the robot-arm front-end needs:
#   1. MuJoCo (physics simulator), prebuilt, into third_party/mujoco
#   2. The SO-101 arm model from MuJoCo Menagerie, into models/so101
# Usage: scripts/download_arm.sh
set -eu

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
MUJOCO_VERSION="3.17.0"
MENAGERIE_COMMIT="3abca2e302625cec9429c49806121ea2855cef81"

# --- MuJoCo -----------------------------------------------------------------
MJ_DIR="$ROOT/third_party/mujoco"
if [ -f "$MJ_DIR/include/mujoco/mujoco.h" ]; then
  echo "ok        MuJoCo already in $MJ_DIR"
else
  case "$(uname -s)-$(uname -m)" in
    Linux-x86_64)  ASSET="mujoco-$MUJOCO_VERSION-linux-x86_64.tar.gz" ;;
    Linux-aarch64) ASSET="mujoco-$MUJOCO_VERSION-linux-aarch64.tar.gz" ;;
    *) echo "error: no prebuilt MuJoCo for $(uname -s)-$(uname -m) in this script;" \
            "see https://github.com/google-deepmind/mujoco/releases" >&2; exit 1 ;;
  esac
  echo "download  $ASSET"
  mkdir -p "$ROOT/third_party"
  TMP="$(mktemp -d)"
  curl -fsSL -o "$TMP/$ASSET" \
    "https://github.com/google-deepmind/mujoco/releases/download/$MUJOCO_VERSION/$ASSET"
  tar -xzf "$TMP/$ASSET" -C "$TMP"
  rm -rf "$MJ_DIR"
  mv "$TMP/mujoco-$MUJOCO_VERSION" "$MJ_DIR"
  rm -rf "$TMP"
fi

# --- SO-101 model -----------------------------------------------------------
MODEL_DIR="$ROOT/models/so101"
BASE="https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/$MENAGERIE_COMMIT/robotstudio_so101"
mkdir -p "$MODEL_DIR/assets"
for f in so101.xml scene.xml LICENSE; do
  [ -s "$MODEL_DIR/$f" ] || { echo "download  $f"; curl -fsSL -o "$MODEL_DIR/$f" "$BASE/$f"; }
done
for f in base_motor_holder_so101_v1 base_so101_v2 motor_holder_so101_base_v1 \
         motor_holder_so101_wrist_v1 moving_jaw_so101_gripper_part0_v1 \
         moving_jaw_so101_gripper_part1_v1 moving_jaw_so101_gripper_v1 moving_jaw_so101_v1 \
         rotation_pitch_so101_v1 sts3215_03a_no_horn_v1 sts3215_03a_v1 under_arm_so101_v1 \
         upper_arm_so101_v1 waveshare_mounting_plate_so101_v2 \
         wrist_roll_follower_so101_camera_mount wrist_roll_follower_so101_gripper_part0_v1 \
         wrist_roll_follower_so101_gripper_v1 wrist_roll_follower_so101_v1 \
         wrist_roll_pitch_so101_v2; do
  [ -s "$MODEL_DIR/assets/$f.stl" ] || {
    echo "download  assets/$f.stl"
    curl -fsSL -o "$MODEL_DIR/assets/$f.stl" "$BASE/assets/$f.stl"
  }
done
echo "MuJoCo:  $MJ_DIR"
echo "SO-101:  $MODEL_DIR"
