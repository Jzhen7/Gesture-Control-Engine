# Downloads what the robot-arm front-end needs:
#   1. MuJoCo (physics simulator), prebuilt, into third_party\mujoco
#   2. The SO-101 arm model from MuJoCo Menagerie, into models\so101
# Usage:  powershell -ExecutionPolicy Bypass -File scripts\download_arm.ps1
$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"   # Invoke-WebRequest is very slow with the progress bar

$Root = Resolve-Path (Join-Path $PSScriptRoot "..")
$MujocoVersion = "3.17.0"
$MenagerieCommit = "3abca2e302625cec9429c49806121ea2855cef81"

# --- MuJoCo -----------------------------------------------------------------
$MjDir = Join-Path $Root "third_party\mujoco"
if (Test-Path (Join-Path $MjDir "include\mujoco\mujoco.h")) {
    Write-Host "ok        MuJoCo already in $MjDir"
} else {
    $asset = "mujoco-$MujocoVersion-windows-x86_64.zip"
    Write-Host "download  $asset"
    New-Item -ItemType Directory -Force -Path (Join-Path $Root "third_party") | Out-Null
    $zip = Join-Path $env:TEMP $asset
    Invoke-WebRequest -UseBasicParsing -OutFile $zip `
        -Uri "https://github.com/google-deepmind/mujoco/releases/download/$MujocoVersion/$asset"
    if (Test-Path $MjDir) { Remove-Item -Recurse -Force $MjDir }
    Expand-Archive -Path $zip -DestinationPath $MjDir
    Remove-Item $zip
    # Some releases nest everything in a versioned folder; flatten it.
    $nested = Get-ChildItem $MjDir -Directory | Where-Object { Test-Path (Join-Path $_.FullName "include\mujoco") }
    if ($nested -and -not (Test-Path (Join-Path $MjDir "include"))) {
        Get-ChildItem $nested.FullName | Move-Item -Destination $MjDir
        Remove-Item $nested.FullName
    }
}

# --- SO-101 model -----------------------------------------------------------
$ModelDir = Join-Path $Root "models\so101"
$Base = "https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/$MenagerieCommit/robotstudio_so101"
New-Item -ItemType Directory -Force -Path (Join-Path $ModelDir "assets") | Out-Null
$files = @("so101.xml", "scene.xml", "LICENSE") + (@(
    "base_motor_holder_so101_v1", "base_so101_v2", "motor_holder_so101_base_v1",
    "motor_holder_so101_wrist_v1", "moving_jaw_so101_gripper_part0_v1",
    "moving_jaw_so101_gripper_part1_v1", "moving_jaw_so101_gripper_v1", "moving_jaw_so101_v1",
    "rotation_pitch_so101_v1", "sts3215_03a_no_horn_v1", "sts3215_03a_v1", "under_arm_so101_v1",
    "upper_arm_so101_v1", "waveshare_mounting_plate_so101_v2",
    "wrist_roll_follower_so101_camera_mount", "wrist_roll_follower_so101_gripper_part0_v1",
    "wrist_roll_follower_so101_gripper_v1", "wrist_roll_follower_so101_v1",
    "wrist_roll_pitch_so101_v2") | ForEach-Object { "assets/$_.stl" })
foreach ($f in $files) {
    $dest = Join-Path $ModelDir $f
    if ((Test-Path $dest) -and ((Get-Item $dest).Length -gt 0)) { continue }
    Write-Host "download  $f"
    Invoke-WebRequest -UseBasicParsing -Uri "$Base/$f" -OutFile $dest
}
Write-Host "MuJoCo:  $MjDir"
Write-Host "SO-101:  $ModelDir"
