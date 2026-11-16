# Changelog

All notable changes to this project are documented here. The format follows
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and the project uses
[Semantic Versioning](https://semver.org/).

## [Unreleased]

### Added
- **Robot-arm front-end** (`handtracker arm`): a simulated SO-101 arm (MuJoCo
  3.17) follows the hand in 3D; the gripper closes on a fist.
- Calibrated 3D hand position in the engine, with depth from apparent palm size.
- `ArmBackend` interface, workspace mapping, `ArmController` with One Euro
  filtering, speed limit, clamping and hold-on-hand-loss.
- Damped-least-squares inverse kinematics with a null-space bias toward a
  ready pose.
- OpenCV-drawn 3D arm view alongside the camera preview.
- `config/arm.ini`, `[arm]` config section, `scripts/download_arm.{ps1,sh}`.
- Unit tests for the arm logic and headless MuJoCo simulation tests in CI.
- `desktop` subcommand (alias of `control`).
- Docs: Dash 1 (C++ rewrite) and Dash 2 (robot arm) explanations, future features plan.
- Windows builds copy OpenCV's DLLs next to `handtracker.exe` (CMake 3.21+), so
  a prebuilt OpenCV runs without editing `PATH`.

## [2.0.0] - 2026-11-16

Complete rewrite of the Python prototype in C++17.

### Added
- Native MediaPipe hand pipeline (palm detection + 21-point landmarks) on OpenCV
  DNN, with frame-to-frame tracking.
- Rotation-, scale- and handedness-invariant gesture classifier with a
  majority-vote smoother.
- Time-based swipe, flip and hold detection with cooldown and latching.
- INI configuration for every threshold and gesture binding, including key
  chords, media keys, app/URL launching and process-gated bindings.
- Brightness, volume and depth modes as an explicit state machine with a
  hand-lost timeout.
- Native Windows backend: Core Audio volume, WMI + DDC/CI brightness,
  SendInput keyboard/mouse, Toolhelp process detection, DPI awareness.
- Background worker thread for OS calls, with coalescing of continuous values.
- Mouse mode with One Euro filtering, edge clamping and pinch-to-click.
- Orientation mode showing 3D hand rotation from metric world landmarks.
- On-frame overlay: skeleton, hold progress ring, level bar, toasts, fps.
- `--dry-run`, `--no-preview` and `--image` (still-image smoke test) options.
- Unit tests and GitHub Actions CI on Windows, Linux and macOS.
- Documentation: architecture, gestures, configuration, building, and
  limitations/roadmap.

### Changed
- Brightness follows hand height instead of apparent finger length.
- Volume dial is measured from vertical and maps linearly between configurable angles.
- Mouse click is a pinch instead of an open palm.

### Removed
- tkinter bar windows (replaced by the overlay).
- The non-functional calibration step.

## [1.0.0] - 2025-07

Original Python prototype ([Jzhen7/Hand-Tracker](https://github.com/Jzhen7/Hand-Tracker)).
