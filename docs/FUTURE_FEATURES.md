# Future features

Planned work, grouped by area and ordered by priority within each group. Known
limitations of the current code are tracked separately in
[LIMITATIONS_AND_ROADMAP.md](LIMITATIONS_AND_ROADMAP.md).

Status: **🟢 in progress** · **🟡 next** · **⚪ later**

---

## Direction: one hand-tracking engine, several front-ends

The project is growing from "gesture control for the desktop" into a
**hand-tracking engine with independent front-ends**:

| Front-end | What the hand controls | Status |
|---|---|---|
| **Desktop** | media keys, volume, brightness, windows, mouse | ✅ shipped (v2.0) |
| **Robot arm** | a simulated (later real) robot arm in 3D space | 🟢 in progress |

Each front-end gets its own subcommand, config file, docs folder and CI jobs.
The engine (tracking, landmarks, gestures, filters) never depends on a
front-end, and front-ends never depend on each other. New hand features (for
example a 3D hand position estimate) go into the engine so every front-end can
use them.

---

## Robot arm

### 🟢 Milestone 1: arm mimics the hand (MVP)

The smallest useful version: the arm follows the hand and the gripper follows
the fist.

| Behaviour | Definition |
|---|---|
| **Position** | The gripper moves to the point in the arm's 3D workspace that matches the hand's position in front of the camera: left/right → left/right, up/down → up/down, closer/further → forward/back |
| **Gripper closed** | Only when the hand is a **fist** (`Gesture::Fist`, after smoothing) |
| **Gripper open** | Any other hand shape |
| **Hand lost** | The arm holds its last position; the gripper keeps its last state |

Design:

- **Simulator:** [MuJoCo](https://mujoco.org) (Apache-2.0, native C API, runs
  headless for CI) with an arm model from
  [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie).
  Default model: **SO-ARM100/101**, so that a cheap open-source real arm can
  reuse the same kinematics later.
- **3D hand position (engine):** x and y from the palm centre in the image;
  depth from apparent palm size (wrist → middle knuckle, which doesn't change
  when the fingers curl into a fist). A short calibration ("hold an open palm
  where it's comfortable for 2 s") sets the origin and the reference palm size.
- **Workspace mapping:** a calibrated hand box maps linearly onto a reachable
  box in front of the arm, then gets clamped. One Euro filtering per axis plus
  a speed limit remove jitter.
- **Inverse kinematics:** position-only damped least squares using MuJoCo's
  Jacobian (`mj_jac`). The arm reaches any point inside its workspace;
  orientation is ignored in this milestone.
- **ArmBackend interface:** `state()`, `commandJoints()`, `commandGripper()`,
  `stop()`, so a real arm can replace the simulator without touching the
  mapping.
- **App:** `handtracker arm` opens the camera preview plus a MuJoCo viewer;
  `config/arm.ini` holds the workspace box, filters and model path.
- **Tests:** unit tests for depth estimation, workspace mapping and the
  gripper rule; a headless simulation test asserting that the end effector
  converges on reachable targets within 1 cm.

Delivered as four PRs:

1. `feat(engine)`: 3D hand position, calibration, grip state, with tests (no MuJoCo)
2. `feat(arm)`: `ArmBackend`, MuJoCo backend, IK, headless sim test, optional CMake target
3. `feat(app)`: `handtracker arm`, viewer, `config/arm.ini`
4. `docs(arm)`: `docs/arm/`, README section and demo GIF

### 🟡 Milestone 2: precise control

- **Clutch mode:** move the arm only while an "engage" gesture is held, using
  relative motion (like lifting a mouse), so the hand can reposition and the
  arm can cover more than the camera's view.
- **Wrist orientation** from the hand's 3D rotation (`computeOrientation`).
- **Proportional gripper** from pinch distance instead of open/closed only.
- **Stop gesture** (e-stop) and a **home pose** gesture.
- Safety limits: joint limits, maximum speed, keep-out zones.
- Record/replay of hand streams (`handtracker record`, `--source replay:`), so
  the arm can be developed and tested without a camera.

### ⚪ Milestone 3: real hardware

- SO-101 serial backend (Feetech bus servos).
- ROS 2 bridge, so any ROS-supported arm can be driven.
- Sim-to-real calibration of workspace and gripper.

### ⚪ Ideas

- Pick-and-place demo scene with objects in the simulator.
- Two-handed control: one hand positions the arm, the other works the gripper.
- Second camera (or stereo) for real depth instead of palm-size depth.
- Imitation-learning data capture (LeRobot format) from teleoperation sessions.

---

## Engine

- 🟡 **Restructure into `engine/` + `frontends/`** (no behaviour change), with a
  `HandFrame` stream between sources (camera, replay) and consumers (desktop, arm).
- 🟡 **Record / replay** of hand streams to `.jsonl`.
- 🟡 **Calibration wizard** shared by all front-ends (palm size, reach, comfortable range).
- ⚪ **Learned gesture classifier** with user-recorded custom gestures.
- ⚪ **GPU inference** (OpenCV DNN CUDA/OpenVINO targets) and a pipelined capture thread.
- ⚪ **Two-hand tracking** exposed to front-ends.

## Desktop

- 🟡 Bind depth mode to zoom or scroll.
- 🟡 Scroll and right-click gestures in mouse mode; multi-monitor support.
- ⚪ Per-application gesture profiles.
- ⚪ Linux and macOS system-control backends.
- ⚪ Tray app with pause/resume.

## Project

- 🟡 GitHub milestones (`v2.1 Restructure`, `v3.0 Robot arm`) with one issue per PR.
- 🟡 Demo GIFs for each front-end in the README.
- ⚪ Prebuilt Windows release (zip with OpenCV DLLs and models) attached to GitHub Releases.
