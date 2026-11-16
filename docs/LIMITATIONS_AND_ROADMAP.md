# Limitations and roadmap

This document has three parts: what was limiting in the original Python
project and how v2 deals with it, what is still limited in v2, and where the
project could go next.

---

## 1. Limitations found in v1 and how v2 handles them

| # | v1 limitation | Where in v1 | v2 resolution |
|---|---|---|---|
| 1 | **Upright-only fingers.** `tip.y < base.y` fails as soon as the hand tilts, and volume mode *requires* tilting | `find_gesture` | Distance-ratio tests relative to the wrist; rotation-invariant (unit-tested at −150°…180°) |
| 2 | **Handedness-dependent thumb.** `landmark[4].x > landmark[3].x` inverts for the left hand or a mirrored feed | `find_gesture` | Thumb measured against the pinky knuckle; works for both hands |
| 3 | **Frame-count timing.** "6 frames ≈ 0.3 s at 20 fps": swipes and holds change with CPU load and camera | `POSITION_BUFFER_SIZE`, `STATIONARY_FRAMES` | All windows in milliseconds |
| 4 | **Double-firing.** No cooldown; a held gesture re-triggered every second | main loop | Cooldown after every event; holds latch until the gesture changes |
| 5 | **Holds while moving.** A hold only checked the label, not motion, so modes were entered accidentally | stationary check | Hold also requires the wrist to stay within `hold_max_motion` |
| 6 | **Flicker.** A single misclassified frame reset everything | `gesture_buffer` | Majority-vote smoother |
| 7 | **Stuck in modes.** A mode could only be left with an open palm, even after the hand left the frame | mode flags | Hand-lost timeout |
| 8 | **Distance-dependent brightness.** Brightness came from apparent finger length, which changes with distance to the camera | `dy_to_brightness` | Uses palm height in the frame, independent of distance |
| 9 | **Volume dial discontinuity.** Angle mapping wrapped around ±180° and fell back to 50% outside the range | `degree_to_volume` | Roll measured from vertical, linear between configurable angles, clamped |
| 10 | **Blocking OS calls.** `sbc.set_brightness` (WMI) ran every frame on the video loop; `psutil.process_iter` scanned all processes on every swipe | main loop | Worker thread with coalescing, rate-limited level filter, 2 s process cache |
| 11 | **Hard-coded bindings.** Changing a gesture meant editing an `if/elif` chain | main loop | INI bindings with process gating, launch actions, key chords |
| 12 | **Mutually exclusive mode booleans.** Four flags that could in principle all be true | `brightness_mode`, … | Single `Mode` enum state machine |
| 13 | **Mouse jitter and edge freeze.** Raw landmark → cursor; outside the margin the cursor stopped updating; magic numbers (`603`, `37`) tied to one camera | `mouse_mirroring.py` | One Euro filter, edge clamping, resolution-independent mapping |
| 14 | **Palm-to-drag.** Open palm held the mouse button; any misread dropped what you were dragging | `mouse_mirroring.py` | Pinch with hysteresis; button always released when the hand is lost |
| 15 | **DPI scaling.** pyautogui coordinates are wrong on scaled displays | — | Process marked DPI-aware |
| 16 | **Orientation "up" from fingertip.** Curling the middle finger swung the computed hand axis | `hand_up_vector` | Uses the middle knuckle; prefers metric world landmarks |
| 17 | **Calibration that wasn't.** `time.sleep(2)` inside the loop froze the video and stored nothing | `hand_tracker_bare.py` | Removed. Real calibration is on the roadmap below |
| 18 | **Windows-only, untestable.** Direct pycaw/win32 calls everywhere; no tests | everywhere | `SystemControl` interface, null backend, unit tests, CI on 3 OSes |
| 19 | **Three tkinter windows.** Separate windows each needing `root.update()` | `bar.py` | Overlay drawn into the preview |
| 20 | **Heavy runtime.** Python + MediaPipe + OpenCV + ~10 packages | imports | One executable + OpenCV |
| 21 | **Minor bugs.** `bar.py` example calls `Bar(root)` without required args; `margin` used before assignment on the first frame in mouse mode; brightness passed 0–100 to a 0–1 bar; volume vs. brightness bar title never changed | various | Fixed by construction in the rewrite |

---

## 2. Known limitations of v2

### Recognition

- **Heuristic classifier.** Finger states come from geometric rules. They're
  robust to rotation and scale, but can misread fingers pointing straight at
  the camera (foreshortening) and partially bent fingers. The thresholds are
  configurable but global.
- **Fixed vocabulary.** Eight static gestures; no way to record your own.
- **Single controlling hand.** `max_hands` can track more, but only the first
  hand drives the controller, and two-handed gestures aren't supported.
- **Flip is direction-agnostic.** Turning the hand back over counts as a second
  flip (after the cooldown). Telling palm-forward from back-of-hand reliably
  needs handedness and is left for later.
- **Lighting and background.** Inherited from the models: low light, motion
  blur, gloves and skin-coloured backgrounds reduce accuracy.

### Pipeline

- **CPU inference only by default.** OpenCV DNN supports CUDA/OpenVINO targets,
  but they aren't exposed in the config yet.
- **Sequential pipeline.** Capture, inference and drawing run on one thread, so
  inference time caps the frame rate.
- **Tracking drift.** Tracking stops only when landmark confidence drops below
  `landmark_score`. A fast-moving hand can be lost for a frame before
  re-detection.

### Platform

- **Windows-only OS actions.** Linux and macOS builds track hands and run the
  gesture logic, but actions are only logged.
- **Brightness hardware support.** WMI works on most laptop panels and DDC/CI on
  many external monitors; some monitors and docks support neither.
- **Elevated windows.** Windows blocks injected input into apps running as
  administrator unless the hand tracker is also elevated (UIPI).
- **Primary monitor only** for mouse mode.
- **Depth mode is read-only.** It shows a distance proxy but isn't bound to an
  action.

---

## 3. Roadmap and expansion ideas

Roughly ordered by value for effort.

### Near term

- [ ] **Calibration wizard.** Record the user's open palm, fist and comfortable
  reach to set per-user thresholds, the mouse active region and the volume angle
  range. v1's `ideas.txt` asked for this.
- [ ] **Linux backend.** PipeWire/PulseAudio volume, `ddcutil`/`/sys/class/backlight`
  brightness, XTest or `uinput` input, `/proc` process checks.
- [ ] **macOS backend.** CoreAudio volume, `CGEvent` input, `IODisplay` brightness.
- [ ] **Hot reload** of `handtracker.ini` when it changes.
- [ ] **Accelerator selection.** Expose `cv::dnn` backend/target (CUDA, OpenVINO,
  OpenCL) in `[models]`.
- [ ] **Bind depth mode** to zoom (`ctrl+plus/minus`) or scroll.
- [ ] **Scroll gesture** in mouse mode (two-finger vertical motion), plus right-click
  (thumb + middle pinch).
- [ ] **Multi-monitor mouse** mapping across the virtual desktop.

### Medium term

- [ ] **Learned gesture classifier.** Replace the rule table with a small MLP on
  normalised landmarks (MediaPipe's gesture recogniser approach). Ship a
  `record` command so users can add custom gestures from a few samples.
- [ ] **Dynamic gestures.** Circles, waves and pinch-and-drag as sequences
  (DTW or a small temporal network) rather than single displacements.
- [ ] **Two-handed gestures.** For example, pinch with both hands and pull apart
  to zoom.
- [ ] **Per-application profiles.** Switch binding sets by foreground window
  (Spotify, Chrome, PowerPoint, games).
- [ ] **Pipelined capture/inference** on separate threads with a latest-frame queue.
- [ ] **Tray app.** Run without a console, toggle the preview, pause tracking
  with a hotkey or gesture.
- [ ] **On-screen feedback** without the preview window: a small transparent
  HUD showing the current gesture and mode.

### Exploratory

- [ ] **3D interaction / CAD.** v1's notes imagined "CAD/modelling using the
  camera". The orientation app already computes a full hand rotation matrix
  from metric world landmarks; streaming it (with pinch as grab) to Blender or
  a three.js viewer over WebSocket/OSC would make the hand a 6-DoF controller.
- [ ] **OSC / MIDI output** for music software and lighting rigs.
- [ ] **Accessibility profile.** Dwell-to-click and larger hold timers for users
  who can't use a mouse.
- [ ] **Sign-language fingerspelling** demo on top of the learned classifier.
- [ ] **Plugin actions.** Load action handlers from shared libraries or scripts.
