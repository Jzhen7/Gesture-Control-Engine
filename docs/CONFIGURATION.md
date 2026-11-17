# Configuration reference

The app reads `config/handtracker.ini` (searched in the working directory, then
next to the executable), or the file given with `--config`. The format is INI:
`[section]` headers, `key = value` lines, and comments starting with `#` or `;`
(also at the end of a line, after a space). Every key is optional. Unknown keys
and invalid values are reported as warnings at startup and ignored.

Booleans accept `true/false`, `yes/no`, `on/off` and `1/0`.

## `[camera]`

| Key | Default | Meaning |
|---|---|---|
| `index` | `0` | Camera device index (`--camera` overrides it) |
| `width`, `height` | `640`, `480` | Requested capture size |
| `mirror` | `true` | Flip horizontally (selfie view) |

## `[models]`

| Key | Default | Meaning |
|---|---|---|
| `palm` | `models/palm_detection_mediapipe_2023feb.onnx` | Palm detector path |
| `landmark` | `models/handpose_estimation_mediapipe_2023feb.onnx` | Landmark model path |
| `palm_score` | `0.5` | Minimum palm detection confidence |
| `nms_threshold` | `0.3` | IoU threshold for non-max suppression |
| `landmark_score` | `0.8` | Minimum landmark confidence; below this the hand is dropped |
| `max_hands` | `1` | Hands to track (only the first drives the controller) |
| `tracking` | `true` | Reuse last frame's landmarks instead of re-running the palm detector |

Relative model paths are resolved against the working directory, then the
executable's directory and its parents.

## `[preview]`

| Key | Default | Meaning |
|---|---|---|
| `enabled` | `true` | Show the camera window (`--no-preview` disables it) |
| `topmost` | `true` | Keep the window above others (Windows) |
| `show_stats` | `true` | Draw fps and inference time |

## `[classifier]`

| Key | Default | Meaning |
|---|---|---|
| `finger_extension_ratio` | `1.1` | Finger is "up" when tip-to-wrist > ratio × pip-to-wrist |
| `thumb_extension_ratio` | `1.05` | Thumb is "out" when tip-to-pinky-knuckle > ratio × IP-to-pinky-knuckle |
| `smoothing_frames` | `5` | Majority vote window; larger = steadier but slower to react |

## `[temporal]`

| Key | Default | Meaning |
|---|---|---|
| `swipe_window_ms` | `300` | Sliding window for swipe/flip detection |
| `swipe_distance` | `0.15` | Net wrist movement (fraction of frame) that counts as a swipe |
| `hold_ms` | `1000` | Time a gesture must be held still |
| `hold_max_motion` | `0.08` | Max wrist drift (fraction of frame) during a hold |
| `cooldown_ms` | `400` | No new events for this long after one fires |
| `flip_min_separation` | `0.15` | Knuckle separation (palm sizes) needed to tell palm from back |

## `[modes]`

| Key | Default | Meaning |
|---|---|---|
| `brightness_gesture` | `peace` | Gesture that adjusts brightness in brightness mode |
| `volume_gesture` | `three` | Gesture that adjusts volume in volume mode |
| `depth_gesture` | `one` | Gesture read in depth mode |
| `exit_gesture` | `palm` | Leaves any mode |
| `hand_lost_timeout_ms` | `3000` | Leave the mode after the hand is gone this long |
| `volume_min_angle`, `volume_max_angle` | `-60`, `60` | Roll angles mapped to 0% and 100% |
| `brightness_top`, `brightness_bottom` | `0.2`, `0.8` | Palm height (fraction of frame) for 100% and 0% |
| `volume_smoothing`, `brightness_smoothing` | `0.4`, `0.3` | EMA weight of the newest sample |
| `volume_interval_ms`, `brightness_interval_ms` | `40`, `150` | Minimum time between OS updates |

## `[mouse]`

| Key | Default | Meaning |
|---|---|---|
| `margin` | `0.12` | Frame border ignored on each side; the inner box maps to the screen |
| `pinch_on` | `0.25` | Thumb–index distance (palm sizes) that presses the button |
| `pinch_off` | `0.35` | Distance that releases it (must exceed `pinch_on`) |
| `min_cutoff` | `1.0` | One Euro filter cutoff (Hz): lower = smoother, laggier |
| `beta` | `0.02` | One Euro speed coefficient: higher = less lag when moving fast |

## `[arm]` (robot-arm front-end, in `config/arm.ini`)

`handtracker arm` reads `config/arm.ini` by default. It uses the same
`[camera]`, `[models]` and `[classifier]` sections as above, plus:

| Key | Default | Meaning |
|---|---|---|
| `model` | `models/so101/scene.xml` | MuJoCo scene (from `scripts/download_arm.*`) |
| `calibration_ms` | `2000` | How long to hold the open palm when calibrating |
| `calibration_max_motion` | `0.05` | Max palm drift (fraction of frame) while calibrating |
| `range_x`, `range_y` | `0.30`, `0.25` | Hand offset from the calibration pose (fraction of frame) that reaches the workspace edge |
| `depth_range` | `0.35` | Change in ln(palm size) that reaches the front/back of the workspace |
| `workspace_min`, `workspace_max` | `0.17 -0.13 0.06`, `0.31 0.13 0.24` | Reachable box in metres, arm frame (X forward, Y left, Z up) |
| `min_cutoff`, `beta` | `1.0`, `5.0` | One Euro filter on the arm target |
| `max_speed` | `0.6` | Fastest the target may move (m/s) |
| `gripper_open`, `gripper_closed` | `1.2`, `0.0` | Jaw joint angles in radians |
| `gripper_mode` | `proportional` | `proportional`: jaws follow the fingertip claw; `binary`: fist = closed, anything else = open |
| `grip_closed_spread` | `0.25` | Fingertip spread (mean distance between all pairs of fingertips ÷ palm length) that counts as fully closed |
| `grip_deadband` | `0.1` | Snap to exactly closed/open within this fraction of each end |
| `grip_min_straightness` | `0.8` | How straight *every* finger must be (1 = straight) to count as a claw; below it the gripper holds |
| `grip_min_cutoff`, `grip_beta` | `2.0`, `2.0` | One Euro filter on the gripper opening |
| `ik_iterations` | `10` | IK refinement steps per frame |
| `ik_damping` | `0.05` | IK damping (stability near full stretch) |
| `ik_null_gain` | `0.05` | Pull toward a comfortable pose; keep it above 0 |

Vectors are three numbers separated by spaces or commas.

## `[bindings]`

```ini
<gesture> <event> = <action> [@process.exe]
```

If the file has a `[bindings]` section, it **replaces** all default bindings.
The first binding that matches a gesture + event wins.

| Part | Values |
|---|---|
| gesture | `fist` `peace` `middle` `one` `two` `three` `four` `palm` |
| event | `hold` `swipe_left` `swipe_right` `swipe_up` `swipe_down` `flip` |
| action | `keys:<chord>`, `media:<key>`, `mode:<mode>`, `launch:<target>`, `quit` |
| `@process` | Fire only while this executable is running (case-insensitive) |

**Key names** for `keys:` (joined with `+`): `ctrl alt shift win`, `a`–`z`,
`0`–`9`, `f1`–`f24`, `tab enter esc space backspace delete insert home end
pageup pagedown left right up down printscreen`, `media_next media_prev
media_playpause media_stop volume_mute volume_up volume_down browser_back
browser_forward browser_refresh`.

**Media shortcuts** for `media:`: `next prev playpause stop mute volup voldown`.

**Modes** for `mode:`: `brightness volume depth`.

**Launch** targets are passed to the shell, so programs on `PATH`, full paths,
documents and URLs all work. Put the `@process` suffix after a space.

### Examples

```ini
[bindings]
palm swipe_up    = keys:win+d                  # show desktop
palm swipe_down  = keys:win+d
peace hold       = launch:https://github.com
two swipe_left   = keys:alt+left   @chrome.exe # browser back
two swipe_right  = keys:alt+right  @chrome.exe
three flip       = keys:alt+tab
fist hold        = mode:brightness
four hold        = mode:volume
one hold         = media:playpause
middle hold      = quit
```
