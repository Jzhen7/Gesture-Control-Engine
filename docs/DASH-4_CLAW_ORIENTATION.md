# Dash 4: claw orientation from hand orientation

> **What this doc is:** how the claw's orientation follows your hand's, and,
> above all, *why* it works the way it does. This dash had more design
> choices than any before it, so section 12 lists every decision with its
> reason, and section 13 lists the alternatives that were rejected.
> It builds on [Dash 2](DASH-2_ROBOT_ARM.md) (the arm, IK, coordinate frames)
> and [Dash 3](DASH-3_LIVE_CLAW.md) (the claw gesture).

**Status:** implemented and tested with synthetic 3D hands and in the MuJoCo
simulation (99/99 unit tests, 15/15 simulation tests on Windows 11, MSVC 2022).
**Not yet tried live with a webcam** (section 14, item 1).

**In one sentence:** after calibrating, **tip your fingers toward the camera
and the claw tips down; twist your hand like a doorknob and the claw twists
the same way.** Position and gripper work exactly as in Dashes 2 and 3.

| Your hand (relative to the calibration pose) | The claw |
|---|---|
| fingertips tip **toward** the camera (wrist flexion) | **pitches down** |
| fingertips tip **away** from the camera | **pitches up** |
| **twist** about the finger axis (forearm rotation) | **rolls** the same way about its own pointing axis |
| **side bend** (fingers lean left/right) | nothing: the arm physically can't (section 2) |
| hand leaves the view | holds its orientation |

```powershell
build\Release\handtracker.exe arm   # calibrate: open palm, facing the camera, fingers up, 2 s
```

The camera panel shows your hand's `flex / twist / side` angles. The 3D view
shows the claw's `pitch` and `roll`, the values asked for, and `LIMITED` when
the arm can't reach the requested pitch at that spot.

---

## 1. What "orientation" means for a gripper

Any rigid object can turn three ways:

| Turn | Gripper meaning | Hand equivalent |
|---|---|---|
| **Pitch** | point more down / more up | wrist flexion: fingers toward / away from the camera |
| **Roll** | spin about its own pointing axis | forearm rotation: the doorknob twist |
| **Yaw** | point more left / more right | wrist side bend |

Together with the 3 position numbers, that's **6 degrees of freedom** (DOF)
for full control of a gripper in space.

---

## 2. The arm can only do 5 of the 6

The SO-101 has 5 joints (plus the gripper). Probing the MuJoCo model with a
throwaway Python script showed their axes:

| Joint | Turns about | Effect |
|---|---|---|
| `shoulder_pan` | vertical | swings the whole arm left/right |
| `shoulder_lift` | horizontal, the arm's "left" | ⎫ |
| `elbow_flex` | the **same** horizontal axis | ⎬ a flat 3-link chain in a vertical plane |
| `wrist_flex` | the **same** horizontal axis | ⎭ |
| `wrist_roll` | exactly the claw's **pointing** axis | spins the claw (measured: axis = −1 × pointing axis) |

What that means:

- **Position (3 numbers)** uses pan + the vertical-plane chain.
- **Pitch** is the sum of the three parallel joints' angles. The chain has
  one more joint than a 2D position needs, so it has **one spare motion**
  that changes pitch without moving the fingertip. Dash 2 used that spare
  motion to keep the arm near a comfortable pose. Dash 4 uses it for pitch.
- **Roll** is one joint, `wrist_roll`, about the pointing axis itself. It's
  set directly, with no IK needed.
- **Yaw is not free.** Only the pan turns the arm sideways, and the pan is
  already fixed by *where* the claw is (it must face the target). The claw
  always points within the vertical plane through the base. A 6th joint
  would be needed to yaw independently.

**Decision:** control **pitch and roll**, ignore **side bend**. The hand's
side bend is still measured and shown on screen, so a future 6-DOF arm can
use it.

---

## 3. The pipeline

| # | Step | Code |
|---|---|---|
| 1 | Landmarks (world 3D points preferred) | engine (Dash 1) |
| 2 | Convert to **arm space** (forward, left, up) | `toArmSpace()` |
| 3 | Build the **hand frame** from palm landmarks | `handFrame()` |
| 4 | Split rotation since calibration into **flex, twist, side** | `handAngles()` |
| 5 | **One Euro filter** flex and twist | `ArmController::orient()` |
| 6 | **Map**: pitch = neutral − flex; roll = twist (× gains, clamped) | `orient()` |
| 7 | **Turn-rate limit** (180°/s) | `orient()` |
| 8 | **IK**: position first, pitch in the spare motion, roll direct; position is checked and pitch gives way if needed | `MujocoArm::solveIk()` |
| 9 | Servos, physics, draw | Dash 2 |

In parallel, Dash 2's depth step gained a **tilt correction** (section 7),
because tipping your hand would otherwise move the arm.

---

## 4. Measuring the hand's orientation

### 4.1 One frame for hand and arm ("arm space")

Landmarks come in camera terms (x right, y **down**, z **away** from the
camera). They're converted to the arm's own convention, as if you stood
behind the arm facing the same way:

```text
arm space  X = forward (toward the camera) = −z
           Y = your left                   = −x   (mirrored feed: image right = your right)
           Z = up                          = −y
```

**Why:** this frame is right-handed, like the arm's. A rotation measured on
the hand can be applied to the claw unchanged, and every sign convention
below is a physical one, not an artifact of image coordinates. (Measuring
in raw image coordinates, which are mirrored and y-down, would silently
reverse some rotation directions.)

### 4.2 The hand frame

Built **only from palm landmarks** (wrist, index knuckle, middle knuckle,
pinky knuckle):

| Axis | Definition | At calibration |
|---|---|---|
| `y` fingers | wrist → middle knuckle | up |
| `z` front | palm normal, flipped so it faced the camera at calibration | forward |
| `x` across | y × z | your left |

- **Palm only, so the claw gesture can't disturb it.** Pinching or spreading
  the fingertips (Dash 3) doesn't change the knuckles. Tested: open hand,
  claw and fist give identical twist (< 0.0001 rad).
- **The "front" sign is fixed at calibration.** The palm-normal cross
  product points out of the palm for one hand and out of the back for the
  other. Rather than rely on the model's left/right guess, calibration
  records which sign made it face the camera and reuses it all session.
  Tested: a mirrored hand (= the other hand, or the back of the hand)
  gives the same angles.

### 4.3 Splitting the rotation: flex, twist, side

The hand's rotation since calibration is split with a **swing-twist
decomposition** about the finger axis:

1. **Swing:** where do the fingers point now, seen in the calibration frame?
   - `flex = atan2(fingers·front, fingers·fingers_cal)`
   - `side = asin(fingers·across)`
2. **Twist:** undo the swing (the shortest rotation that carries the old
   finger axis onto the new one). Whatever rotation is left about the
   finger axis is the twist.

**Why swing-twist and not standard Euler angles:** Euler angles (rotX, rotY,
rotZ, as `computeOrientation()` reports) depend on the order of the
rotations and get mixed up when two are combined. Swing-twist measures the
twist about the hand's *current* finger axis, so it means "doorknob" even
when the hand is also flexed. Tested: twisting 40° then flexing 30° reads
back 40° and 30°.

**Sign conventions** (right-hand rule in arm space):
- `flex` + = fingertips toward the front of the hand (the camera, at calibration)
- `twist` + = counter-clockwise when looking back along the fingers from their tips
- `side` + = fingers lean toward the hand's `x` axis (your left, at calibration)

---

## 5. Mapping hand angles onto the claw

### 5.1 "Lay your hand forward"

The calibration pose has fingers **up**, but the claw at rest points
**forward and down**. The mapping is **relative** to calibration, and it
helps to picture your hand rotated forward to lie along the claw:

| Hand | ↔ | Claw |
|---|---|---|
| fingers | ↔ | pointing direction |
| front (palm side facing the camera) | ↔ | underside |
| across (your left) | ↔ | the claw's left |

With that picture:
- tipping your fingers toward your front = the claw tipping toward its
  underside = **pitch down**
- twisting about your fingers = twisting about its pointing axis, same
  direction = **roll**

```text
claw pitch = neutral_pitch − pitch_gain × flex      (clamped to −90°…+30°)
claw roll  =                 roll_gain  × twist     (clamped to ±150°)
```

### 5.2 Defaults and why

| Setting | Default | Why |
|---|---|---|
| `neutral_pitch_deg` | **−35°** | Pitches around −30° to −45° are reachable at the most workspace points (110 and 104 of 125, section 6.4). −35° sits between them, a natural "reach forward and down" angle. |
| `pitch_gain`, `roll_gain` | **1.0** | 1:1 is the easiest to learn ("the claw does what my hand does"). |
| `pitch_min_deg` / `pitch_max_deg` | **−90° / +30°** | Straight down is useful for picking things up. Above +30° is reachable at fewer than 1 in 3 points. |
| `roll_limit_deg` | **±150°** | The joint stops at about ±157°; this leaves a margin. |
| `orient_min_cutoff`, `orient_beta` | **1.0 Hz, 0.5** | Hand angles are noisier than position. A lower cut-off than the gripper's 2 Hz trades a little lag for a steadier wrist. |
| `max_angular_speed_deg` | **180°/s** | Like the position speed limit: a mis-tracked frame can't spin the wrist round in one go. |

A negative gain reverses a direction, which is the escape hatch if a sign
turns out wrong live (section 14, item 2).

---

## 6. Inverse kinematics: position first, pitch second

This was the hardest part. The first two designs failed in testing; the
numbers below are what drove each change.

### 6.1 The pitch task

Pitch is measured in the arm's vertical plane: the claw's pointing axis
against the arm's horizontal heading. The heading is taken from the
`shoulder_lift` axis rather than the claw, so the angle stays defined even
with the claw pointing straight down. Because lift, elbow and wrist flex all
turn about that same axis, how fast pitch changes per joint is exact and
simple: `−(joint's angular velocity · arm's left axis)`. That comes straight
from MuJoCo's rotational Jacobian.

### 6.2 Attempt 1: pitch in the null space (Python prototype)

Standard "task priority" IK: solve position with damped least squares, and
add pitch only through the null-space projector `N = I − J⁺J`, so it can't
move the fingertip. The textbook says that's enough.

| Result | |
|---|---|
| Position error with an **unreachable** pitch | up to **152 mm** at level pitch, **299 mm** at +45° |

**Why it failed:** where the requested pitch is impossible, the pitch task
pushes joints into their limits. They get clamped *after* the step, which
breaks the projection, so the fingertip leaves its target.

### 6.3 Attempt 2: joint-limit-aware steps (C++)

A joint that would leave its range is parked on its limit and the step is
re-solved without it (a small "active set" loop, up to 4 passes).

| Result | |
|---|---|
| Pitch where reachable | within 2° |
| Position error with an unreachable pitch | up to **193 mm** (worse) |

**Why it failed:** with a joint parked, the remaining three can't always
reach the point. The pitch task "won" by trapping the arm somewhere the
position couldn't be reached.

### 6.4 Attempt 3 (shipped): check, then give way

Position priority is now **enforced** rather than hoped for:

1. Solve with the requested pitch.
2. Solve with **no** pitch request (the Dash 2/3 solve, which reaches the
   whole workspace).
3. If step 1 lands more than 1 mm worse than step 2, the pitch isn't
   reachable here. **Bisect** between the free pitch (reachable) and the
   requested one (not), 6 steps (1/64 of the gap), and keep the closest
   pitch that still reaches the target.
4. Report `pitchLimited`, so the view can show `LIMITED`.

Reachability table (`sim: pitch reachability…` test; 5×5×5 grid over the
workspace, 300 iterations per solve):

| Pitch asked | Reached (pos < 2 mm and pitch < 2°) of 125 | Worst position error |
|---|---|---|
| −90° (straight down) | 18 | 3.0 mm |
| −75° | 48 | 3.2 mm |
| −60° | 74 | 3.5 mm |
| −45° | 104 | 2.5 mm |
| **−30°** | **110** | 3.2 mm |
| −15° | 97 | 2.0 mm |
| 0° (level) | 78 | 3.4 mm |
| +15° | 50 | 2.2 mm |
| +30° | 37 | 2.2 mm |
| +45° | 7 | 2.4 mm |

The worst position error is now **3.5 mm** (it was 193 mm), against 2.5 mm
for a position-only solve. **Pointing straight down works at only 18 of 125
points**, mostly low and far out. Near the base or up high, the claw gives
way toward about −30°. That's a property of the SO-101's geometry, not of
the solver.

### 6.5 Problem found in tracking: warm start

The static solves above use 300 iterations. Live, the arm gets 10 per frame.
A simulated tracking run (30 random targets, pitches −60…−20°, rolls ±90°)
missed by up to **44.7 mm**.

**Why:** each frame's solve restarted from the arm's *physical* pose, which
lags behind the servos. With the wrist parked at a limit, both the pitch
solve and the no-pitch reference made too little progress in 10 iterations,
so the check in 6.4 compared two equally stuck answers and passed.

**Fix:** each frame now starts from **the previous frame's solution** (a
"warm start", standard practice), so the iterations add up across frames.
Worst tracking miss: **2.0 mm**. Pitch error where reachable: **0.06°**.

### 6.6 Effect on Dash 2/3 (no pitch requested)

The joint-limit handling and warm start also apply when orientation is off.
Comparison against the Dash 3 solver (built from `main`, same 125 points):

| Position-only | Dash 3 solver | Dash 4 solver |
|---|---|---|
| 10 iterations (one frame) | worst 1.6 mm, mean 0.76 mm | identical |
| 300 iterations | worst **6.4 mm** (slow drift into a joint limit) | worst **2.5 mm** |
| Time per 10-iteration solve | 8.1 µs | 8.3 µs |

So it is never worse, and it fixes a slow drift the old solver had. The
worst case with pitch (8 solves per frame) is about 70 µs, against a 33 ms
frame budget.

---

## 7. The coupling problem: tipping your hand moved the arm

Dash 2 estimates depth from **how big the palm looks**. Tipping your hand
forward to pitch the claw makes the palm look *shorter*, which reads as
"moved away", and the arm would drive backwards. That was listed as a Dash 2
limitation ("tilting your hand also changes its apparent size"); Dash 4
makes tilting deliberate, so it had to be fixed.

**Fix (`tilt_correction`, default on):** measure how much of the palm lies
in the image plane using the 3D world points, and divide the on-screen palm
length by that fraction (clamped at 0.3, about 72°, beyond which it isn't
trusted).

| Hand flexed by | Depth reading without correction | With correction |
|---|---|---|
| 20° | −0.18 (of the ±1 range) | 0.00 |
| 40° | **−0.76** (≈ 5 cm of arm travel) | 0.00 |
| −30° | −0.41 | 0.00 |

**Discovered along the way:** Dash 2's palm size already includes the
model's per-point depth guess (image z), so it partly compensates already.
The test therefore flattens image z to isolate the correction. Which measure
works better live (the image z or the world points) is unknown until a
webcam test, so `tilt_correction = false` restores the Dash 2 measure for
comparison.

**Remaining coupling** (tested): twisting the hand 60° moves the arm target
**0 mm**. Flexing it 30° moves it **5.3 mm**, because the palm centre
(midway between wrist and middle knuckle) swings as the hand pivots at the
wrist.

---

## 8. Safety and smoothness

| Mechanism | Protects against |
|---|---|
| Position priority with checked fall-back (6.4) | an orientation request dragging the claw off target |
| Pitch clamp (−90…+30°) and roll clamp (±150°) | asking for the impossible |
| Turn-rate limit (180°/s) | a mis-tracked frame spinning the wrist |
| One Euro filter on flex/twist | landmark jitter making the wrist shiver |
| Hold on hand loss | wandering when you step away |
| Recalibrate (`c`) resets the neutral orientation | drift in what "neutral" feels like |
| `orientation = false` | exact Dash 3 behaviour (pitch free, roll 0) |

---

## 9. Code changes

| File | Change |
|---|---|
| `core/hand_pose.*` (**new**) | arm-space conversion, hand frame, front sign, swing-twist `handAngles()`, `palmInPlaneFraction()` |
| `core/hand_space.*` | Calibration stores the neutral `frame` and `frontSign` (averaged over the 2 s); optional tilt-corrected palm size |
| `arm/arm_backend.hpp` | **`WristTarget`** (optional pitch, roll) + `setWristTarget()`; `ArmState` gains pitch, roll, `pitchLimited`, claw axes |
| `arm/mujoco_arm.*` | `pitchOf()`; `solveIk(target, wrist, …)` with the pitch task, joint-limit active set, position check + bisection, warm start; old `solveIk(target, q)` kept |
| `arm/arm_controller.*` | `orient()`: angles → filter → map → clamp → rate limit; neutral wrist on calibration |
| `core/config.*`, `config/arm.ini` | orientation keys (degrees in the file), IK pitch keys, `tilt_correction` |
| `apps/arm_view.cpp` | Jaws drawn from the claw's real 3D axes (pitch and roll visible) plus a pointing line; pitch/roll readout with `LIMITED` |
| `apps/arm_app.cpp` | flex/twist/side readout; start-up line |
| tests | `rotateHand()`: turns a synthetic hand in 3D (image and world points). 17 new unit tests, 6 new simulation tests (section 10). |

**`pitch` is `std::optional`:** "no pitch request" (let the arm pick, the
Dash 2/3 behaviour) is a real state, distinct from "pitch 0" (level). An
optional makes the two impossible to confuse.

---

## 10. Testing

| Test | Proves |
|---|---|
| hand frame at calibration | fingers up, front forward, across left |
| flex / twist / side, each alone | each is read back correctly (±1°) and doesn't leak into the others |
| twist while flexed | the swing-twist split gives both back |
| other hand / back of hand | the same angles (front-sign logic) |
| fingers curled | orientation unchanged (palm-only frame) |
| calibrator | records the neutral frame |
| tilt correction | flex doesn't change depth (table in section 7) |
| claw at any orientation | Dash 3 openness and claw check unaffected (±0.01) |
| controller: neutral, flex → pitch down, twist → roll | the mapping and its signs |
| controller: gains, clamps, rate limit | roll ×2 clamped to 60°; a sudden 50° twist limited to one frame's worth |
| controller: hand lost | holds |
| controller: turning the hand | arm target moves 0 mm (twist) / 5.3 mm (flex) |
| controller: orientation off | Dash 3 behaviour |
| config | every key, degrees converted, bad value warned |
| **sim:** pitch −35/−60/−15° at the centre | reached within 0.05°, position within 0.25 mm |
| **sim:** roll +45° | jaws physically turn **+44°** about the claw axis (right-hand rule), pitch unchanged |
| **sim:** roll beyond the joint | clamped |
| **sim:** unreachable pitch (−90° near the top) | gives way to −30°, position 1.1 mm, `pitchLimited` |
| **sim:** tracking moving pose targets | worst 2.0 mm, pitch 0.06° where reachable |
| **sim:** reachability report | the table in 6.4 |

---

## 11. Settings (`config/arm.ini`, `[arm]`)

| Key | Default | Meaning |
|---|---|---|
| `orientation` | `true` | follow hand flex (pitch) and twist (roll) |
| `neutral_pitch_deg` | `-35` | claw pitch for the calibration pose |
| `pitch_gain`, `roll_gain` | `1.0` | claw degrees per hand degree; negative reverses |
| `pitch_min_deg`, `pitch_max_deg` | `-90`, `30` | requestable pitch range |
| `roll_limit_deg` | `150` | ± roll range |
| `orient_min_cutoff`, `orient_beta` | `1.0`, `0.5` | One Euro filter on hand angles |
| `max_angular_speed_deg` | `180` | turn-rate limit, °/s |
| `tilt_correction` | `true` | depth corrected for hand tilt (section 7) |
| `ik_pitch_damping` | `0.1` | pitch-task damping |
| `ik_pitch_step` | `0.1` | max pitch correction per iteration (rad) |
| `ik_pitch_search_steps` | `6` | bisection steps when pitch gives way |

Angles are in **degrees in the file** (easier to read and tune) and radians
in the code (MuJoCo's unit).

---

## 12. Decisions made on your behalf

| # | Decision | Chosen | Why | Change it by |
|---|---|---|---|---|
| 1 | Which turns to control | **pitch + roll**; side bend ignored | the 5-joint arm has no free yaw (section 2) | needs a 6-DOF arm |
| 2 | Absolute or relative | **relative** to the calibration pose | absolute "claw points where fingers point" would aim the claw straight up at calibration, and the wrist's comfortable range is only about ±60° | — |
| 3 | Direction mapping | "lay your hand forward": flex toward camera = pitch down; twist = same-direction roll | one mental model covers both axes, and it's what the hand would do if it *were* the claw | `pitch_gain`, `roll_gain` (negative reverses) |
| 4 | Gains | 1:1 | easiest to learn | `pitch_gain`, `roll_gain` |
| 5 | Neutral pitch | −35° | most reachable band (6.4) | `neutral_pitch_deg` |
| 6 | Pitch range | −90° … +30° | straight-down grasps allowed; steep upward is rarely reachable | `pitch_min_deg`, `pitch_max_deg` |
| 7 | Roll range | ±150° | joint stops at about ±157° | `roll_limit_deg` |
| 8 | Hand measurement frame | arm space (right-handed, physical) | rotations transfer to the arm without sign surprises | — |
| 9 | Hand frame landmarks | palm only (wrist + 3 knuckles) | the claw gesture and fists don't disturb orientation | `handFrame()` |
| 10 | Left/right hand handling | front sign fixed at calibration | works for either hand and for the back of the hand, without trusting the model's handedness label | — |
| 11 | Angle decomposition | swing-twist about the fingers | twist stays "doorknob" even when flexed; Euler angles mix axes | `handAngles()` |
| 12 | Points used | world 3D when available, else image | world points are metric and not foreshortened | — |
| 13 | IK priority | position **strictly** first, enforced by a check | a claw at the right spot at a slightly wrong angle beats the reverse | — |
| 14 | Unreachable pitch | give way to the nearest reachable pitch (bisection), show `LIMITED` | always does something sensible; you can see why | `ik_pitch_search_steps` |
| 15 | Joint limits in IK | active set (park and re-solve) | clamping after the step broke the priority (6.2) | — |
| 16 | IK start point | warm start from the last solution | 44.7 mm → 2.0 mm tracking error (6.5) | — |
| 17 | Roll in IK | set directly, not solved | its axis *is* the pointing axis; no search needed | — |
| 18 | Pitch measure | angle in the arm's vertical plane, heading from `shoulder_lift` | defined everywhere, including straight down | `pitchOf()` |
| 19 | Smoothing | One Euro 1 Hz / β 0.5 + 180°/s rate limit | steadier than the gripper filter, because angles are noisier | `orient_*`, `max_angular_speed_deg` |
| 20 | Tilt and depth | corrected using world points, on by default | otherwise a 40° flex moves the arm about 5 cm (section 7) | `tilt_correction` |
| 21 | Hand lost | hold orientation | same rule as position and grip | — |
| 22 | Recalibration | also resets neutral orientation | one gesture sets everything | — |
| 23 | Config units | degrees in `arm.ini` | people think in degrees | — |
| 24 | Interface | `WristTarget` with optional pitch; old `solveIk` kept | "no request" ≠ "level"; old callers keep working | — |
| 25 | Default | orientation **on** | it's the Dash 4 feature; `orientation = false` = Dash 3 | `orientation` |
| 26 | Applying the solver fixes to Dash 2/3 | yes | never worse, fixes a drift (6.6) | — |
| 27 | Branch / commits | `feature/claw-orientation`, authored as you, no Claude line | as before | — |

---

## 13. Alternatives considered and rejected

| Alternative | Why not |
|---|---|
| **Full rotation mapping, then drop yaw** (apply the hand's whole 3D rotation to the claw and discard what the arm can't do) | At any non-level pitch, dropping world yaw leaks part of a side bend into roll: leaning your fingers sideways would twist the claw. Separate angles keep each hand motion on one claw motion. |
| **Absolute mimic** (claw points exactly where your fingers point) | The calibration pose has fingers up, so the claw would start pointing at the ceiling. Reaching "straight down" would need the fingers to point at the floor, which is uncomfortable. |
| **Side bend → pan** (use the base to yaw) | The pan also moves the claw's *position* sideways, so the claw would leave its target. Position priority forbids that. |
| **Weighted IK** (position and pitch in one least-squares solve) | When both can't be met it compromises on *both*, so the claw misses its target. Strict priority was preferred. |
| **Analytic IK** (closed-form trigonometry for the planar chain) | Fast and exact in principle, but the SO-101 model has small link offsets (≈18 mm sideways, the fingertip 12 mm off the roll axis). These make a clean formula fiddly and error-prone. The numerical solver handles them automatically and costs only microseconds. Worth revisiting for a real arm. |
| **Precomputing the reachable pitch range** per position (lookup table) | Accurate only on its grid, and has to be rebuilt whenever the workspace or model changes. The run-time check needs no maintenance. |
| **Using `computeOrientation()`'s Euler angles** (already in the engine) | They work in raw image axes (y down, mirrored), and Euler angles mix axes when combined (section 4.3). It's left unchanged for the desktop orientation mode. |
| **Orientation off by default** | Hides the feature. `orientation = false` is one line. |

---

## 14. Known limitations and open problems

1. **Not live-tested.** *(Do first.)* The angles, signs and gains are
   verified on synthetic hands and in the simulator, but not on a real
   webcam.
2. **Assumed sign of the model's depth axis.** The conversion assumes
   MediaPipe's z grows *away* from the camera (as documented for image
   points; world points are assumed the same). If flex works backwards
   live, set `pitch_gain = -1`. If twist feels mirrored, set `roll_gain = -1`.
3. **Mirrored camera assumed.** Like Dash 2, the mapping assumes
   `mirror = true`. With an unmirrored camera, left/right and some rotation
   directions reverse.
4. **Twisting edge-on hurts tracking.** At about 90° of twist the hand is
   edge-on to the camera. Fingers hide each other, landmark accuracy drops,
   and so do the claw measurements (Dash 3) and the palm normal. Expect a
   comfortable roll of about ±60–70° by hand. Raise `roll_gain` if you need
   more claw roll from less hand twist.
5. **Pitch gives way near the edges.** By the arm's geometry, straight down
   is reachable at only 14% of the workspace, and the claw quietly
   (`LIMITED`) settles for less elsewhere. Near the line between reachable
   and not, the given-way pitch can change slightly frame to frame.
6. **Yaw is impossible** on this arm (section 2). Side bend is shown on
   screen but does nothing.
7. **Flexing still moves the arm a little:** 5.3 mm for 30°, from the palm
   centre swinging. Anchoring position on the wrist alone would remove it,
   but would change Dash 2's feel; left for live testing.
8. **Calibrate facing the camera.** The front sign is decided at
   calibration; a palm held edge-on to the camera makes it ambiguous.
9. **Position-only accuracy is about 1 mm, not 0.1 mm.** Mean 0.76 mm at
   10 iterations, the same for Dash 3 and Dash 4 solvers (6.6). The 0.1 mm
   in Dash 2 came from its Python prototype. Lower damping or more
   iterations would tighten it.
10. **No gripper speed limit or force** (unchanged from Dash 3).

---

## 15. New tools / libraries

None in the product. **Prototyping only:** Python 3.11 with the `mujoco`
package (4.0.1, in a throwaway virtual environment, not part of the project)
probed the joint axes and tested IK strategies before the C++. A separate
throwaway build compared the Dash 3 and Dash 4 solvers (section 6.6).

---

## 16. Git and next steps

- **Before Dash 4:** Dash 3 (`feature/live-claw`) was merged into `main` and
  pushed to GitHub (merge commit `93a2222`).
- **Dash 4:** branch **`feature/claw-orientation`**, from `main` after that
  merge, in three commits: `4e68baf` (code and tests), `ea0cc85` (this doc
  and the other docs), and a final doc update. It is then **merged into
  `main` and pushed**.
- Every commit is authored as **Jacob**, with no `Co-Authored-By` line.
- Your own note at the end of `FUTURE_FEATURES.md` ("Add physics engine and
  interactable objects in the simulation") is included unchanged in the docs
  commit.
- **The GitHub repository has moved** to `Jzhen7/Gesture-Control-Engine`.
  Pushes still work through GitHub's redirect. The local `origin` URL hasn't
  been changed; `git remote set-url origin
  https://github.com/Jzhen7/Gesture-Control-Engine.git` would update it.

**Next (live test checklist):**
1. Calibrate (open palm, facing the camera, fingers up, 2 s), then watch
   `flex / twist` on the camera panel and `pitch / roll` in the 3D view.
2. **If tipping works backwards, set `pitch_gain = -1`; if twisting feels
   mirrored, set `roll_gain = -1`.** The direction of the model's depth axis
   is the one sign that couldn't be confirmed without a camera (14.2).
3. Expect about **±60–70° of comfortable roll**: past that the hand is
   edge-on to the camera and tracking degrades. Raise `roll_gain` for more
   claw roll from less hand twist (14.4).
4. Tipping the hand 30° still moves the arm about **5 mm** (palm centre
   swinging). Removing it (wrist-anchored position) would change how
   position feels, so judge it in person (14.7).
5. Tune `neutral_pitch_deg` and the gains for comfort.
