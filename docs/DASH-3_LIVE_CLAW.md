# Dash 3: live claw control

> **What this doc is:** how proportional ("live") gripper control works, and
> the decisions made building it. It builds on [Dash 2](DASH-2_ROBOT_ARM.md)
> (the arm itself) and [Dash 1](DASH-1_CPP_REVAMP.md) (C++ basics).
> Planned work is in [FUTURE_FEATURES.md](FUTURE_FEATURES.md).

**Status:** implemented and tested with synthetic hands and in the MuJoCo
simulation (82/82 unit tests, 9/9 simulation tests on Windows 11, MSVC 2022).
**Not yet tried live with a webcam.** The default thresholds still need tuning
on real hands (section 11).

**What changed from Dash 2:**

| | Dash 2 (MVP) | Dash 3 |
|---|---|---|
| Gripper states | open **or** closed | **any opening** from 0% to 100% |
| Close it by | making a **fist** | **straight fingers**, all five fingertips **touching each other** in front of the palm |
| Open it by | any other hand shape | spreading the fingertips apart |
| A fist | closes | **does nothing**: the gripper holds where it is |
| Hand lost | holds position and grip | unchanged |

The old rule is still there: set `gripper_mode = binary` in `config/arm.ini`.

---

## 1. The claw gesture

```text
   open (100%)            half (50%)             closed (0%)

   |  |  |  |               \ \ | /                  \\|//
   |  |  |  |  /             \ \|/   /                \|/  <- all five tips
   |  |  |  | /               \ |   /                  V      touch, in front
   +--------+/                 +----+                 +--+    of the palm
     palm                       palm                   palm
```

- **Every finger stays straight.** Fingers bend only at the base knuckle,
  swinging forward so the tips meet.
- **Closed** = all five fingertips touching one another, in front of the palm.
- **Open** = the open palm you calibrated with.
- Anything in between gives a gripper opening in between.

---

## 2. Data flow

Dash 2's pipeline is unchanged up to the gripper step. Step 6 is new:

| # | Step | Output | Code |
|---|---|---|---|
| 1–5 | camera → landmarks → gesture → 3D position → arm target | target in metres | Dash 2 |
| 6a | **Claw pose?** Is *every* finger straight? | yes / no | `isClawPose()` |
| 6b | **Fingertip spread**: how bunched the five tips are | number, scale-free | `fingertipSpread()` |
| 6c | **Openness**: spread mapped between "closed" and your calibrated palm, with snap zones at each end | 0…1 | `handOpenness()` |
| 6d | **One Euro filter**, only while in the claw pose | smoothed 0…1 | `ArmController::grip()` |
| 7 | **Gripper servo angle** = closed angle + opening × (open − closed) | radians | `MujocoArm::advance()` |

If step 6a says "no" (a curled finger, a fist, a relaxed hand), steps 6b–7 are
skipped and the gripper keeps its last opening.

---

## 3. Measuring "how open": fingertip spread

```text
spread = average distance between every pair of the five fingertips (10 pairs)
         ÷ palm length (wrist → middle knuckle)
```

| Hand | Spread (synthetic hand in the tests) |
|---|---|
| open palm | 0.84 |
| claw, all tips touching | 0.14 |
| fist (for comparison; the claw check rejects it anyway) | 0.32 |

**Why all pairs, not just thumb ↔ fingers?** The first version measured thumb
tip → each fingertip, the way a two-jaw gripper works. Testing showed a flaw:
in a flat hand the four fingertips already sit close together, so an **"OK"
sign** (thumb touching only the index) read as **0% open, fully closed**.
Measuring all pairs makes "closed" mean what you specified: *all* tips
touching each other. The OK sign now reads about 18% open.

**Why divide by palm length?** The same hand looks bigger near the camera.
Dividing by a length on the same hand cancels that out, so moving toward the
camera (which steers the arm forward) doesn't change the grip. Palm length is
the same wrist → middle-knuckle distance Dash 2 uses for depth. It doesn't
change when the fingers move.

**Turning spread into 0…1:**

```text
raw      = (spread − closed_spread) / (your calibrated open spread − closed_spread)
openness = clamp((raw − deadband) / (1 − 2·deadband), 0, 1)
```

- `closed_spread` (default 0.25) is "tips touching". It isn't 0, because
  fingertip landmarks sit at the centre of each fingertip and fingers have
  thickness.
- "Fully open" comes from **your own calibration palm**, which is already
  recorded during the 2-second open-palm calibration. Big hands, small hands
  and different finger spreads all get the full 0–100% range.
- The **dead band** (default 10% at each end) snaps "nearly closed" to exactly
  closed and "nearly open" to exactly open. Without it, landmark noise would
  leave the jaws a few percent open when you mean "shut".

---

## 4. Is it a claw? Finger straightness

The spread alone can't tell a claw from a fist, because a fist also bunches the
fingertips (0.32). So each frame also checks that the fingers are straight:

```text
straightness (per finger) = |knuckle → tip|  ÷  (|knuckle → middle joint|
                                                 + |middle joint → end joint|
                                                 + |end joint → tip|)
```

That is the straight-line distance divided by the distance along the bones.

| Finger | Straightness |
|---|---|
| straight | 1.0 |
| straight, swung forward at the knuckle (the claw) | **still 1.0**. This is why it measures from the knuckle and not the wrist. |
| bent 25° at each joint | 0.94 |
| bent 60° at each joint | 0.68 |
| curled into a fist | 0.23–0.29 |

**The least straight finger decides.** The hand is a claw only when *every*
one of the four fingers reaches `grip_min_straightness` (default 0.8). The first
version averaged the four, and a test showed one fully curled pinky only
pulled the average down to 0.82, just under the threshold at the time. Taking
the minimum makes that case 0.29: an unambiguous "no".

**What happens when it isn't a claw:** the gripper **holds** its last opening.
This follows Dash 2's rule that an ambiguous hand shape changes nothing. In
practice:
- You can close on an object with the claw, then relax your hand into a fist,
  and the gripper stays shut.
- A fist can't close the gripper by accident.

**Why the thumb isn't checked for straightness:** a thumb meeting the
fingertips naturally bends at its end joint, and how much varies between
people. The thumb still counts in the spread (section 3), so it must actually
meet the other tips to close the gripper.

**It works in 2D too:** a straight finger projects to a straight line in a
camera image, even when it points toward the camera. Straightness is therefore
reliable on image points as well as on the 3D points.

---

## 5. Imperfect gestures: what happens, and what guards against it

Every row below except the last group is pinned by a unit test, so the
behaviour can't change silently.

| Situation | What happens | Guard |
|---|---|---|
| A **fist** | Gripper holds | claw check (straightness 0.23) |
| **One finger curled**, the rest a perfect claw | Gripper holds | claw check on the *least* straight finger (0.29) |
| **Two fingers half bent** (60° per joint) | Gripper holds | claw check (0.68 < 0.8) |
| **One finger slightly bent** (25° per joint) | Still counts; gripper follows | 0.94 passes. Small natural bends are tolerated. |
| **Fingers out of sync**: index + middle on the thumb, ring + pinky still open | In-between opening (about 18%), **not** closed | all-pairs spread |
| **"OK" sign**: thumb on the index tip only | About 18% open, **not** closed | all-pairs spread |
| **Four fingers together, thumb out** ("karate chop") | Mostly open (> 50%) | thumb is part of the spread |
| Hand **closer / further**, **bigger / smaller**, **rotated** in the image, **left or right hand** | Same openness (within 0.1%) | everything is a ratio of lengths on the same hand |
| Claw **pointing at the camera** | Uses the model's 3D world points instead of the flat image | `hand.world` preferred when present |
| **Fingertip flutter** frame to frame | Smoothed out | One Euro filter, dead band at each end |
| **Hand lost** or tracking confidence < 0.8 | Arm and gripper hold | the vision layer drops low-confidence hands (`landmark_score`) |
| **Making or opening the claw** | Arm position doesn't move (< 0.1 mm) | position uses wrist + middle knuckle only |

**Not guarded yet.** These are the gaps, also listed in section 11:

| Situation | What happens today |
|---|---|
| **Fingers hidden** (hand edge-on, fingers behind the palm, claw pointing straight at the lens) | The model always returns all 21 points, guessing hidden ones, and this model gives no per-point visibility score. If the guess makes a finger look bent, the claw check holds the gripper (safe). If the guess looks like a plausible claw, the gripper follows the guess. |
| **Palm facing away** from the camera | Not detected. The measurements are mirror-symmetric, so they still work in principle, but the landmark model is less accurate from behind. |
| **A sudden bad frame** that still passes the claw check | The filter softens it but doesn't reject it. The jaws can twitch toward the bad value for a frame or two. |
| **Hovering at the straightness threshold** | The gripper alternates between following and holding. Nothing jumps, but it can feel sticky. |

---

## 6. 3D points when available

The hand model outputs two sets of 21 points: **image points** (pixels) and
**world points** (metres, the model's own 3D estimate). Both measurements use
the world points when they exist, the same way `computeOrientation()` already
does. This matters for the claw because it often points at the camera, which
makes finger lengths look shorter in the image. The synthetic test hands have
no world points, so the tests use the image-point path (plus one test that
checks world points take priority).

---

## 7. Smoothing and responsiveness

| Mechanism | Why | Setting |
|---|---|---|
| One Euro filter on openness | Stops the jaws twitching, but lets a fast grab through quickly | `grip_min_cutoff` (2.0 Hz), `grip_beta` (2.0) |
| Dead band at both ends | Fully closed really is 0, fully open really is 1 | `grip_deadband` (0.1) |
| Hold when not a claw | Curled or transitional hands don't move the jaws | `grip_min_straightness` (0.8) |
| Hold on hand loss | Same as Dash 2 | always on |

**Measured lag** (from the tests):

| Stage | Time from "open" to "10% open" |
|---|---|
| Hand → command (filter) | **2 frames ≈ 66 ms** at 30 fps |
| Command → simulated jaws | **275 ms** (MuJoCo position servo) |

So most of the delay is the servo physically moving, not the software.

---

## 8. Code changes

| File | Change |
|---|---|
| `core/hand_space.*` | **New:** `fingertipSpread()`, `fingerStraightness()`, `isClawPose()`, `handOpenness()`, `GripConfig`. Calibration also records `openSpread`. |
| `arm/arm_backend.hpp` | `setGripper(bool)` → **`setGripperOpening(float)`** (0 = closed, 1 = open). `ArmState::gripperClosed` → `gripperCommand` (0…1). |
| `arm/mujoco_arm.*` | Servo angle is interpolated between the closed and open angles. Measured `gripperOpening` now uses the same scale (1 = configured open angle). |
| `arm/arm_controller.*` | New `grip()` step: binary or proportional, claw check, One Euro filter. Status has `clawPose`, `handOpenness`, `gripperOpening`. |
| `core/config.*`, `config/arm.ini` | New keys (section 9) |
| `apps/arm_app.cpp`, `apps/arm_view.cpp`, `apps/main.cpp` | Overlay shows "claw 63% open -> gripper 63%" or "fingers bent: gripper held". Jaw colour blends red → green with the opening. Gripper text shows the commanded vs. measured %. Help text updated. |
| `tests/test_arm.cpp` | Synthetic **claw** builder (straight fingers swung to a meeting point) and a joint-by-joint `bendFinger()`. Tests for every row in section 5, plus the controller (follows, quick grab, fist holds, lag, no arm motion) and config. |
| `tests/test_arm_sim.cpp` | Real simulator: settles at 25/50/75% within 5%, open → shut < 0.5 s, clamping. |

**Why rename `setGripper` instead of adding an overload?** C++ silently turns
`true` into `1.0f`. With an overload, old code calling `setGripper(true)`
("close") would compile and mean "fully open". Renaming makes any old call a
compile error.

**Why the measured opening changed scale:** in Dash 2, "1" meant the joint's
mechanical maximum (1.75 rad), but "open" is 1.2 rad, so a fully open gripper
read 69%. Now commanded and measured values use the same 0…1 scale, so
"commanded 50%, measured 50%" means the jaws got there. When the jaws close on
an object, measured stays above commanded. That gap will be useful for grasp
detection later.

---

## 9. Settings (`config/arm.ini`, `[arm]`)

| Key | Default | Meaning |
|---|---|---|
| `gripper_mode` | `proportional` | `proportional` (Dash 3 claw) or `binary` (Dash 2 fist rule) |
| `grip_closed_spread` | `0.25` | Fingertip spread that counts as fully closed. Raise it if touching your fingertips together still leaves the jaws a bit open. |
| `grip_deadband` | `0.1` | Snap zone at each end (fraction of the range) |
| `grip_min_straightness` | `0.8` | How straight *every* finger must be to count as a claw (1 = perfectly straight). Lower it if the gripper "freezes" while you're using it. |
| `grip_min_cutoff` | `2.0` | One Euro filter: lower = steadier but laggier |
| `grip_beta` | `2.0` | One Euro filter: higher = less lag on fast grabs |

---

## 10. Decisions made on your behalf

| Decision | Chosen | Why | Change it by |
|---|---|---|---|
| Closed gesture | straight fingers, all tips touching in front of the palm | as you specified | — |
| Openness measure | mean distance over all 10 fingertip pairs ÷ palm length | thumb-only measure let an OK sign close the gripper (section 3) | `fingertipSpread()` |
| What counts as "100% open" | your calibration palm | already captured; adapts to your hand | recalibrate (`c`) |
| Claw check | *every* finger's straightness ≥ 0.8 | the average let one curled finger through (section 4) | `grip_min_straightness` |
| Thumb straightness | not checked | the thumb bends naturally when meeting the fingertips | `fingerStraightness()` |
| Not a claw (e.g. fist) | gripper **holds** | can't close by accident; lets you relax while holding something | code |
| Old fist rule | kept as `gripper_mode = binary` | for comparison, and as a fallback if the claw proves hard to track | `arm.ini` |
| Default mode | `proportional` | it's the Dash 3 feature | `arm.ini` |
| Filter | One Euro, 2 Hz / β 2 | 66 ms to react to a sudden grab while still damping flutter | `grip_min_cutoff`, `grip_beta` |
| Dead band | 10% each end | guarantees full close / full open despite noise | `grip_deadband` |
| Interface | renamed to `setGripperOpening(float)` | old bool calls can't silently invert (section 8) | — |
| Doc name | `docs/DASH-3_LIVE_CLAW.md` | follows Dash 1 / 2 | rename freely |
| Branch | `feature/live-claw`, committed locally under your git identity | for you to review before it's pushed or merged | — |

**Libraries / tools:** nothing new. It reuses OpenCV and MuJoCo 3.17.0 (Dash 2)
and the project's own One Euro filter. No Python prototype was needed: the
synthetic hand generator gained a claw pose and a joint-bending helper instead.

---

## 11. Known limitations and open problems

1. **Not tested live yet.** *(Address first.)* All numbers come from
   synthetic hands and the simulator. The 0.25 "closed" spread and the 0.8
   straightness threshold are educated guesses for real MediaPipe landmarks.
   They need a webcam session to confirm, using the on-screen "claw 63% open"
   readout.
2. **Hidden fingertips.** With the claw pointing straight at the lens, or the
   hand edge-on, fingertips hide each other and the model guesses their
   positions (section 5). Tilt your hand so the camera sees the claw at an
   angle. A possible fix: compare image and world finger lengths to detect
   heavy foreshortening, and hold the gripper when it's too extreme.
3. **Partial pinches aren't fully open.** An OK sign or half the fingers at
   the thumb read about 18% open, not 100%. They don't close the gripper, but
   they do move it. A stricter "only follow when the tips converge evenly"
   check is possible, at the cost of a less forgiving gesture.
4. **Threshold stickiness.** A finger hovering around straightness 0.8 makes
   the gripper alternate between following and holding. Fix: hysteresis
   (enter the claw at 0.8, leave it at 0.7).
5. **No outlier rejection.** One badly tracked frame that passes the claw
   check is smoothed, not discarded.
6. **Jumps after a pause.** After a fist or a lost hand, the gripper jumps
   straight to the claw's current opening when the claw returns. Unlike the
   arm's position, the gripper has no speed limit. The real servo will limit
   it physically, but a gripper speed limit is worth adding before real
   hardware.
7. **No grip force.** The jaws are a position servo. Squeezing an object
   harder means commanding a smaller opening, not more force. The simulator
   has no objects to grip yet (the pick-and-place scene is still a future
   feature).
8. **Two modes, two gestures.** Binary mode still closes on a fist. That's
   deliberate (it's the Dash 2 behaviour), but it means the two modes use
   different "close" gestures.
