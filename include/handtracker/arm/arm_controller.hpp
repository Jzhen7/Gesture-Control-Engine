// Hand -> arm "mimic" controller (robot-arm milestone 1).
//
// Per frame:  hand landmarks -> gesture (smoothed) -> 3D hand position
//             -> workspace target -> One Euro filter -> speed limit -> backend
//             and, if the fingers are straight (claw pose), claw openness
//             (fingertip spread, 0..1) -> One Euro filter -> gripper opening
//             (or, in binary mode, fist? -> closed : open)
//
// Lifecycle:  Calibrating (hold an open palm still) -> Tracking.
// If the hand disappears the arm simply keeps its last target and gripper.
#pragma once

#include <array>
#include <functional>
#include <optional>
#include <string>

#include "handtracker/arm/arm_backend.hpp"
#include "handtracker/arm/workspace.hpp"
#include "handtracker/core/config.hpp"
#include "handtracker/core/filters.hpp"
#include "handtracker/core/gesture.hpp"
#include "handtracker/core/hand_space.hpp"

namespace ht::arm {

enum class Phase { Calibrating, Tracking };

struct ArmStatus {
  Phase phase = Phase::Calibrating;
  float calibrationProgress = 0.f;
  bool handVisible = false;
  Gesture gesture = Gesture::None;
  Vec3 handPosition;  ///< Normalised hand cube, before clamping.
  Vec3 target;        ///< Filtered target sent to the arm (metres).
  bool clawPose = false;       ///< Fingers straight enough to steer the gripper this frame.
  float handOpenness = 0.f;    ///< Raw claw openness this frame (0 = tips together, 1 = open).
  float gripperOpening = 1.f;  ///< Opening sent to the arm (0 = closed, 1 = open).
  HandAngles handAngles;       ///< Dash 4: hand rotation since calibration (raw, radians).
  WristTarget wrist;           ///< Dash 4: claw pitch/roll sent to the arm (filtered, limited).
  std::optional<HandSpaceCalibration> calibration;
};

class ArmController {
 public:
  using MessageSink = std::function<void(const std::string&)>;

  ArmController(const AppConfig& cfg, ArmBackend& arm, MessageSink sink = {});

  void update(const std::optional<HandLandmarks>& hand, double tMs);
  /// Throw away the calibration and ask for a new one.
  void recalibrate();
  /// Skip the calibration step (used by tests and --image style tools).
  void setCalibration(const HandSpaceCalibration& cal);

  const ArmStatus& status() const { return status_; }
  const WorkspaceBox& workspace() const { return box_; }

 private:
  void track(const HandLandmarks& hand, Gesture g, double tMs);
  void grip(const HandLandmarks& hand, Gesture g, double tSeconds);
  void orient(const HandLandmarks& hand, double tSeconds);
  /// The wrist target for the calibration pose (neutral pitch, no roll).
  WristTarget neutralWrist() const;

  AppConfig cfg_;
  ArmBackend& arm_;
  MessageSink sink_;
  WorkspaceBox box_;
  GestureSmoother smoother_;
  HandCalibrator calibrator_;
  std::array<OneEuroFilter, 3> filters_;
  OneEuroFilter gripFilter_;
  OneEuroFilter flexFilter_;
  OneEuroFilter twistFilter_;
  bool hasWrist_ = false;
  double lastWristT_ = 0.0;
  bool hasTarget_ = false;
  double lastT_ = 0.0;
  ArmStatus status_;
};

}  // namespace ht::arm
