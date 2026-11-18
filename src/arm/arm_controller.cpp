#include "handtracker/arm/arm_controller.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace ht::arm {

ArmController::ArmController(const AppConfig& cfg, ArmBackend& arm, MessageSink sink)
    : cfg_(cfg),
      arm_(arm),
      sink_(std::move(sink)),
      box_{cfg.arm.workspaceMin, cfg.arm.workspaceMax},
      smoother_(cfg.smoothingFrames),
      calibrator_(cfg.arm.calibrationMs, cfg.arm.calibrationMaxMotion, cfg.arm.hand.tiltCorrection),
      filters_{OneEuroFilter(cfg.arm.minCutoff, cfg.arm.beta),
               OneEuroFilter(cfg.arm.minCutoff, cfg.arm.beta),
               OneEuroFilter(cfg.arm.minCutoff, cfg.arm.beta)},
      gripFilter_(cfg.arm.gripMinCutoff, cfg.arm.gripBeta),
      flexFilter_(cfg.arm.orientMinCutoff, cfg.arm.orientBeta),
      twistFilter_(cfg.arm.orientMinCutoff, cfg.arm.orientBeta) {
  status_.target = box_.center();
  arm_.setTarget(status_.target);  // Start in the middle of the workspace, gripper open.
  arm_.setGripperOpening(1.f);
  status_.wrist = neutralWrist();
  arm_.setWristTarget(status_.wrist);
}

WristTarget ArmController::neutralWrist() const {
  WristTarget w;
  if (cfg_.arm.orientation) w.pitch = std::clamp(cfg_.arm.neutralPitch, cfg_.arm.pitchMin, cfg_.arm.pitchMax);
  return w;
}

void ArmController::recalibrate() {
  status_.phase = Phase::Calibrating;
  status_.calibration.reset();
  calibrator_.reset();
  if (sink_) sink_("calibration: hold an open palm still where it feels comfortable");
}

void ArmController::setCalibration(const HandSpaceCalibration& cal) {
  status_.calibration = cal;
  status_.phase = Phase::Tracking;
  // The calibration pose *is* the neutral orientation, so start from it.
  flexFilter_.reset();
  twistFilter_.reset();
  hasWrist_ = false;
  status_.handAngles = {};
  status_.wrist = neutralWrist();
  arm_.setWristTarget(status_.wrist);
}

void ArmController::update(const std::optional<HandLandmarks>& hand, double tMs) {
  status_.handVisible = hand.has_value();
  status_.gesture = hand ? smoother_.push(classifyGesture(*hand, cfg_.classifier)) : Gesture::None;
  if (!hand) smoother_.reset();

  if (status_.phase == Phase::Calibrating) {
    if (auto cal = calibrator_.update(hand, status_.gesture, tMs)) {
      setCalibration(*cal);
      if (sink_) sink_("calibrated: the arm now follows your hand");
    }
    status_.calibrationProgress = calibrator_.progress(tMs);
    return;
  }
  if (hand) track(*hand, status_.gesture, tMs);
  // Hand lost: deliberately do nothing, so the arm holds its last target and grip.
}

void ArmController::track(const HandLandmarks& hand, Gesture g, double tMs) {
  status_.handPosition = handPosition(hand, *status_.calibration, cfg_.arm.hand);
  const Vec3 raw = handToWorkspace(status_.handPosition, box_);

  // 1. Smooth out landmark jitter (One Euro: smooth when still, responsive when moving).
  const double t = tMs / 1000.0;
  Vec3 smooth{static_cast<float>(filters_[0].filter(raw.x, t)),
              static_cast<float>(filters_[1].filter(raw.y, t)),
              static_cast<float>(filters_[2].filter(raw.z, t))};

  // 2. Speed limit, so a tracking glitch can't fling the arm across the workspace.
  if (hasTarget_) {
    const double dt = std::max(0.0, t - lastT_);
    const float maxStep = cfg_.arm.maxSpeed * static_cast<float>(dt);
    const Vec3 step = smooth - status_.target;
    const float len = step.norm();
    if (len > maxStep && len > 0.f) smooth = status_.target + step * (maxStep / len);
  }
  hasTarget_ = true;
  lastT_ = t;
  status_.target = box_.clamp(smooth);
  arm_.setTarget(status_.target);

  grip(hand, g, t);
  orient(hand, t);
}

void ArmController::orient(const HandLandmarks& hand, double t) {
  if (!cfg_.arm.orientation) return;
  const auto& cal = *status_.calibration;
  status_.handAngles = handAngles(handFrame(hand, cal.frontSign), cal.frame);

  // 1. Smooth the two angles we use (side bend is measured but has nowhere to go:
  //    this arm can't yaw the claw independently of where it is).
  const double flex = flexFilter_.filter(status_.handAngles.flex, t);
  const double twist = twistFilter_.filter(status_.handAngles.twist, t);

  // 2. Map: tipping the fingers towards the camera tips the claw down; twisting
  //    the hand twists the claw the same way about its own pointing axis.
  WristTarget want;
  want.pitch = std::clamp(cfg_.arm.neutralPitch - cfg_.arm.pitchGain * static_cast<float>(flex),
                          cfg_.arm.pitchMin, cfg_.arm.pitchMax);
  want.roll = std::clamp(cfg_.arm.rollGain * static_cast<float>(twist), -cfg_.arm.rollLimit,
                         cfg_.arm.rollLimit);

  // 3. Turn-rate limit, like the position speed limit: a mis-tracked frame
  //    can't spin the wrist round in one go.
  if (hasWrist_ && status_.wrist.pitch) {
    const float maxStep = cfg_.arm.maxAngularSpeed * static_cast<float>(std::max(0.0, t - lastWristT_));
    want.pitch = *status_.wrist.pitch + std::clamp(*want.pitch - *status_.wrist.pitch, -maxStep, maxStep);
    want.roll = status_.wrist.roll + std::clamp(want.roll - status_.wrist.roll, -maxStep, maxStep);
  }
  hasWrist_ = true;
  lastWristT_ = t;
  status_.wrist = want;
  arm_.setWristTarget(want);
}

void ArmController::grip(const HandLandmarks& hand, Gesture g, double t) {
  status_.clawPose = isClawPose(hand, cfg_.arm.grip);
  status_.handOpenness = handOpenness(hand, *status_.calibration, cfg_.arm.grip);

  if (cfg_.arm.gripperMode == GripperMode::Binary) {
    // Dash 2 rule: closed only for a fist. An ambiguous frame (None) keeps the last state.
    if (g == Gesture::None) return;
    const bool closed = gripClosed(g);
    const bool wasClosed = status_.gripperOpening < 0.5f;
    if (closed != wasClosed && sink_) sink_(closed ? "gripper: close" : "gripper: open");
    status_.gripperOpening = closed ? 0.f : 1.f;
  } else {
    // Dash 3: the jaws follow the claw continuously. Curled fingers (a fist,
    // a relaxed hand) aren't the claw, so the gripper holds where it is.
    if (!status_.clawPose) return;
    // The filter removes the flutter of individual fingertips but lets a quick grab through.
    const double f = gripFilter_.filter(status_.handOpenness, t);
    status_.gripperOpening = std::clamp(static_cast<float>(f), 0.f, 1.f);
  }
  arm_.setGripperOpening(status_.gripperOpening);
}

}  // namespace ht::arm
