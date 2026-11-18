// Robot-arm front-end logic, tested against a fake arm (no simulator needed).
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "hand_builder.hpp"
#include "handtracker/arm/arm_controller.hpp"
#include "handtracker/arm/workspace.hpp"
#include "handtracker/core/config.hpp"
#include "handtracker/core/hand_space.hpp"
#include "test_framework.hpp"

using namespace ht;
using namespace ht::arm;
using namespace testutil;

namespace {

constexpr double kFrameMs = 33.0;

class FakeArm final : public ArmBackend {
 public:
  std::vector<Vec3> targets;
  std::vector<float> grips;  ///< Every opening sent, 0 = closed, 1 = open.
  void setTarget(const Vec3& p) override { targets.push_back(p); }
  void setGripperOpening(float opening) override { grips.push_back(opening); }
  std::vector<WristTarget> wrists;
  void setWristTarget(const WristTarget& w) override { wrists.push_back(w); }
  void advance(double) override {}
  ArmState state() const override { return {}; }
  std::string name() const override { return "fake"; }
};

HandSpaceCalibration calibrationFor(const HandLandmarks& hand) {
  HandSpaceCalibration c;
  const Vec3 p = palmCenterNormalized(hand);
  c.centerX = p.x;
  c.centerY = p.y;
  c.palmSize = palmSizeNormalized(hand);
  c.openSpread = fingertipSpread(hand);
  c.frontSign = frontSign(hand);
  c.frame = handFrame(hand, c.frontSign);
  return c;
}

constexpr float kDeg = 3.14159265f / 180.f;

/// Rotates v about unit axis a by angle (Rodrigues).
Vec3 rotateVec(const Vec3& v, const Vec3& a, float angle) {
  const float c = std::cos(angle), s = std::sin(angle);
  return v * c + cross(a, v) * s + a * (dot(a, v) * (1.f - c));
}

/// Turns a synthetic hand in 3D about its wrist. `axis` is in arm space
/// (X towards the camera, Y the user's left, Z up). Both the image and the
/// world points are rotated, so it exercises the 3D path like the real model.
HandLandmarks rotateHand(HandLandmarks h, const Vec3& axis, float degrees) {
  const Vec3 wrist = h[LandmarkId::Wrist];
  for (auto& p : h.image) {
    const Vec3 a = rotateVec(toArmSpace(p - wrist), normalized(axis), degrees * kDeg);
    p = wrist + Vec3{-a.y, -a.z, -a.x};  // arm space -> landmark axes (inverse of toArmSpace)
  }
  HandLandmarks::Points world = h.image;
  for (auto& p : world) p = (p - wrist) * 0.001f;  // "metres", wrist at the origin
  h.world = world;
  return h;
}

const Vec3 kForward{1, 0, 0}, kLeft{0, 1, 0}, kUp{0, 0, 1};

/// The "claw": every finger straight, swung at its base knuckle so all five
/// fingertips meet at one point in front of the palm.
/// t = 0 tips touching (closed), t = 1 open palm. Each finger stays a straight
/// line throughout. Tips are ~0.15 palm-lengths apart when "touching" (fingers
/// have thickness).
HandLandmarks claw(float t, const Pose& pose = {}) {
  const auto palm = makeHand(kPalm, pose);
  HandLandmarks h = palm;
  const Vec3 wrist = palm[LandmarkId::Wrist];
  const Vec3 knuckle = palm[LandmarkId::MiddleMcp];
  // Seen from the front, the tips of a claw meet just beyond the knuckles
  // (the fingers swing towards the camera).
  const Vec3 meet = wrist + (knuckle - wrist) * 1.25f;
  const Vec3 across = normalized(palm[LandmarkId::IndexMcp] - palm[LandmarkId::PinkyMcp]);
  const float gap = 0.15f * pose.scale;
  // Base joint of each digit and how far along it its later joints sit.
  struct Digit { std::size_t base; float offset; };
  const Digit digits[] = {{2, 0.f}, {5, 1.f}, {9, 0.5f}, {13, -0.5f}, {17, -1.f}};
  for (const Digit& d : digits) {
    const std::size_t tip = d.base == 2 ? 4 : d.base + 3;
    const Vec3 base = palm.image[d.base];
    const Vec3 closedTip = meet + across * (d.offset * gap);
    const float openLen = distance(palm.image[tip], base);
    for (std::size_t j = d.base + 1; j <= tip; ++j) {
      const float frac = distance(palm.image[j], base) / openLen;  // keep joints in proportion
      const Vec3 closed = base + (closedTip - base) * frac;
      h.image[j] = closed * (1.f - t) + palm.image[j] * t;
    }
  }
  return h;
}

HandLandmarks partlyOpen(float t, const Pose& pose = {}) { return claw(t, pose); }

AppConfig binaryConfig() {
  AppConfig cfg = defaultConfig();
  cfg.arm.gripperMode = GripperMode::Binary;
  return cfg;
}

}  // namespace

TEST_CASE("hand space: calibration pose is the origin") {
  const auto hand = makeHand(kPalm);
  const Vec3 p = handPosition(hand, calibrationFor(hand), {});
  CHECK_NEAR(p.x, 0, 1e-5);
  CHECK_NEAR(p.y, 0, 1e-5);
  CHECK_NEAR(p.z, 0, 1e-5);
}

TEST_CASE("hand space: right, up and closer map to +x, +y, +z") {
  const auto cal = calibrationFor(makeHand(kPalm));
  Pose right;
  right.x += 96;  // 0.15 of a 640 px frame
  CHECK_NEAR(handPosition(makeHand(kPalm, right), cal, {}).x, 0.5, 1e-3);
  Pose up;
  up.y -= 60;  // image y points down
  CHECK(handPosition(makeHand(kPalm, up), cal, {}).y > 0.4f);
  Pose closer;
  closer.scale *= 1.2f;  // bigger palm = nearer the camera
  const float z = handPosition(makeHand(kPalm, closer), cal, {}).z;
  CHECK_NEAR(z, std::log(1.2) / 0.35, 1e-3);
}

TEST_CASE("hand space: making a fist does not change depth") {
  const auto cal = calibrationFor(makeHand(kPalm));
  CHECK_NEAR(handPosition(makeHand(kFist), cal, {}).z, 0, 1e-5);
}

TEST_CASE("hand space: only a fist closes the gripper") {
  CHECK(gripClosed(Gesture::Fist));
  for (Gesture g : {Gesture::Palm, Gesture::Peace, Gesture::One, Gesture::Three, Gesture::None}) {
    CHECK(!gripClosed(g));
  }
}

TEST_CASE("openness: fingertip spread separates an open palm from a closed claw") {
  const float open = fingertipSpread(makeHand(kPalm));
  const float closed = fingertipSpread(claw(0.f));
  std::cerr << "  spread: open palm " << open << ", closed claw " << closed
            << ", fist " << fingertipSpread(makeHand(kFist)) << "\n";
  CHECK(open > 0.6f);
  CHECK(closed < 0.2f);
}

TEST_CASE("openness: 0 for a closed claw, 1 for the calibration palm, in between otherwise") {
  const auto cal = calibrationFor(makeHand(kPalm));
  const GripConfig cfg;
  CHECK_NEAR(handOpenness(claw(0.f), cal, cfg), 0, 1e-6);
  CHECK_NEAR(handOpenness(makeHand(kPalm), cal, cfg), 1, 1e-6);
  const float half = handOpenness(partlyOpen(0.5f), cal, cfg);
  CHECK(half > 0.3f);
  CHECK(half < 0.7f);
}

TEST_CASE("openness: rises steadily as the hand opens") {
  const auto cal = calibrationFor(makeHand(kPalm));
  float prev = -1.f;
  for (int i = 0; i <= 10; ++i) {
    const float o = handOpenness(partlyOpen(static_cast<float>(i) / 10.f), cal, {});
    CHECK(o >= prev);
    prev = o;
  }
}

TEST_CASE("openness: unaffected by hand size, distance or rotation") {
  const auto cal = calibrationFor(makeHand(kPalm));
  const float ref = handOpenness(partlyOpen(0.6f), cal, {});
  Pose p;
  p.scale = 160.f;
  p.rotationDeg = 35.f;
  p.x = 200.f;
  CHECK_NEAR(handOpenness(partlyOpen(0.6f, p), cal, {}), ref, 1e-3);
  p.mirrored = true;
  CHECK_NEAR(handOpenness(partlyOpen(0.6f, p), cal, {}), ref, 1e-3);
}

TEST_CASE("openness: prefers metric world landmarks when available") {
  // The image says "closed" (fingers foreshortened towards the camera), the 3D
  // world points say "open": the world points win.
  auto hand = claw(0.f);
  hand.world = makeHand(kPalm).image;
  CHECK_NEAR(fingertipSpread(hand), fingertipSpread(makeHand(kPalm)), 1e-5);
}

TEST_CASE("claw pose: straight fingers count, curled fingers don't") {
  std::cerr << "  straightness: palm " << fingerStraightness(makeHand(kPalm)) << ", closed claw "
            << fingerStraightness(claw(0.f)) << ", fist " << fingerStraightness(makeHand(kFist))
            << "\n";
  const GripConfig cfg;
  CHECK(isClawPose(makeHand(kPalm), cfg));
  CHECK(isClawPose(claw(0.f), cfg));
  CHECK(isClawPose(claw(0.5f), cfg));
  CHECK(!isClawPose(makeHand(kFist), cfg));
  CHECK(!isClawPose(makeHand(kOne), cfg));  // thumb up, fingers curled
  Pose p;
  p.rotationDeg = 60.f;
  p.scale = 70.f;
  CHECK(isClawPose(claw(0.f, p), cfg));
  CHECK(!isClawPose(makeHand(kFist, p), cfg));
}

/// Copies one finger's three outer joints (middle joint, end joint, tip) from `from`.
void copyFinger(HandLandmarks& to, const HandLandmarks& from, LandmarkId knuckle) {
  const auto k = static_cast<std::size_t>(knuckle);
  for (std::size_t j = k + 1; j <= k + 3; ++j) to.image[j] = from.image[j];
}

/// Bends one finger at its middle and end joints by `degrees` each (in the
/// image plane), like a real finger curling. ~90-100 degrees each = curled into a fist.
void bendFinger(HandLandmarks& h, LandmarkId knuckle, float degrees) {
  const auto k = static_cast<std::size_t>(knuckle);
  const float a = degrees * 3.14159265f / 180.f;
  auto rotate = [&](std::size_t pivot, std::size_t from) {
    const Vec3 c = h.image[pivot];
    for (std::size_t j = from; j <= k + 3; ++j) {
      const Vec3 d = h.image[j] - c;
      h.image[j] = {c.x + d.x * std::cos(a) - d.y * std::sin(a),
                    c.y + d.x * std::sin(a) + d.y * std::cos(a), h.image[j].z};
    }
  };
  rotate(k + 1, k + 2);  // bend at the middle joint: end joint and tip swing
  rotate(k + 2, k + 3);  // bend at the end joint: tip swings further
}

TEST_CASE("imperfect claw: one finger fully curled is not a claw (gripper holds)") {
  auto h = claw(0.f);
  bendFinger(h, LandmarkId::PinkyMcp, 100.f);
  std::cerr << "  pinky curled: straightness " << fingerStraightness(h) << "\n";
  CHECK(!isClawPose(h, {}));
}

TEST_CASE("imperfect claw: two fingers half bent is not a claw") {
  auto h = claw(0.f);
  bendFinger(h, LandmarkId::RingMcp, 60.f);
  bendFinger(h, LandmarkId::PinkyMcp, 60.f);
  std::cerr << "  ring + pinky bent 60 deg: straightness " << fingerStraightness(h) << "\n";
  CHECK(!isClawPose(h, {}));
}

TEST_CASE("imperfect claw: one finger slightly bent still counts") {
  auto h = claw(0.f);
  bendFinger(h, LandmarkId::RingMcp, 25.f);
  std::cerr << "  ring bent 25 deg per joint: straightness " << fingerStraightness(h) << "\n";
  CHECK(isClawPose(h, {}));
}

TEST_CASE("imperfect claw: fingers out of sync give an in-between opening") {
  // Index and middle tips on the thumb, ring and pinky still wide open.
  const auto cal = calibrationFor(makeHand(kPalm));
  auto h = claw(0.f);
  const auto open = claw(1.f);
  copyFinger(h, open, LandmarkId::RingMcp);
  copyFinger(h, open, LandmarkId::PinkyMcp);
  const float o = handOpenness(h, cal, {});
  std::cerr << "  half the fingers closed: openness " << o << "\n";
  CHECK(isClawPose(h, {}));
  CHECK(o > 0.15f);
  CHECK(o < 0.8f);
}

TEST_CASE("imperfect claw: thumb meets only the index (OK sign) is not fully closed") {
  const auto cal = calibrationFor(makeHand(kPalm));
  auto h = makeHand(kPalm);
  h[LandmarkId::ThumbTip] = h[LandmarkId::IndexTip];
  const float o = handOpenness(h, cal, {});
  std::cerr << "  thumb on index tip only: openness " << o << "\n";
  CHECK(o > 0.1f);
}

TEST_CASE("openness: fingers together but thumb out is not a closed claw") {
  // A flat "karate chop" hand: four fingers side by side, thumb away. Only the
  // thumb meeting the fingertips should close the gripper.
  const auto cal = calibrationFor(makeHand(kPalm));
  CHECK(handOpenness(makeHand(kPalm), cal, {}) > 0.9f);
  auto flat = makeHand(kPalm);
  for (LandmarkId tip : {LandmarkId::IndexTip, LandmarkId::RingTip, LandmarkId::PinkyTip}) {
    flat[tip].x = flat[LandmarkId::MiddleTip].x;
  }
  CHECK(handOpenness(flat, cal, {}) > 0.5f);
}

TEST_CASE("openness: dead band snaps near-closed and near-open to exactly 0 and 1") {
  const auto cal = calibrationFor(makeHand(kPalm));
  GripConfig cfg;
  cfg.deadband = 0.2f;
  CHECK_NEAR(handOpenness(partlyOpen(0.05f), cal, cfg), 0, 1e-6);
  CHECK_NEAR(handOpenness(partlyOpen(0.97f), cal, cfg), 1, 1e-6);
}

TEST_CASE("openness: a calibration palm no more open than a closed claw stays in range") {
  HandSpaceCalibration cal = calibrationFor(makeHand(kPalm));
  cal.openSpread = 0.3f;  // same as closedSpread
  const float o = handOpenness(makeHand(kPalm), cal, {});
  CHECK(o >= 0.f);
  CHECK(o <= 1.f);
}

TEST_CASE("calibrator: open palm held still for the duration") {
  HandCalibrator cal(2000.0);
  const auto hand = makeHand(kPalm);
  std::optional<HandSpaceCalibration> result;
  double t = 0;
  for (; t < 1900; t += kFrameMs) CHECK(!cal.update(hand, Gesture::Palm, t).has_value());
  for (; t < 2200 && !result; t += kFrameMs) result = cal.update(hand, Gesture::Palm, t);
  REQUIRE(result.has_value());
  CHECK_NEAR(result->palmSize, palmSizeNormalized(hand), 1e-5);
  CHECK_NEAR(result->centerX, palmCenterNormalized(hand).x, 1e-5);
  CHECK_NEAR(result->openSpread, fingertipSpread(hand), 1e-5);
}

TEST_CASE("calibrator: other gestures or moving restart it") {
  HandCalibrator cal(1000.0);
  double t = 0;
  for (int i = 0; i < 20; ++i, t += kFrameMs) cal.update(makeHand(kPalm), Gesture::Palm, t);
  CHECK(cal.progress(t) > 0.5f);
  cal.update(makeHand(kFist), Gesture::Fist, t);
  CHECK_NEAR(cal.progress(t), 0, 1e-6);

  for (int i = 0; i < 20; ++i, t += kFrameMs) cal.update(makeHand(kPalm), Gesture::Palm, t);
  Pose moved;
  moved.x += 100;
  cal.update(makeHand(kPalm, moved), Gesture::Palm, t);
  CHECK_NEAR(cal.progress(t), 0, 1e-6);
}

TEST_CASE("workspace: centre, axes and clamping") {
  const WorkspaceBox box{{0.17f, -0.13f, 0.06f}, {0.31f, 0.13f, 0.24f}};
  const Vec3 c = handToWorkspace({0, 0, 0}, box);
  CHECK_NEAR(c.x, 0.24, 1e-6);
  CHECK_NEAR(c.y, 0.0, 1e-6);
  CHECK_NEAR(c.z, 0.15, 1e-6);

  CHECK_NEAR(handToWorkspace({1, 0, 0}, box).y, -0.13, 1e-6);  // user's right = arm's -Y
  CHECK_NEAR(handToWorkspace({0, 1, 0}, box).z, 0.24, 1e-6);   // up
  CHECK_NEAR(handToWorkspace({0, 0, 1}, box).x, 0.31, 1e-6);   // forward
  const Vec3 far = handToWorkspace({5, -5, 5}, box);
  CHECK(box.contains(far));
}

TEST_CASE("arm controller: calibrates, then follows the hand") {
  AppConfig cfg = defaultConfig();
  FakeArm arm;
  ArmController ctl(cfg, arm);
  CHECK(ctl.status().phase == Phase::Calibrating);

  double t = 0;
  for (int i = 0; i < 80; ++i, t += kFrameMs) ctl.update(makeHand(kPalm), t);
  REQUIRE(ctl.status().phase == Phase::Tracking);

  Pose right;
  right.x += 150;
  for (int i = 0; i < 60; ++i, t += kFrameMs) ctl.update(makeHand(kPalm, right), t);
  CHECK(arm.targets.back().y < -0.1f);  // moved to the arm's right
  CHECK_NEAR(arm.grips.back(), 1, 1e-3);  // open palm = open gripper
}

TEST_CASE("arm controller (proportional): gripper follows claw openness") {
  AppConfig cfg = defaultConfig();
  FakeArm arm;
  ArmController ctl(cfg, arm);
  const auto cal = calibrationFor(makeHand(kPalm));
  ctl.setCalibration(cal);
  double t = 0;
  const float expectHalf = handOpenness(partlyOpen(0.5f), cal, cfg.arm.grip);
  for (int i = 0; i < 30; ++i, t += kFrameMs) ctl.update(partlyOpen(0.5f), t);
  CHECK_NEAR(arm.grips.back(), expectHalf, 0.02);
  for (int i = 0; i < 30; ++i, t += kFrameMs) ctl.update(claw(0.f), t);
  CHECK(arm.grips.back() < 0.02f);
  for (int i = 0; i < 30; ++i, t += kFrameMs) ctl.update(makeHand(kPalm), t);
  CHECK(arm.grips.back() > 0.98f);
}

TEST_CASE("arm controller (proportional): a fist doesn't close the gripper, it holds it") {
  AppConfig cfg = defaultConfig();
  FakeArm arm;
  ArmController ctl(cfg, arm);
  ctl.setCalibration(calibrationFor(makeHand(kPalm)));
  double t = 0;
  for (int i = 0; i < 30; ++i, t += kFrameMs) ctl.update(makeHand(kPalm), t);
  for (int i = 0; i < 30; ++i, t += kFrameMs) ctl.update(makeHand(kFist), t);
  CHECK(arm.grips.back() > 0.98f);  // still open
  CHECK(!ctl.status().clawPose);
  // Close with the claw, then curl into a fist: the gripper stays closed.
  for (int i = 0; i < 30; ++i, t += kFrameMs) ctl.update(claw(0.f), t);
  CHECK(arm.grips.back() < 0.02f);
  for (int i = 0; i < 30; ++i, t += kFrameMs) ctl.update(makeHand(kFist), t);
  CHECK(arm.grips.back() < 0.02f);
}

TEST_CASE("arm controller (proportional): a quick grab gets through the filter") {
  AppConfig cfg = defaultConfig();
  FakeArm arm;
  ArmController ctl(cfg, arm);
  ctl.setCalibration(calibrationFor(makeHand(kPalm)));
  double t = 0;
  for (int i = 0; i < 10; ++i, t += kFrameMs) ctl.update(makeHand(kPalm), t);
  for (int i = 0; i < 8; ++i, t += kFrameMs) ctl.update(claw(0.f), t);  // ~0.25 s
  CHECK(arm.grips.back() < 0.2f);
}

TEST_CASE("arm controller (proportional): filter lag on a sudden grab") {
  AppConfig cfg = defaultConfig();
  FakeArm arm;
  ArmController ctl(cfg, arm);
  ctl.setCalibration(calibrationFor(makeHand(kPalm)));
  double t = 0;
  for (int i = 0; i < 30; ++i, t += kFrameMs) ctl.update(makeHand(kPalm), t);
  int frames = 0;
  while (arm.grips.back() > 0.1f && frames < 60) {
    ctl.update(claw(0.f), t);
    t += kFrameMs;
    ++frames;
  }
  std::cerr << "  open -> 10% command took " << frames << " frames (" << frames * kFrameMs
            << " ms at 30 fps)\n";
  CHECK(frames <= 6);
}

TEST_CASE("arm controller (proportional): closing the claw doesn't move the arm") {
  AppConfig cfg = defaultConfig();
  FakeArm arm;
  ArmController ctl(cfg, arm);
  ctl.setCalibration(calibrationFor(makeHand(kPalm)));
  double t = 0;
  for (int i = 0; i < 30; ++i, t += kFrameMs) ctl.update(makeHand(kPalm), t);
  const Vec3 before = arm.targets.back();
  for (int i = 0; i < 30; ++i, t += kFrameMs) ctl.update(partlyOpen(0.2f), t);
  CHECK((arm.targets.back() - before).norm() < 1e-4f);
}

TEST_CASE("arm controller (binary): fist closes, any other shape opens") {
  AppConfig cfg = binaryConfig();
  FakeArm arm;
  ArmController ctl(cfg, arm);
  ctl.setCalibration(calibrationFor(makeHand(kPalm)));
  double t = 0;
  for (int i = 0; i < 10; ++i, t += kFrameMs) ctl.update(makeHand(kFist), t);
  CHECK_NEAR(arm.grips.back(), 0, 1e-6);
  for (int i = 0; i < 10; ++i, t += kFrameMs) ctl.update(makeHand(kPeace), t);
  CHECK_NEAR(arm.grips.back(), 1, 1e-6);
  for (int i = 0; i < 10; ++i, t += kFrameMs) ctl.update(makeHand(kFist), t);
  CHECK_NEAR(arm.grips.back(), 0, 1e-6);
}

TEST_CASE("arm controller: holds position and grip when the hand is lost") {
  AppConfig cfg = defaultConfig();  // proportional
  FakeArm arm;
  ArmController ctl(cfg, arm);
  ctl.setCalibration(calibrationFor(makeHand(kPalm)));
  double t = 0;
  for (int i = 0; i < 10; ++i, t += kFrameMs) ctl.update(claw(0.f), t);
  const auto targetsBefore = arm.targets.size();
  const auto gripsBefore = arm.grips.size();
  for (int i = 0; i < 30; ++i, t += kFrameMs) ctl.update(std::nullopt, t);
  CHECK_EQ(arm.targets.size(), targetsBefore);
  CHECK_EQ(arm.grips.size(), gripsBefore);
  CHECK(ctl.status().gripperOpening < 0.05f);
}

TEST_CASE("arm controller: speed limit stops sudden jumps") {
  AppConfig cfg = defaultConfig();
  cfg.arm.maxSpeed = 0.3f;
  FakeArm arm;
  ArmController ctl(cfg, arm);
  ctl.setCalibration(calibrationFor(makeHand(kPalm)));
  double t = 0;
  ctl.update(makeHand(kPalm), t);
  Pose far;
  far.x += 250;  // teleport to the edge
  t += kFrameMs;
  const Vec3 before = arm.targets.back();
  ctl.update(makeHand(kPalm, far), t);
  CHECK((arm.targets.back() - before).norm() <= 0.3f * 0.033f + 1e-4f);
}

TEST_CASE("config: [arm] section") {
  const auto r = parseConfig(R"(
[arm]
model = models/other/scene.xml
workspace_min = 0.1, -0.2, 0.05
workspace_max = 0.3 0.2 0.3
max_speed = 0.4
gripper_closed = -0.05
depth_range = 0.5
workspace_max = 1 2
gripper_mode = Binary
grip_closed_spread = 0.25
grip_deadband = 0.05
grip_min_straightness = 0.9
grip_min_cutoff = 3
grip_beta = 0.5
)");
  CHECK_EQ(r.warnings.size(), static_cast<std::size_t>(1));  // the 2-number vector
  CHECK_EQ(r.config.arm.model, std::string("models/other/scene.xml"));
  CHECK_NEAR(r.config.arm.workspaceMin.y, -0.2, 1e-6);
  CHECK_NEAR(r.config.arm.workspaceMax.z, 0.3, 1e-6);
  CHECK_NEAR(r.config.arm.maxSpeed, 0.4, 1e-6);
  CHECK_NEAR(r.config.arm.gripperClosed, -0.05, 1e-6);
  CHECK_NEAR(r.config.arm.hand.depthRange, 0.5, 1e-6);
  CHECK(r.config.arm.gripperMode == GripperMode::Binary);
  CHECK_NEAR(r.config.arm.grip.closedSpread, 0.25, 1e-6);
  CHECK_NEAR(r.config.arm.grip.deadband, 0.05, 1e-6);
  CHECK_NEAR(r.config.arm.grip.minStraightness, 0.9, 1e-6);
  CHECK_NEAR(r.config.arm.gripMinCutoff, 3, 1e-9);
  CHECK_NEAR(r.config.arm.gripBeta, 0.5, 1e-9);
}

TEST_CASE("config: gripper_mode defaults to proportional and rejects junk") {
  CHECK(defaultConfig().arm.gripperMode == GripperMode::Proportional);
  const auto r = parseConfig("[arm]\ngripper_mode = sometimes\n");
  CHECK_EQ(r.warnings.size(), static_cast<std::size_t>(1));
  CHECK(r.config.arm.gripperMode == GripperMode::Proportional);
}

TEST_CASE("config: shipped config files parse without warnings") {
  for (const char* name : {"handtracker.ini", "arm.ini"}) {
    const auto r = loadConfigFile(std::string(HT_SOURCE_DIR) + "/config/" + name);
    for (const auto& w : r.warnings) std::cerr << "  " << name << ": " << w << "\n";
    CHECK(r.warnings.empty());
  }
  const auto arm = loadConfigFile(std::string(HT_SOURCE_DIR) + "/config/arm.ini");
  CHECK_NEAR(arm.config.arm.workspaceMax.x, 0.31, 1e-6);
  CHECK_EQ(arm.config.arm.model, std::string("models/so101/scene.xml"));
}


// ---------------------------------------------------------------------------
// Dash 4: hand orientation -> claw orientation
// ---------------------------------------------------------------------------

TEST_CASE("hand frame: calibration palm has fingers up, front towards the camera") {
  const auto palm = makeHand(kPalm);
  const HandFrame f = handFrame(palm, frontSign(palm));
  CHECK_NEAR(f.y.z, 1, 1e-3);  // fingers up
  CHECK_NEAR(f.z.x, 1, 1e-2);  // front faces the camera
  CHECK_NEAR(f.x.y, 1, 1e-2);  // across = user's left
}

TEST_CASE("hand angles: tipping the fingers towards the camera is +flex") {
  const auto cal = calibrationFor(makeHand(kPalm));
  for (float deg : {-40.f, -15.f, 20.f, 45.f}) {
    // Rotating about the left axis by +deg tips the fingers forward.
    const auto h = rotateHand(makeHand(kPalm), kLeft, deg);
    const HandAngles a = handAngles(handFrame(h, cal.frontSign), cal.frame);
    CHECK_NEAR(a.flex, deg * kDeg, 0.02);
    CHECK_NEAR(a.twist, 0, 0.02);
    CHECK_NEAR(a.side, 0, 0.02);
  }
}

TEST_CASE("hand angles: turning about the fingers (doorknob) is twist") {
  const auto cal = calibrationFor(makeHand(kPalm));
  for (float deg : {-80.f, -30.f, 30.f, 80.f}) {
    const auto h = rotateHand(makeHand(kPalm), kUp, deg);
    const HandAngles a = handAngles(handFrame(h, cal.frontSign), cal.frame);
    CHECK_NEAR(a.twist, deg * kDeg, 0.02);
    CHECK_NEAR(a.flex, 0, 0.02);
  }
}

TEST_CASE("hand angles: tilting sideways is side, not flex or twist") {
  const auto cal = calibrationFor(makeHand(kPalm));
  // About the forward axis by +20: fingers lean to the user's right (-Y).
  const auto h = rotateHand(makeHand(kPalm), kForward, 20.f);
  const HandAngles a = handAngles(handFrame(h, cal.frontSign), cal.frame);
  CHECK_NEAR(a.side, -20 * kDeg, 0.02);
  CHECK_NEAR(a.flex, 0, 0.02);
  CHECK_NEAR(a.twist, 0, 0.02);
}

TEST_CASE("hand angles: twist is still measured correctly while flexed") {
  // Twist first, then flex: the swing-twist split must give both back.
  const auto cal = calibrationFor(makeHand(kPalm));
  const auto h = rotateHand(rotateHand(makeHand(kPalm), kUp, 40.f), kLeft, 30.f);
  const HandAngles a = handAngles(handFrame(h, cal.frontSign), cal.frame);
  CHECK_NEAR(a.flex, 30 * kDeg, 0.02);
  CHECK_NEAR(a.twist, 40 * kDeg, 0.02);
}

TEST_CASE("hand angles: same answers for the other hand / back of the hand") {
  Pose other;
  other.mirrored = true;  // a left hand, or a right hand seen from the back
  const auto palm = makeHand(kPalm, other);
  CHECK_NEAR(frontSign(palm), -frontSign(makeHand(kPalm)), 1e-6);  // the normal flips...
  const auto cal = calibrationFor(palm);
  const auto h = rotateHand(rotateHand(palm, kUp, -35.f), kLeft, 25.f);
  const HandAngles a = handAngles(handFrame(h, cal.frontSign), cal.frame);
  CHECK_NEAR(a.flex, 25 * kDeg, 0.02);  // ...but the angles don't
  CHECK_NEAR(a.twist, -35 * kDeg, 0.02);
}

TEST_CASE("hand angles: curling the fingers doesn't change the orientation") {
  const auto cal = calibrationFor(makeHand(kPalm));
  const auto open = rotateHand(makeHand(kPalm), kUp, 30.f);
  const auto claw0 = rotateHand(claw(0.f), kUp, 30.f);
  const auto fist = rotateHand(makeHand(kFist), kUp, 30.f);
  const float t = handAngles(handFrame(open, cal.frontSign), cal.frame).twist;
  CHECK_NEAR(handAngles(handFrame(claw0, cal.frontSign), cal.frame).twist, t, 1e-4);
  CHECK_NEAR(handAngles(handFrame(fist, cal.frontSign), cal.frame).twist, t, 1e-4);
}

TEST_CASE("calibrator: records the neutral hand frame") {
  HandCalibrator calib(500.0, 0.05f, true);
  const auto hand = makeHand(kPalm);
  std::optional<HandSpaceCalibration> r;
  for (double t = 0; t < 700 && !r; t += kFrameMs) r = calib.update(hand, Gesture::Palm, t);
  REQUIRE(r.has_value());
  CHECK_NEAR(r->frontSign, frontSign(hand), 1e-6);
  CHECK_NEAR(r->frame.y.z, 1, 1e-3);
  CHECK_NEAR(r->frame.z.x, 1, 1e-2);
}

TEST_CASE("tilt correction: flexing the hand doesn't look like moving away") {
  const auto palm = makeHand(kPalm);
  HandSpaceConfig on;
  HandSpaceConfig off;
  off.tiltCorrection = false;
  HandSpaceCalibration cal = calibrationFor(palm);
  cal.palmSize = palmSizeNormalized(palm, true);
  for (float deg : {20.f, 40.f, -30.f}) {
    // Real image z is a rough guess; flatten it so only the world points know
    // the tilt (the uncorrected Dash 2 measure includes image z).
    auto h = rotateHand(palm, kLeft, deg);
    for (auto& p : h.image) p.z = 0.f;
    const float zOn = handPosition(h, cal, on).z;
    const float zOff = handPosition(h, cal, off).z;
    std::cerr << "  flex " << deg << " deg: depth " << zOff << " uncorrected, " << zOn
              << " corrected\n";
    CHECK_NEAR(zOn, 0, 0.02);
    CHECK(std::fabs(zOff) > std::fabs(zOn));
  }
}

TEST_CASE("claw measurements are unaffected by hand orientation") {
  const auto cal = calibrationFor(makeHand(kPalm));
  const float ref = handOpenness(claw(0.5f), cal, {});
  for (const Vec3& axis : {kUp, kLeft, kForward}) {
    const auto h = rotateHand(claw(0.5f), axis, 50.f);
    CHECK_NEAR(handOpenness(h, cal, {}), ref, 0.01);
    CHECK(isClawPose(h, {}));
  }
}

namespace {
/// Feeds `hand` for `frames` frames and returns the last wrist target.
WristTarget settle(ArmController& ctl, FakeArm& arm, const HandLandmarks& hand, double& t,
                   int frames = 60) {
  for (int i = 0; i < frames; ++i, t += kFrameMs) ctl.update(hand, t);
  return arm.wrists.back();
}
}  // namespace

TEST_CASE("arm controller (orientation): neutral pose gives the neutral pitch, no roll") {
  AppConfig cfg = defaultConfig();
  FakeArm arm;
  ArmController ctl(cfg, arm);
  ctl.setCalibration(calibrationFor(makeHand(kPalm)));
  double t = 0;
  const WristTarget w = settle(ctl, arm, makeHand(kPalm), t);
  REQUIRE(w.pitch.has_value());
  CHECK_NEAR(*w.pitch, cfg.arm.neutralPitch, 1e-3);
  CHECK_NEAR(w.roll, 0, 1e-3);
}

TEST_CASE("arm controller (orientation): flex tips the claw down, twist rolls it") {
  AppConfig cfg = defaultConfig();
  FakeArm arm;
  ArmController ctl(cfg, arm);
  ctl.setCalibration(calibrationFor(makeHand(kPalm)));
  double t = 0;
  WristTarget w = settle(ctl, arm, rotateHand(makeHand(kPalm), kLeft, 30.f), t);
  CHECK_NEAR(*w.pitch, cfg.arm.neutralPitch - 30 * kDeg, 0.02);  // fingers forward -> claw down
  w = settle(ctl, arm, rotateHand(makeHand(kPalm), kUp, 45.f), t);
  CHECK_NEAR(w.roll, 45 * kDeg, 0.02);
  CHECK_NEAR(*w.pitch, cfg.arm.neutralPitch, 0.02);
}

TEST_CASE("arm controller (orientation): gains, clamps and the turn-rate limit") {
  AppConfig cfg = defaultConfig();
  cfg.arm.rollGain = 2.f;
  cfg.arm.rollLimit = 60 * kDeg;
  FakeArm arm;
  ArmController ctl(cfg, arm);
  ctl.setCalibration(calibrationFor(makeHand(kPalm)));
  double t = 0;
  ctl.update(makeHand(kPalm), t);
  t += kFrameMs;
  ctl.update(rotateHand(makeHand(kPalm), kUp, 50.f), t);  // sudden 50 deg twist
  CHECK(std::fabs(arm.wrists.back().roll) <= cfg.arm.maxAngularSpeed * 0.033f + 1e-3f);
  const WristTarget w = settle(ctl, arm, rotateHand(makeHand(kPalm), kUp, 50.f), t);
  CHECK_NEAR(w.roll, 60 * kDeg, 1e-3);  // 2 x 50 = 100, clamped to 60

  // Pitch can't go below straight down, however far the hand flexes.
  const WristTarget down = settle(ctl, arm, rotateHand(makeHand(kPalm), kLeft, 85.f), t);
  CHECK(*down.pitch >= cfg.arm.pitchMin - 1e-4f);
}

TEST_CASE("arm controller (orientation): holds when the hand is lost") {
  AppConfig cfg = defaultConfig();
  FakeArm arm;
  ArmController ctl(cfg, arm);
  ctl.setCalibration(calibrationFor(makeHand(kPalm)));
  double t = 0;
  settle(ctl, arm, rotateHand(makeHand(kPalm), kUp, 40.f), t);
  const auto before = arm.wrists.size();
  for (int i = 0; i < 30; ++i, t += kFrameMs) ctl.update(std::nullopt, t);
  CHECK_EQ(arm.wrists.size(), before);
  CHECK_NEAR(ctl.status().wrist.roll, 40 * kDeg, 0.02);
}

TEST_CASE("arm controller (orientation): turning the hand barely moves the arm") {
  AppConfig cfg = defaultConfig();
  FakeArm arm;
  ArmController ctl(cfg, arm);
  HandCalibrator calib(500.0, 0.05f, true);
  std::optional<HandSpaceCalibration> cal;
  for (double t0 = 0; t0 < 700 && !cal; t0 += kFrameMs) cal = calib.update(makeHand(kPalm), Gesture::Palm, t0);
  ctl.setCalibration(*cal);
  double t = 0;
  for (int i = 0; i < 60; ++i, t += kFrameMs) ctl.update(makeHand(kPalm), t);
  const Vec3 before = arm.targets.back();
  for (const auto& h : {rotateHand(makeHand(kPalm), kUp, 60.f), rotateHand(makeHand(kPalm), kLeft, 30.f)}) {
    for (int i = 0; i < 60; ++i, t += kFrameMs) ctl.update(h, t);
    const float moved = (arm.targets.back() - before).norm();
    std::cerr << "  arm target moved " << moved * 1000.f << " mm\n";
    CHECK(moved < 0.01f);
  }
}

TEST_CASE("arm controller (orientation): off = Dash 3 behaviour") {
  AppConfig cfg = defaultConfig();
  cfg.arm.orientation = false;
  FakeArm arm;
  ArmController ctl(cfg, arm);
  ctl.setCalibration(calibrationFor(makeHand(kPalm)));
  double t = 0;
  for (int i = 0; i < 30; ++i, t += kFrameMs) ctl.update(rotateHand(makeHand(kPalm), kUp, 45.f), t);
  CHECK(!arm.wrists.back().pitch.has_value());
  CHECK_NEAR(arm.wrists.back().roll, 0, 1e-6);
}

TEST_CASE("config: Dash 4 orientation keys (degrees in the file)") {
  const auto r = parseConfig(R"(
[arm]
orientation = false
neutral_pitch_deg = -45
pitch_gain = 0.5
roll_gain = 1.5
pitch_min_deg = -80
pitch_max_deg = 10
roll_limit_deg = 120
orient_min_cutoff = 2
orient_beta = 0.25
max_angular_speed_deg = 90
ik_pitch_search_steps = 4
neutral_pitch_deg = down
)");
  CHECK_EQ(r.warnings.size(), static_cast<std::size_t>(1));  // "down"
  const auto& a = r.config.arm;
  CHECK(!a.orientation);
  CHECK_NEAR(a.neutralPitch, -45 * kDeg, 1e-5);
  CHECK_NEAR(a.pitchGain, 0.5, 1e-6);
  CHECK_NEAR(a.rollGain, 1.5, 1e-6);
  CHECK_NEAR(a.pitchMin, -80 * kDeg, 1e-5);
  CHECK_NEAR(a.pitchMax, 10 * kDeg, 1e-5);
  CHECK_NEAR(a.rollLimit, 120 * kDeg, 1e-5);
  CHECK_NEAR(a.orientMinCutoff, 2, 1e-9);
  CHECK_NEAR(a.orientBeta, 0.25, 1e-9);
  CHECK_NEAR(a.maxAngularSpeed, 90 * kDeg, 1e-5);
  CHECK_EQ(a.ikPitchSearchSteps, 4);
}
