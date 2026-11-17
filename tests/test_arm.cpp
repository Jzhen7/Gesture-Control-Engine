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
  std::vector<bool> grips;
  void setTarget(const Vec3& p) override { targets.push_back(p); }
  void setGripper(bool closed) override { grips.push_back(closed); }
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
  return c;
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
  CHECK(!arm.grips.back());
}

TEST_CASE("arm controller: fist closes, any other shape opens") {
  AppConfig cfg = defaultConfig();
  FakeArm arm;
  ArmController ctl(cfg, arm);
  ctl.setCalibration(calibrationFor(makeHand(kPalm)));
  double t = 0;
  for (int i = 0; i < 10; ++i, t += kFrameMs) ctl.update(makeHand(kFist), t);
  CHECK(arm.grips.back());
  for (int i = 0; i < 10; ++i, t += kFrameMs) ctl.update(makeHand(kPeace), t);
  CHECK(!arm.grips.back());
  for (int i = 0; i < 10; ++i, t += kFrameMs) ctl.update(makeHand(kFist), t);
  CHECK(arm.grips.back());
}

TEST_CASE("arm controller: holds position and grip when the hand is lost") {
  AppConfig cfg = defaultConfig();
  FakeArm arm;
  ArmController ctl(cfg, arm);
  ctl.setCalibration(calibrationFor(makeHand(kPalm)));
  double t = 0;
  for (int i = 0; i < 10; ++i, t += kFrameMs) ctl.update(makeHand(kFist), t);
  const auto targetsBefore = arm.targets.size();
  const auto gripsBefore = arm.grips.size();
  for (int i = 0; i < 30; ++i, t += kFrameMs) ctl.update(std::nullopt, t);
  CHECK_EQ(arm.targets.size(), targetsBefore);
  CHECK_EQ(arm.grips.size(), gripsBefore);
  CHECK(ctl.status().gripperClosed);
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
)");
  CHECK_EQ(r.warnings.size(), static_cast<std::size_t>(1));  // the 2-number vector
  CHECK_EQ(r.config.arm.model, std::string("models/other/scene.xml"));
  CHECK_NEAR(r.config.arm.workspaceMin.y, -0.2, 1e-6);
  CHECK_NEAR(r.config.arm.workspaceMax.z, 0.3, 1e-6);
  CHECK_NEAR(r.config.arm.maxSpeed, 0.4, 1e-6);
  CHECK_NEAR(r.config.arm.gripperClosed, -0.05, 1e-6);
  CHECK_NEAR(r.config.arm.hand.depthRange, 0.5, 1e-6);
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
