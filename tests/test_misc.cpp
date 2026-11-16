// Filters, mappings, orientation, mouse control and the async decorator.
#include <algorithm>
#include <cmath>
#include <memory>

#include "fake_system_control.hpp"
#include "hand_builder.hpp"
#include "handtracker/core/async_system_control.hpp"
#include "handtracker/core/filters.hpp"
#include "handtracker/core/mapping.hpp"
#include "handtracker/core/mouse_controller.hpp"
#include "handtracker/core/orientation.hpp"
#include "test_framework.hpp"

using namespace ht;
using namespace testutil;

TEST_CASE("mapping: mapToUnit clamps and supports inverted ranges") {
  CHECK_NEAR(mapToUnit(5, 0, 10), 0.5, 1e-6);
  CHECK_NEAR(mapToUnit(-5, 0, 10), 0.0, 1e-6);
  CHECK_NEAR(mapToUnit(50, 0, 10), 1.0, 1e-6);
  CHECK_NEAR(mapToUnit(0.2f, 0.8f, 0.2f), 1.0, 1e-6);
  CHECK_NEAR(mapToUnit(3, 3, 3), 0.0, 1e-6);
}

TEST_CASE("mapping: roll angle tracks in-plane rotation") {
  for (float deg : {-60.f, -20.f, 0.f, 35.f, 70.f}) {
    Pose pose;
    pose.rotationDeg = deg;
    CHECK_NEAR(handRollDegrees(makeHand(kThree, pose)), deg, 0.5);
  }
}

TEST_CASE("mapping: pinch ratio is small when thumb meets index") {
  auto hand = makeHand(kPalm);
  CHECK(pinchRatio(hand) > 0.5f);
  hand[LandmarkId::ThumbTip] = hand[LandmarkId::IndexTip];
  CHECK_NEAR(pinchRatio(hand), 0.0, 1e-6);
}

TEST_CASE("filters: level filter smooths and rate-limits") {
  LevelFilter f({0.5f, 0.05f, 100.0});
  REQUIRE(f.update(0.0f, 0).has_value());  // first value always applies
  CHECK(!f.update(1.0f, 50).has_value());  // too soon
  CHECK_NEAR(f.value(), 0.5, 1e-6);        // but smoothing continued
  const auto v = f.update(1.0f, 150);
  REQUIRE(v.has_value());
  CHECK_NEAR(*v, 0.75, 1e-6);
  CHECK(!f.update(0.76f, 400).has_value());  // change below minDelta
}

TEST_CASE("filters: one euro converges on a constant and reduces jitter") {
  OneEuroFilter f(1.0, 0.0);
  double out = 0;
  double maxDev = 0;
  for (int i = 0; i < 300; ++i) {
    const double noisy = 0.5 + ((i % 2) ? 0.02 : -0.02);
    out = f.filter(noisy, i / 30.0);
    if (i > 30) maxDev = std::max(maxDev, std::fabs(out - 0.5));
  }
  CHECK(maxDev < 0.01);  // jitter amplitude was 0.02
}

TEST_CASE("orientation: upright flat hand has zero z-rotation; tracks roll") {
  const auto o = computeOrientation(makeHand(kPalm));
  CHECK_NEAR(o.rotZ, 0.0, 0.5);
  CHECK_NEAR(std::fabs(o.palmNormal.z), 1.0, 1e-4);

  Pose pose;
  pose.rotationDeg = 40;
  CHECK_NEAR(computeOrientation(makeHand(kPalm, pose)).rotZ, 40.0, 0.5);
}

TEST_CASE("orientation: prefers world landmarks when present") {
  auto hand = makeHand(kPalm);
  HandLandmarks::Points world = hand.image;
  for (auto& p : world) p = Vec3(p.y, -p.x, 0.f);  // rotate 90 degrees
  hand.world = world;
  CHECK_NEAR(std::fabs(computeOrientation(hand).rotZ), 90.0, 0.5);
}

TEST_CASE("mouse: active region maps to full screen and clamps") {
  CHECK_NEAR(mapActiveRegion(0.1f, 0.1f), 0.0, 1e-6);
  CHECK_NEAR(mapActiveRegion(0.5f, 0.1f), 0.5, 1e-6);
  CHECK_NEAR(mapActiveRegion(0.9f, 0.1f), 1.0, 1e-6);
  CHECK_NEAR(mapActiveRegion(0.99f, 0.1f), 1.0, 1e-6);  // v1 froze the cursor here
}

TEST_CASE("mouse: pinch presses with hysteresis, losing the hand releases") {
  FakeSystemControl sys;
  MouseConfig cfg;
  MouseController mouse(cfg, sys);
  double t = 0;

  auto open = makeHand(kPalm);
  mouse.update(open, t += 33);
  CHECK(sys.buttons.empty());
  CHECK(sys.cursorMoves == 1);

  auto pinched = open;
  pinched[LandmarkId::ThumbTip] = pinched[LandmarkId::IndexTip];
  mouse.update(pinched, t += 33);
  REQUIRE(sys.buttons.size() == 1);
  CHECK(sys.buttons[0]);

  // In the hysteresis band: stays pressed.
  auto partial = open;
  const float palm = open.palmSize();
  partial[LandmarkId::ThumbTip] = partial[LandmarkId::IndexTip] + Vec3(0.3f * palm, 0.f, 0.f);
  mouse.update(partial, t += 33);
  CHECK_EQ(sys.buttons.size(), static_cast<std::size_t>(1));

  mouse.update(std::nullopt, t += 33);
  REQUIRE(sys.buttons.size() == 2);
  CHECK(!sys.buttons[1]);
}

TEST_CASE("async: commands run in order and continuous ones coalesce") {
  auto fake = std::make_unique<FakeSystemControl>();
  FakeSystemControl* raw = fake.get();
  {
    AsyncSystemControl async(std::move(fake));
    async.sendKeys(*parseHotkey("alt+tab"));
    for (int i = 0; i <= 100; ++i) async.setVolume(static_cast<float>(i) / 100.f);
    async.setMouseButton(true);
    async.flush();
    REQUIRE(!raw->volumes.empty());
    CHECK(raw->volumes.size() <= 101u);
    CHECK_NEAR(raw->volumes.back(), 1.0, 1e-6);  // newest value always wins
    REQUIRE(raw->keys.size() == 1);
    CHECK_EQ(raw->keys[0], std::string("alt+tab"));
    REQUIRE(raw->buttons.size() == 1);
    raw->running.insert("x.exe");
    CHECK(async.isProcessRunning("x.exe"));
  }
}
