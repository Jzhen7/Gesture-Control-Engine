#include "handtracker/core/controller.hpp"

#include "fake_system_control.hpp"
#include "hand_builder.hpp"
#include "test_framework.hpp"

using namespace ht;
using namespace testutil;

namespace {

constexpr double kFrameMs = 33.0;

struct Rig {
  AppConfig cfg = defaultConfig();
  FakeSystemControl sys;
  std::vector<std::string> messages;
  GestureController ctl{cfg, sys, [this](const std::string& m) { messages.push_back(m); }};
  double t = 0;

  void frames(const HandSpec& spec, int n, Pose pose = {}) {
    for (int i = 0; i < n; ++i, t += kFrameMs) ctl.update(makeHand(spec, pose), t);
  }
  void noHand(int n) {
    for (int i = 0; i < n; ++i, t += kFrameMs) ctl.update(std::nullopt, t);
  }
  void swipe(const HandSpec& spec, float x0, float x1) {
    for (int i = 0; i < 10; ++i, t += kFrameMs) {
      Pose pose;
      pose.x = x0 + (x1 - x0) * static_cast<float>(i) / 9.f;
      ctl.update(makeHand(spec, pose), t);
    }
  }
};

}  // namespace

TEST_CASE("controller: fist hold enters brightness mode; raising hand brightens") {
  Rig r;
  r.frames(kFist, 40);
  REQUIRE(r.ctl.mode() == Mode::Brightness);

  Pose low;
  low.y = 440;  // palm centre near the bottom
  r.frames(kPeace, 15, low);
  REQUIRE(!r.sys.brightness.empty());
  const float dim = r.sys.brightness.back();

  Pose high;
  high.y = 150;
  r.frames(kPeace, 40, high);
  CHECK(r.sys.brightness.back() > dim + 0.3f);
  REQUIRE(r.ctl.status().level.has_value());
}

TEST_CASE("controller: open palm exits a mode") {
  Rig r;
  r.frames(kFist, 40);
  REQUIRE(r.ctl.mode() == Mode::Brightness);
  r.frames(kPalm, 6);
  CHECK_EQ(r.ctl.mode(), Mode::Idle);
}

TEST_CASE("controller: volume dial follows hand roll") {
  Rig r;
  r.frames(kTwo, 40);
  REQUIRE(r.ctl.mode() == Mode::Volume);

  Pose left;
  left.rotationDeg = -50;  // leaning left -> quiet
  r.frames(kThree, 30, left);
  REQUIRE(!r.sys.volumes.empty());
  CHECK(r.sys.volumes.back() < 0.2f);

  Pose right;
  right.rotationDeg = 50;  // leaning right -> loud
  r.frames(kThree, 30, right);
  CHECK(r.sys.volumes.back() > 0.8f);
}

TEST_CASE("controller: mode times out when the hand disappears") {
  Rig r;
  r.frames(kTwo, 40);
  REQUIRE(r.ctl.mode() == Mode::Volume);
  r.noHand(50);  // ~1.6 s: still in mode
  CHECK_EQ(r.ctl.mode(), Mode::Volume);
  r.noHand(60);  // > 3 s total
  CHECK_EQ(r.ctl.mode(), Mode::Idle);
}

TEST_CASE("controller: process-gated swipe only fires when the app runs") {
  Rig r;
  r.swipe(kThree, 150, 450);
  CHECK(r.sys.keys.empty());  // spotify.exe not running
  REQUIRE(!r.messages.empty());
  CHECK(r.messages.back().find("skipped") != std::string::npos);

  // Hand leaves the frame and comes back at the swipe start, then rests until
  // the cooldown expires (jumping straight back would itself be a swipe left).
  r.noHand(1);
  Pose start;
  start.x = 150;
  r.frames(kThree, 20, start);
  r.sys.running.insert("spotify.exe");
  r.swipe(kThree, 150, 450);
  REQUIRE(r.sys.keys.size() == 1);
  CHECK_EQ(r.sys.keys[0], std::string("media_next"));
}

TEST_CASE("controller: middle finger hold requests quit") {
  Rig r;
  r.frames(kMiddle, 40);
  CHECK(r.ctl.quitRequested());
}

TEST_CASE("controller: brief flicker does not cancel a hold") {
  Rig r;
  for (int i = 0; i < 40; ++i, r.t += kFrameMs) {
    // Every 7th frame the classifier glitches; the smoother absorbs it.
    r.ctl.update(makeHand(i % 7 == 3 ? kPalm : kFist), r.t);
  }
  CHECK_EQ(r.ctl.mode(), Mode::Brightness);
}
