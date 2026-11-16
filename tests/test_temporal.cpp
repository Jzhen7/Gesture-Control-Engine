#include "handtracker/core/temporal.hpp"

#include <optional>
#include <vector>

#include "hand_builder.hpp"
#include "test_framework.hpp"

using namespace ht;
using namespace testutil;

namespace {

constexpr double kFrameMs = 33.0;

/// Feeds frames moving the wrist linearly and returns all events.
std::vector<GestureEvent> feedMotion(EventDetector& d, const HandSpec& spec, Gesture g,
                                     float x0, float y0, float x1, float y1, int frames,
                                     double& t) {
  std::vector<GestureEvent> events;
  for (int i = 0; i < frames; ++i) {
    const float k = frames > 1 ? static_cast<float>(i) / static_cast<float>(frames - 1) : 0.f;
    Pose pose;
    pose.x = x0 + (x1 - x0) * k;
    pose.y = y0 + (y1 - y0) * k;
    if (auto ev = d.update(g, makeHand(spec, pose), t)) events.push_back(*ev);
    t += kFrameMs;
  }
  return events;
}

}  // namespace

TEST_CASE("temporal: fast horizontal motion is a swipe in the right direction") {
  EventDetector d;
  double t = 0;
  auto ev = feedMotion(d, kThree, Gesture::Three, 200, 300, 420, 300, 8, t);
  REQUIRE(ev.size() == 1);
  CHECK_EQ(ev[0].type, EventType::SwipeRight);
  CHECK_EQ(ev[0].gesture, Gesture::Three);

  EventDetector d2;
  t = 0;
  ev = feedMotion(d2, kOne, Gesture::One, 420, 300, 200, 300, 8, t);
  REQUIRE(ev.size() == 1);
  CHECK_EQ(ev[0].type, EventType::SwipeLeft);
}

TEST_CASE("temporal: vertical swipes use the dominant axis") {
  EventDetector d;
  double t = 0;
  auto ev = feedMotion(d, kPalm, Gesture::Palm, 320, 150, 340, 400, 8, t);
  REQUIRE(ev.size() == 1);
  CHECK_EQ(ev[0].type, EventType::SwipeDown);
}

TEST_CASE("temporal: slow drift is not a swipe") {
  EventDetector d;
  double t = 0;
  // Same distance as a swipe, but over ~2.6 s: never exceeds the threshold in a 300 ms window.
  const auto ev = feedMotion(d, kThree, Gesture::Three, 200, 300, 420, 300, 80, t);
  for (const auto& e : ev) CHECK(e.type != EventType::SwipeRight);
}

TEST_CASE("temporal: one swipe fires once (cooldown)") {
  EventDetector d;
  double t = 0;
  const auto ev = feedMotion(d, kThree, Gesture::Three, 100, 300, 560, 300, 14, t);
  int swipes = 0;
  for (const auto& e : ev) swipes += e.type == EventType::SwipeRight;
  CHECK_EQ(swipes, 1);
}

TEST_CASE("temporal: holding still fires one hold after hold_ms") {
  TemporalConfig cfg;
  cfg.holdMs = 1000;
  EventDetector d(cfg);
  double t = 0;
  std::vector<GestureEvent> events;
  for (int i = 0; i < 70; ++i) {  // ~2.3 s
    if (auto ev = d.update(Gesture::Fist, makeHand(kFist), t)) events.push_back(*ev);
    if (t < 900) CHECK(events.empty());
    t += kFrameMs;
  }
  REQUIRE(events.size() == 1);  // Latched: doesn't repeat while still held.
  CHECK_EQ(events[0].type, EventType::Hold);
  CHECK_EQ(events[0].gesture, Gesture::Fist);
}

TEST_CASE("temporal: hold progress rises from 0 to 1") {
  EventDetector d;
  double t = 0;
  d.update(Gesture::Two, makeHand(kTwo), t);
  CHECK_NEAR(d.holdProgress(t), 0.0, 1e-6);
  t = 500;
  d.update(Gesture::Two, makeHand(kTwo), t);
  CHECK_NEAR(d.holdProgress(t), 0.5, 1e-6);
}

TEST_CASE("temporal: a drifting hand does not trigger a hold") {
  EventDetector d;
  double t = 0;
  // Moves ~0.3 of the frame over 2 s: too slow to swipe, too much to hold.
  const auto ev = feedMotion(d, kFist, Gesture::Fist, 200, 300, 400, 300, 60, t);
  for (const auto& e : ev) CHECK(e.type != EventType::Hold);
}

TEST_CASE("temporal: changing gesture restarts the hold timer") {
  EventDetector d;
  double t = 0;
  for (int i = 0; i < 20; ++i, t += kFrameMs) CHECK(!d.update(Gesture::Two, makeHand(kTwo), t));
  for (int i = 0; i < 20; ++i, t += kFrameMs) CHECK(!d.update(Gesture::Four, makeHand(kFour), t));
}

TEST_CASE("temporal: flipping the hand over is a flip") {
  EventDetector d;
  double t = 0;
  std::vector<GestureEvent> events;
  for (int i = 0; i < 6; ++i, t += kFrameMs) {
    Pose pose;
    pose.mirrored = i >= 3;  // Knuckle order inverts half-way through.
    if (auto ev = d.update(Gesture::Three, makeHand(kThree, pose), t)) events.push_back(*ev);
  }
  REQUIRE(events.size() == 1);
  CHECK_EQ(events[0].type, EventType::Flip);
}

TEST_CASE("temporal: unrecognised pose resets motion") {
  EventDetector d;
  double t = 0;
  Pose pose;
  pose.x = 200;
  d.update(Gesture::Three, makeHand(kThree, pose), t);
  t += kFrameMs;
  d.update(Gesture::None, makeHand(kThree, pose), t);
  t += kFrameMs;
  pose.x = 420;
  CHECK(!d.update(Gesture::Three, makeHand(kThree, pose), t).has_value());
}

TEST_CASE("temporal: event names round-trip") {
  for (EventType e : {EventType::SwipeLeft, EventType::SwipeRight, EventType::SwipeUp,
                      EventType::SwipeDown, EventType::Flip, EventType::Hold}) {
    const auto parsed = eventFromString(toString(e));
    REQUIRE(parsed.has_value());
    CHECK_EQ(*parsed, e);
  }
}
