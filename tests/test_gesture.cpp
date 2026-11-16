#include "handtracker/core/gesture.hpp"

#include "hand_builder.hpp"
#include "test_framework.hpp"

using namespace ht;
using namespace testutil;

TEST_CASE("gesture: every gesture is recognised upright") {
  CHECK_EQ(classifyGesture(makeHand(kFist)), Gesture::Fist);
  CHECK_EQ(classifyGesture(makeHand(kPeace)), Gesture::Peace);
  CHECK_EQ(classifyGesture(makeHand(kMiddle)), Gesture::MiddleFinger);
  CHECK_EQ(classifyGesture(makeHand(kOne)), Gesture::One);
  CHECK_EQ(classifyGesture(makeHand(kTwo)), Gesture::Two);
  CHECK_EQ(classifyGesture(makeHand(kThree)), Gesture::Three);
  CHECK_EQ(classifyGesture(makeHand(kFour)), Gesture::Four);
  CHECK_EQ(classifyGesture(makeHand(kPalm)), Gesture::Palm);
}

TEST_CASE("gesture: finger state is computed per finger") {
  const FingerState f = computeFingerState(makeHand(kThree));
  CHECK(f.thumb);
  CHECK(f.index);
  CHECK(f.middle);
  CHECK(!f.ring);
  CHECK(!f.pinky);
  CHECK_EQ(f.count(), 3);
}

TEST_CASE("gesture: invariant to in-plane rotation (v1 only worked upright)") {
  for (float deg : {-150.f, -90.f, -45.f, 30.f, 90.f, 180.f}) {
    Pose pose;
    pose.rotationDeg = deg;
    CHECK_EQ(classifyGesture(makeHand(kFist, pose)), Gesture::Fist);
    CHECK_EQ(classifyGesture(makeHand(kThree, pose)), Gesture::Three);
    CHECK_EQ(classifyGesture(makeHand(kPalm, pose)), Gesture::Palm);
    CHECK_EQ(classifyGesture(makeHand(kPeace, pose)), Gesture::Peace);
  }
}

TEST_CASE("gesture: invariant to handedness/mirroring (v1's thumb test flipped)") {
  Pose pose;
  pose.mirrored = true;
  CHECK_EQ(classifyGesture(makeHand(kOne, pose)), Gesture::One);
  CHECK_EQ(classifyGesture(makeHand(kFist, pose)), Gesture::Fist);
  CHECK_EQ(classifyGesture(makeHand(kFour, pose)), Gesture::Four);
}

TEST_CASE("gesture: invariant to scale (distance from camera)") {
  for (float scale : {30.f, 100.f, 250.f}) {
    Pose pose;
    pose.scale = scale;
    CHECK_EQ(classifyGesture(makeHand(kTwo, pose)), Gesture::Two);
  }
}

TEST_CASE("gesture: unmapped finger combinations are None") {
  CHECK_EQ(classifyGesture(makeHand({false, false, false, false, true})), Gesture::None);  // pinky
  CHECK_EQ(classifyGesture(makeHand({true, false, false, false, true})), Gesture::None);   // shaka
}

TEST_CASE("gesture: names round-trip, including v1 names") {
  for (Gesture g : {Gesture::Fist, Gesture::Peace, Gesture::MiddleFinger, Gesture::One,
                    Gesture::Two, Gesture::Three, Gesture::Four, Gesture::Palm}) {
    const auto parsed = gestureFromString(toString(g));
    REQUIRE(parsed.has_value());
    CHECK_EQ(*parsed, g);
  }
  CHECK_EQ(*gestureFromString("3"), Gesture::Three);
  CHECK_EQ(*gestureFromString("PALM"), Gesture::Palm);
  CHECK(!gestureFromString("wave").has_value());
}

TEST_CASE("smoother: majority vote suppresses single-frame flicker") {
  GestureSmoother s(5);
  for (int i = 0; i < 5; ++i) s.push(Gesture::Three);
  CHECK_EQ(s.push(Gesture::Four), Gesture::Three);  // one outlier
  CHECK_EQ(s.push(Gesture::Three), Gesture::Three);
}

TEST_CASE("smoother: switches after a sustained change, None when ambiguous") {
  GestureSmoother s(5);
  for (int i = 0; i < 5; ++i) s.push(Gesture::Fist);
  s.push(Gesture::Palm);
  s.push(Gesture::Palm);
  CHECK_EQ(s.push(Gesture::Palm), Gesture::Palm);  // 3 of 5

  GestureSmoother t(4);
  t.push(Gesture::One);
  t.push(Gesture::One);
  t.push(Gesture::Two);
  CHECK_EQ(t.push(Gesture::Two), Gesture::None);  // 2-2 tie
}
