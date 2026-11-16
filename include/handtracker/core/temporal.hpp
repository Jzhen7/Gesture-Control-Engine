// Dynamic gesture detection: swipes, hand flips and timed holds.
//
// v1 measured everything in frame counts ("6 frames ~ 0.3 s at 20 fps"), so its
// behaviour changed with camera speed and CPU load. v2 uses timestamps, adds a
// cooldown so one motion can't fire twice, latches holds so they fire once, and
// requires the hand to actually be still for a hold.
#pragma once

#include <deque>
#include <optional>
#include <string_view>

#include "handtracker/core/gesture.hpp"
#include "handtracker/core/landmarks.hpp"

namespace ht {

enum class EventType { SwipeLeft, SwipeRight, SwipeUp, SwipeDown, Flip, Hold };

std::string_view toString(EventType e);
std::optional<EventType> eventFromString(std::string_view name);

struct GestureEvent {
  EventType type;
  Gesture gesture;
};

struct TemporalConfig {
  double swipeWindowMs = 300.0;    ///< Motion is measured over this sliding window.
  float swipeDistance = 0.15f;     ///< Normalised displacement needed for a swipe.
  double holdMs = 1000.0;          ///< How long a gesture must be held still.
  float holdMaxMotion = 0.08f;     ///< Max normalised wrist drift while holding.
  double cooldownMs = 400.0;       ///< Quiet period after any event.
  float flipMinSeparation = 0.15f; ///< Knuckle separation (in palm sizes) for a confident flip side.
};

class EventDetector {
 public:
  explicit EventDetector(TemporalConfig cfg = {});

  /// Feeds one frame of a visible hand. Returns at most one event.
  std::optional<GestureEvent> update(Gesture gesture, const HandLandmarks& hand, double tMs);

  /// Forget all history (call when the hand is lost or the mode changes).
  void reset();

  /// Progress of the current hold in [0, 1], for UI feedback.
  float holdProgress(double tMs) const;
  Gesture holdGesture() const { return holdGesture_; }

  const TemporalConfig& config() const { return cfg_; }

 private:
  struct Sample {
    double t;
    float x;
    float y;
    int side;  ///< +1 / -1: which way the index/pinky knuckles are ordered; 0 = edge-on.
  };

  void resetHold();
  std::optional<GestureEvent> emit(EventType type, Gesture g, double tMs);

  TemporalConfig cfg_;
  std::deque<Sample> samples_;
  Gesture holdGesture_ = Gesture::None;
  double holdStart_ = 0.0;
  float holdX_ = 0.f;
  float holdY_ = 0.f;
  bool holdLatched_ = false;
  double cooldownUntil_ = -1e300;
};

}  // namespace ht
