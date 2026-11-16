// Static (single-frame) gesture recognition.
//
// v1 classified fingers by comparing raw y coordinates (tip above knuckle), which
// only works with an upright hand, and decided the thumb by comparing x
// coordinates, which flips meaning between left/right hands and mirrored feeds.
// v2 instead measures each finger relative to the hand itself, which makes the
// classifier invariant to in-plane rotation, hand scale and handedness.
#pragma once

#include <cstddef>
#include <deque>
#include <optional>
#include <string_view>

#include "handtracker/core/landmarks.hpp"

namespace ht {

/// Finger-count gestures count from the thumb outward, as in the original project:
/// One = thumb, Two = thumb + index, Three = thumb + index + middle, ...
enum class Gesture {
  None,
  Fist,          ///< All fingers folded.
  Peace,         ///< Index + middle extended, thumb tucked.
  MiddleFinger,  ///< Only the middle finger extended (quit gesture).
  One,           ///< Thumb only.
  Two,           ///< Thumb + index.
  Three,         ///< Thumb + index + middle.
  Four,          ///< Thumb + index + middle + ring.
  Palm,          ///< All five extended.
};

std::string_view toString(Gesture g);
std::optional<Gesture> gestureFromString(std::string_view name);

struct FingerState {
  bool thumb = false;
  bool index = false;
  bool middle = false;
  bool ring = false;
  bool pinky = false;

  int count() const { return thumb + index + middle + ring + pinky; }
  bool operator==(const FingerState& o) const {
    return thumb == o.thumb && index == o.index && middle == o.middle && ring == o.ring &&
           pinky == o.pinky;
  }
};

struct ClassifierConfig {
  /// A finger counts as extended when |tip - wrist| > ratio * |pip - wrist|.
  float fingerExtensionRatio = 1.1f;
  /// The thumb counts as extended when |tip - pinkyMcp| > ratio * |ip - pinkyMcp|.
  float thumbExtensionRatio = 1.05f;
};

FingerState computeFingerState(const HandLandmarks& hand, const ClassifierConfig& cfg = {});
Gesture classifyFingers(const FingerState& fingers);
Gesture classifyGesture(const HandLandmarks& hand, const ClassifierConfig& cfg = {});

/// Majority vote over the last N frames. Removes single-frame flicker that would
/// otherwise reset hold timers and break swipes.
class GestureSmoother {
 public:
  explicit GestureSmoother(std::size_t window = 5);

  /// Adds a frame and returns the smoothed gesture (None when no gesture has a majority).
  Gesture push(Gesture g);
  void reset();
  std::size_t window() const { return window_; }

 private:
  std::size_t window_;
  std::deque<Gesture> history_;
};

}  // namespace ht
