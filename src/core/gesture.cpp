#include "handtracker/core/gesture.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <string>

namespace ht {
namespace {

struct GestureName {
  Gesture gesture;
  std::string_view name;
};

constexpr std::array<GestureName, 9> kGestureNames{{
    {Gesture::None, "none"},
    {Gesture::Fist, "fist"},
    {Gesture::Peace, "peace"},
    {Gesture::MiddleFinger, "middle"},
    {Gesture::One, "one"},
    {Gesture::Two, "two"},
    {Gesture::Three, "three"},
    {Gesture::Four, "four"},
    {Gesture::Palm, "palm"},
}};

std::string lower(std::string_view s) {
  std::string out(s);
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return out;
}

bool fingerExtended(const HandLandmarks& h, LandmarkId pip, LandmarkId tip, float ratio) {
  const Vec3& wrist = h[LandmarkId::Wrist];
  return distance(h[tip], wrist) > ratio * distance(h[pip], wrist);
}

}  // namespace

std::string_view toString(Gesture g) {
  for (const auto& entry : kGestureNames) {
    if (entry.gesture == g) return entry.name;
  }
  return "none";
}

std::optional<Gesture> gestureFromString(std::string_view name) {
  const std::string key = lower(name);
  for (const auto& entry : kGestureNames) {
    if (entry.name == key) return entry.gesture;
  }
  // Accept v1's names too.
  if (key == "1") return Gesture::One;
  if (key == "2") return Gesture::Two;
  if (key == "3") return Gesture::Three;
  if (key == "4") return Gesture::Four;
  if (key == "middle_finger") return Gesture::MiddleFinger;
  return std::nullopt;
}

FingerState computeFingerState(const HandLandmarks& h, const ClassifierConfig& cfg) {
  FingerState f;
  const float r = cfg.fingerExtensionRatio;
  f.index = fingerExtended(h, LandmarkId::IndexPip, LandmarkId::IndexTip, r);
  f.middle = fingerExtended(h, LandmarkId::MiddlePip, LandmarkId::MiddleTip, r);
  f.ring = fingerExtended(h, LandmarkId::RingPip, LandmarkId::RingTip, r);
  f.pinky = fingerExtended(h, LandmarkId::PinkyPip, LandmarkId::PinkyTip, r);

  // A tucked thumb folds across the palm towards the pinky knuckle, so its tip
  // ends up closer to the pinky base than its IP joint is. This holds for both
  // hands, mirrored or not, and at any in-plane rotation.
  const Vec3& pinkyBase = h[LandmarkId::PinkyMcp];
  f.thumb = distance(h[LandmarkId::ThumbTip], pinkyBase) >
            cfg.thumbExtensionRatio * distance(h[LandmarkId::ThumbIp], pinkyBase);
  return f;
}

Gesture classifyFingers(const FingerState& f) {
  //                   thumb  index  middle  ring   pinky
  struct Pattern {
    bool thumb, index, middle, ring, pinky;
    Gesture gesture;
  };
  static constexpr std::array<Pattern, 7> kPatterns{{
      {false, false, false, false, false, Gesture::Fist},
      {false, true, true, false, false, Gesture::Peace},
      {true, false, false, false, false, Gesture::One},
      {true, true, false, false, false, Gesture::Two},
      {true, true, true, false, false, Gesture::Three},
      {true, true, true, true, false, Gesture::Four},
      {true, true, true, true, true, Gesture::Palm},
  }};
  for (const auto& p : kPatterns) {
    if (p.thumb == f.thumb && p.index == f.index && p.middle == f.middle && p.ring == f.ring &&
        p.pinky == f.pinky) {
      return p.gesture;
    }
  }
  // The middle finger gesture ignores the thumb.
  if (!f.index && f.middle && !f.ring && !f.pinky) return Gesture::MiddleFinger;
  return Gesture::None;
}

Gesture classifyGesture(const HandLandmarks& hand, const ClassifierConfig& cfg) {
  return classifyFingers(computeFingerState(hand, cfg));
}

GestureSmoother::GestureSmoother(std::size_t window) : window_(std::max<std::size_t>(1, window)) {}

Gesture GestureSmoother::push(Gesture g) {
  history_.push_back(g);
  while (history_.size() > window_) history_.pop_front();

  std::array<std::size_t, kGestureNames.size()> counts{};
  for (Gesture h : history_) ++counts[static_cast<std::size_t>(h)];
  const auto best = std::max_element(counts.begin(), counts.end());
  if (*best * 2 > history_.size()) {
    return static_cast<Gesture>(std::distance(counts.begin(), best));
  }
  return Gesture::None;
}

void GestureSmoother::reset() { history_.clear(); }

}  // namespace ht
