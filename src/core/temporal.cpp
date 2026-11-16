#include "handtracker/core/temporal.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace ht {
namespace {

struct EventName {
  EventType type;
  std::string_view name;
};

constexpr std::array<EventName, 6> kEventNames{{
    {EventType::SwipeLeft, "swipe_left"},
    {EventType::SwipeRight, "swipe_right"},
    {EventType::SwipeUp, "swipe_up"},
    {EventType::SwipeDown, "swipe_down"},
    {EventType::Flip, "flip"},
    {EventType::Hold, "hold"},
}};

}  // namespace

std::string_view toString(EventType e) {
  for (const auto& entry : kEventNames) {
    if (entry.type == e) return entry.name;
  }
  return "?";
}

std::optional<EventType> eventFromString(std::string_view name) {
  for (const auto& entry : kEventNames) {
    if (entry.name == name) return entry.type;
  }
  return std::nullopt;
}

EventDetector::EventDetector(TemporalConfig cfg) : cfg_(cfg) {}

void EventDetector::reset() {
  samples_.clear();
  resetHold();
}

void EventDetector::resetHold() {
  holdGesture_ = Gesture::None;
  holdStart_ = 0.0;
  holdLatched_ = false;
}

float EventDetector::holdProgress(double tMs) const {
  if (holdGesture_ == Gesture::None || holdLatched_ || cfg_.holdMs <= 0.0) return 0.f;
  return static_cast<float>(std::clamp((tMs - holdStart_) / cfg_.holdMs, 0.0, 1.0));
}

std::optional<GestureEvent> EventDetector::emit(EventType type, Gesture g, double tMs) {
  samples_.clear();
  cooldownUntil_ = tMs + cfg_.cooldownMs;
  return GestureEvent{type, g};
}

std::optional<GestureEvent> EventDetector::update(Gesture gesture, const HandLandmarks& hand,
                                                  double tMs) {
  if (gesture == Gesture::None) {
    // Same as v1: an unrecognised pose breaks any motion in progress.
    reset();
    return std::nullopt;
  }

  const Vec3 wrist = hand.normalized(LandmarkId::Wrist);
  const float palm = std::max(hand.palmSize(), 1e-6f);
  const float separation = (hand[LandmarkId::IndexMcp].x - hand[LandmarkId::PinkyMcp].x) / palm;
  int side = 0;
  if (separation > cfg_.flipMinSeparation) side = 1;
  if (separation < -cfg_.flipMinSeparation) side = -1;

  samples_.push_back({tMs, wrist.x, wrist.y, side});
  while (!samples_.empty() && tMs - samples_.front().t > cfg_.swipeWindowMs) samples_.pop_front();

  // Hold tracking runs even during cooldown so the timer is accurate.
  if (gesture != holdGesture_) {
    holdGesture_ = gesture;
    holdStart_ = tMs;
    holdX_ = wrist.x;
    holdY_ = wrist.y;
    holdLatched_ = false;
  } else if (std::hypot(wrist.x - holdX_, wrist.y - holdY_) > cfg_.holdMaxMotion) {
    holdStart_ = tMs;  // Moving hands are not holding; restart the timer.
    holdX_ = wrist.x;
    holdY_ = wrist.y;
  }

  if (tMs < cooldownUntil_) return std::nullopt;

  // 1. Flip: the index/pinky knuckle order inverts within the window.
  int firstSide = 0;
  int lastSide = 0;
  for (const Sample& s : samples_) {
    if (s.side == 0) continue;
    if (firstSide == 0) firstSide = s.side;
    lastSide = s.side;
  }
  if (firstSide != 0 && lastSide != 0 && firstSide != lastSide) {
    holdLatched_ = true;  // Don't let the flip also count as a hold.
    return emit(EventType::Flip, gesture, tMs);
  }

  // 2. Swipe: net displacement across the window, classified by dominant axis.
  const Sample& first = samples_.front();
  const Sample& last = samples_.back();
  if (last.t - first.t >= cfg_.swipeWindowMs * 0.5) {
    const float dx = last.x - first.x;
    const float dy = last.y - first.y;
    if (std::max(std::fabs(dx), std::fabs(dy)) > cfg_.swipeDistance) {
      EventType type;
      if (std::fabs(dx) >= std::fabs(dy)) {
        type = dx > 0 ? EventType::SwipeRight : EventType::SwipeLeft;
      } else {
        type = dy > 0 ? EventType::SwipeDown : EventType::SwipeUp;
      }
      holdStart_ = tMs;
      holdX_ = wrist.x;
      holdY_ = wrist.y;
      return emit(type, gesture, tMs);
    }
  }

  // 3. Hold: same gesture, kept still, for holdMs. Fires once per hold.
  if (!holdLatched_ && tMs - holdStart_ >= cfg_.holdMs) {
    holdLatched_ = true;
    return emit(EventType::Hold, gesture, tMs);
  }
  return std::nullopt;
}

}  // namespace ht
