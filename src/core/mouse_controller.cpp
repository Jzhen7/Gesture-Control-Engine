#include "handtracker/core/mouse_controller.hpp"

#include <algorithm>

#include "handtracker/core/mapping.hpp"

namespace ht {

float mapActiveRegion(float v, float margin) {
  margin = std::clamp(margin, 0.f, 0.45f);
  return std::clamp((v - margin) / (1.f - 2.f * margin), 0.f, 1.f);
}

MouseController::MouseController(const MouseConfig& cfg, SystemControl& system)
    : cfg_(cfg),
      system_(system),
      fx_(cfg.minCutoff, cfg.beta),
      fy_(cfg.minCutoff, cfg.beta) {}

void MouseController::release() {
  if (status_.pressed) {
    system_.setMouseButton(false);
    status_.pressed = false;
  }
}

void MouseController::update(const std::optional<HandLandmarks>& hand, double tMs) {
  if (!hand) {
    release();  // Never leave the button stuck down when tracking drops.
    status_.tracking = false;
    fx_.reset();
    fy_.reset();
    return;
  }

  const Vec3 p = hand->normalized(LandmarkId::MiddleMcp);
  const double t = tMs / 1000.0;
  status_.x = static_cast<float>(fx_.filter(mapActiveRegion(p.x, cfg_.margin), t));
  status_.y = static_cast<float>(fy_.filter(mapActiveRegion(p.y, cfg_.margin), t));
  status_.tracking = true;
  system_.moveCursor(status_.x, status_.y);

  status_.pinch = pinchRatio(*hand);
  if (!status_.pressed && status_.pinch < cfg_.pinchOn) {
    status_.pressed = true;
    system_.setMouseButton(true);
  } else if (status_.pressed && status_.pinch > cfg_.pinchOff) {
    status_.pressed = false;
    system_.setMouseButton(false);
  }
}

}  // namespace ht
