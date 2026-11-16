// Hand-driven mouse (port and upgrade of v1's mouse_mirroring.py).
//
// v1 mapped the raw knuckle position to the screen with hard-coded camera
// offsets, so the cursor jittered constantly and stopped at the frame margin.
// v2 filters the position with a One Euro filter, clamps at the edges, and
// clicks with a pinch (with hysteresis) instead of an open palm.
#pragma once

#include <optional>

#include "handtracker/core/config.hpp"
#include "handtracker/core/filters.hpp"
#include "handtracker/core/landmarks.hpp"
#include "handtracker/core/system_control.hpp"

namespace ht {

/// Maps a normalised camera point inside the active region to [0, 1] screen space.
float mapActiveRegion(float v, float margin);

class MouseController {
 public:
  struct Status {
    bool tracking = false;
    bool pressed = false;
    float x = 0.f;
    float y = 0.f;
    float pinch = 0.f;
  };

  MouseController(const MouseConfig& cfg, SystemControl& system);

  void update(const std::optional<HandLandmarks>& hand, double tMs);
  const Status& status() const { return status_; }
  /// Releases the button if it is held (call before shutting down).
  void release();

 private:
  MouseConfig cfg_;
  SystemControl& system_;
  OneEuroFilter fx_;
  OneEuroFilter fy_;
  Status status_;
};

}  // namespace ht
