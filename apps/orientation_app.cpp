#include <cstdio>

#include "app.hpp"
#include "handtracker/core/orientation.hpp"
#include "handtracker/ui/overlay.hpp"

namespace ht::app {
namespace {

float toUnit(float degrees) { return (degrees + 180.f) / 360.f; }

class OrientationApp final : public App {
 public:
  void update(const std::optional<HandLandmarks>& hand, double) override {
    if (hand) orientation_ = computeOrientation(*hand);
    visible_ = hand.has_value();
  }

  void draw(cv::Mat& frame, const std::optional<HandLandmarks>& hand, double) override {
    if (hand) ui::drawHand(frame, *hand);
    if (!visible_) {
      ui::drawTextPanel(frame, {"orientation", "show your hand to the camera"});
      return;
    }
    const auto& o = orientation_;
    ui::drawLevelBar(frame, "rot z", toUnit(o.rotZ), 0);
    ui::drawLevelBar(frame, "rot y", toUnit(o.rotY), 1);
    ui::drawLevelBar(frame, "rot x", toUnit(o.rotX), 2);
    char a[64], b[64], c[64], d[64];
    std::snprintf(a, sizeof(a), "rot x: %6.1f deg", o.rotX);
    std::snprintf(b, sizeof(b), "rot y: %6.1f deg", o.rotY);
    std::snprintf(c, sizeof(c), "rot z: %6.1f deg", o.rotZ);
    std::snprintf(d, sizeof(d), "twist: %6.1f deg", o.twist);
    ui::drawTextPanel(frame, {"orientation", a, b, c, d});
  }

 private:
  HandOrientation orientation_;
  bool visible_ = false;
};

}  // namespace

std::unique_ptr<App> makeOrientationApp(const AppConfig&) {
  return std::make_unique<OrientationApp>();
}

}  // namespace ht::app
