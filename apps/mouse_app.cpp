#include <cstdio>

#include "app.hpp"
#include "handtracker/core/mouse_controller.hpp"
#include "handtracker/ui/overlay.hpp"

namespace ht::app {
namespace {

class MouseApp final : public App {
 public:
  MouseApp(const AppConfig& cfg, SystemControl& system)
      : margin_(cfg.mouse.margin), mouse_(cfg.mouse, system) {}

  void update(const std::optional<HandLandmarks>& hand, double tMs) override {
    mouse_.update(hand, tMs);
  }

  void draw(cv::Mat& frame, const std::optional<HandLandmarks>& hand, double) override {
    ui::drawActiveRegion(frame, margin_);
    if (hand) ui::drawHand(frame, *hand);
    const auto& s = mouse_.status();
    char pos[64];
    std::snprintf(pos, sizeof(pos), "cursor: %.2f, %.2f", s.x, s.y);
    char pinch[64];
    std::snprintf(pinch, sizeof(pinch), "pinch: %.2f %s", s.pinch, s.pressed ? "(down)" : "");
    ui::drawTextPanel(frame, {"mouse mode", s.tracking ? pos : "cursor: (no hand)", pinch,
                              "pinch thumb + index to click/drag"});
  }

  void shutdown() override { mouse_.release(); }

 private:
  float margin_;
  MouseController mouse_;
};

}  // namespace

std::unique_ptr<App> makeMouseApp(const AppConfig& cfg, SystemControl& system) {
  return std::make_unique<MouseApp>(cfg, system);
}

}  // namespace ht::app
