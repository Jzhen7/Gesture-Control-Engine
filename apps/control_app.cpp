#include <iostream>

#include "app.hpp"
#include "handtracker/core/controller.hpp"
#include "handtracker/ui/overlay.hpp"

namespace ht::app {
namespace {

constexpr double kToastMs = 1800.0;

class ControlApp final : public App {
 public:
  ControlApp(const AppConfig& cfg, SystemControl& system)
      : controller_(cfg, system, [](const std::string& msg) { std::cout << msg << std::endl; }) {}

  void update(const std::optional<HandLandmarks>& hand, double tMs) override {
    controller_.update(hand, tMs);
  }

  void draw(cv::Mat& frame, const std::optional<HandLandmarks>& hand, double tMs) override {
    const auto& s = controller_.status();
    if (hand) {
      ui::drawHand(frame, *hand);
      ui::drawHoldRing(frame, *hand, s.holdProgress);
    }

    std::vector<std::string> lines;
    lines.push_back(std::string("mode: ") + std::string(toString(s.mode)));
    lines.push_back(std::string("gesture: ") +
                    (s.handVisible ? std::string(toString(s.gesture)) : std::string("(no hand)")));
    if (s.mode != Mode::Idle) lines.emplace_back("open palm to exit");
    ui::drawTextPanel(frame, lines);

    if (s.mode != Mode::Idle && s.level) ui::drawLevelBar(frame, std::string(toString(s.mode)), *s.level);

    const double age = tMs - s.lastMessageT;
    if (age >= 0.0 && age < kToastMs) {
      ui::drawToast(frame, s.lastMessage, static_cast<float>(1.0 - age / kToastMs));
    }
  }

  bool quitRequested() const override { return controller_.quitRequested(); }

 private:
  GestureController controller_;
};

}  // namespace

std::unique_ptr<App> makeControlApp(const AppConfig& cfg, SystemControl& system) {
  return std::make_unique<ControlApp>(cfg, system);
}

}  // namespace ht::app
