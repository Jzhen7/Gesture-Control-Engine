// `handtracker arm`: the robot-arm front-end.
//
// Each camera frame:  hand -> ArmController (target + gripper) -> MujocoArm
// (IK + physics for the real time that passed) -> draw camera | 3D arm view.
#include <cstdio>
#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "app.hpp"
#include "arm_view.hpp"
#include "handtracker/arm/arm_controller.hpp"
#include "handtracker/arm/mujoco_arm.hpp"
#include "handtracker/ui/overlay.hpp"

namespace ht::app {
namespace {

constexpr double kToastMs = 1800.0;

class ArmApp final : public App {
 public:
  explicit ArmApp(const AppConfig& cfg)
      : cfg_(cfg),
        arm_(cfg.arm),
        controller_(cfg, arm_, [this](const std::string& m) { onMessage(m); }),
        view_(cfg.camera.width, cfg.camera.height) {
    std::cout << "arm backend: " << arm_.name() << "\n"
              << "gripper: "
              << (cfg.arm.gripperMode == GripperMode::Binary
                      ? "binary (fist = closed, anything else = open)"
                      : "proportional (straight fingers: tips together = closed, spread = open)")
              << "\n"
              << "keys: c = recalibrate, a/d = rotate view, q = quit\n";
    controller_.recalibrate();
  }

  void update(const std::optional<HandLandmarks>& hand, double tMs) override {
    controller_.update(hand, tMs);
    const double dt = lastT_ < 0.0 ? 0.0 : (tMs - lastT_) / 1000.0;
    lastT_ = tMs;
    arm_.advance(dt);  // Simulate exactly the real time that passed.
  }

  void draw(cv::Mat& frame, const std::optional<HandLandmarks>& hand, double tMs) override {
    const auto& s = controller_.status();
    if (hand) {
      ui::drawHand(frame, *hand);
      if (s.phase == arm::Phase::Calibrating) ui::drawHoldRing(frame, *hand, s.calibrationProgress);
    }

    std::vector<std::string> lines;
    lines.emplace_back("robot arm");
    if (s.phase == arm::Phase::Calibrating) {
      lines.emplace_back("CALIBRATING: hold an open palm still");
      lines.emplace_back("where it feels comfortable (2 s)");
    } else {
      char buf[96];
      if (!s.handVisible) {
        lines.emplace_back("no hand: arm holds position and grip");
      } else if (cfg_.arm.gripperMode == GripperMode::Binary) {
        lines.emplace_back(s.gripperOpening < 0.5f ? "fist: gripper closed" : "open hand: gripper open");
      } else {
        if (s.clawPose) {
          std::snprintf(buf, sizeof(buf), "claw %3.0f%% open -> gripper %3.0f%%",
                        s.handOpenness * 100.f, s.gripperOpening * 100.f);
        } else {
          std::snprintf(buf, sizeof(buf), "fingers bent: gripper held at %3.0f%%",
                        s.gripperOpening * 100.f);
        }
        lines.emplace_back(buf);
      }
      std::snprintf(buf, sizeof(buf), "hand  x %+.2f  y %+.2f  depth %+.2f", s.handPosition.x,
                    s.handPosition.y, s.handPosition.z);
      lines.emplace_back(buf);
    }
    lines.emplace_back("c recalibrate | a/d rotate view | q quit");
    ui::drawTextPanel(frame, lines);

    const double age = tMs - messageT_;
    if (age >= 0.0 && age < kToastMs) {
      ui::drawToast(frame, message_, static_cast<float>(1.0 - age / kToastMs));
    }

    // Camera on the left, arm on the right, same height.
    cv::Mat armImg = view_.render(arm_.state(), controller_.workspace(), s);
    if (armImg.rows != frame.rows) {
      cv::Mat resized;
      cv::resize(armImg, resized, cv::Size(armImg.cols * frame.rows / armImg.rows, frame.rows));
      armImg = resized;
    }
    cv::Mat combined;
    cv::hconcat(frame, armImg, combined);
    frame = combined;
    lastDrawT_ = tMs;
  }

  void onKey(int key) override {
    if (key == 'c') controller_.recalibrate();
    if (key == 'a') view_.orbit(10.0);
    if (key == 'd') view_.orbit(-10.0);
  }

 private:
  void onMessage(const std::string& m) {
    std::cout << m << std::endl;
    message_ = m;
    messageT_ = lastDrawT_;
  }

  AppConfig cfg_;
  arm::MujocoArm arm_;  // Declared before controller_, which uses it.
  arm::ArmController controller_;
  ArmView view_;
  double lastT_ = -1.0;
  double lastDrawT_ = 0.0;
  std::string message_;
  double messageT_ = -1e300;
};

}  // namespace

std::unique_ptr<App> makeArmApp(const AppConfig& cfg) { return std::make_unique<ArmApp>(cfg); }

}  // namespace ht::app
