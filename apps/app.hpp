// Application-level plumbing shared by the three app modes:
//   control      gesture -> OS actions (v1: hand_tracker_functional.py)
//   mouse        hand -> cursor        (v1: mouse_mirroring.py)
//   orientation  3D hand rotation view (v1: hand_tracker_bare.py)
#pragma once

#include <memory>
#include <optional>
#include <string>

#include <opencv2/core.hpp>

#include "handtracker/core/config.hpp"
#include "handtracker/core/landmarks.hpp"
#include "handtracker/core/system_control.hpp"
#include "handtracker/vision/hand_tracker.hpp"

namespace ht::app {

class App {
 public:
  virtual ~App() = default;
  virtual void update(const std::optional<HandLandmarks>& hand, double tMs) = 0;
  virtual void draw(cv::Mat& frame, const std::optional<HandLandmarks>& hand, double tMs) = 0;
  virtual bool quitRequested() const { return false; }
  virtual void shutdown() {}
};

std::unique_ptr<App> makeControlApp(const AppConfig& cfg, SystemControl& system);
std::unique_ptr<App> makeMouseApp(const AppConfig& cfg, SystemControl& system);
std::unique_ptr<App> makeOrientationApp(const AppConfig& cfg);

/// Live camera loop. Returns a process exit code.
int runCamera(App& app, vision::HandTracker& tracker, const AppConfig& cfg,
              const std::string& windowTitle);

/// Runs detection on a still image and prints what it finds. Used for smoke
/// tests and debugging. Returns 3 if fewer than `expectHands` hands are found,
/// 4 if `expectGesture` is set and the first hand shows a different gesture.
int runImage(vision::HandTracker& tracker, const AppConfig& cfg, const std::string& inputPath,
             const std::string& outputPath, int expectHands, const std::string& expectGesture);

}  // namespace ht::app
