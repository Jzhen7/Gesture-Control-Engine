// The gesture controller: turns a stream of hand observations into OS actions.
//
// This is the heart of v1's hand_tracker_functional.py, restructured as an
// explicit state machine (Idle -> Brightness / Volume / Depth -> Idle) driven by
// data-defined bindings instead of an if/elif chain.
#pragma once

#include <functional>
#include <optional>
#include <string>

#include "handtracker/core/config.hpp"
#include "handtracker/core/filters.hpp"
#include "handtracker/core/gesture.hpp"
#include "handtracker/core/landmarks.hpp"
#include "handtracker/core/system_control.hpp"
#include "handtracker/core/temporal.hpp"

namespace ht {

struct ControllerStatus {
  bool handVisible = false;
  Gesture rawGesture = Gesture::None;
  Gesture gesture = Gesture::None;  ///< After smoothing.
  Mode mode = Mode::Idle;
  std::optional<float> level;  ///< Current value of the active mode, 0..1.
  float holdProgress = 0.f;
  Gesture holdGesture = Gesture::None;
  std::string lastMessage;
  double lastMessageT = -1e300;
};

class GestureController {
 public:
  using MessageSink = std::function<void(const std::string&)>;

  GestureController(const AppConfig& cfg, SystemControl& system, MessageSink sink = {});

  /// Feed one frame. `hand` is nullopt when no hand is visible.
  void update(const std::optional<HandLandmarks>& hand, double tMs);

  const ControllerStatus& status() const { return status_; }
  bool quitRequested() const { return quit_; }
  Mode mode() const { return status_.mode; }

 private:
  void updateMode(Gesture g, const HandLandmarks& hand, double tMs);
  void dispatch(const GestureEvent& ev, double tMs);
  void execute(const Action& action, double tMs);
  void enterMode(Mode m, double tMs);
  void exitMode(const std::string& reason, double tMs);
  void message(const std::string& text, double tMs);

  AppConfig cfg_;
  SystemControl& system_;
  MessageSink sink_;
  GestureSmoother smoother_;
  EventDetector detector_;
  LevelFilter volumeFilter_;
  LevelFilter brightnessFilter_;
  ControllerStatus status_;
  double lastHandT_ = -1.0;
  bool quit_ = false;
};

}  // namespace ht
