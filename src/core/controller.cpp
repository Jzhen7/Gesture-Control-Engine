#include "handtracker/core/controller.hpp"

#include <cmath>
#include <utility>

#include "handtracker/core/mapping.hpp"

namespace ht {

GestureController::GestureController(const AppConfig& cfg, SystemControl& system, MessageSink sink)
    : cfg_(cfg),
      system_(system),
      sink_(std::move(sink)),
      smoother_(cfg.smoothingFrames),
      detector_(cfg.temporal),
      volumeFilter_(cfg.modes.volumeFilter),
      brightnessFilter_(cfg.modes.brightnessFilter) {}

void GestureController::message(const std::string& text, double tMs) {
  status_.lastMessage = text;
  status_.lastMessageT = tMs;
  if (sink_) sink_(text);
}

void GestureController::update(const std::optional<HandLandmarks>& hand, double tMs) {
  if (!hand) {
    status_.handVisible = false;
    status_.rawGesture = Gesture::None;
    status_.gesture = Gesture::None;
    status_.holdProgress = 0.f;
    status_.holdGesture = Gesture::None;
    detector_.reset();
    smoother_.reset();
    // v1 stayed in a mode forever if the hand left the frame. Time out instead.
    if (status_.mode != Mode::Idle && lastHandT_ >= 0.0 &&
        tMs - lastHandT_ > cfg_.modes.handLostTimeoutMs) {
      exitMode("hand lost", tMs);
    }
    return;
  }

  lastHandT_ = tMs;
  status_.handVisible = true;
  status_.rawGesture = classifyGesture(*hand, cfg_.classifier);
  const Gesture g = smoother_.push(status_.rawGesture);
  status_.gesture = g;

  if (status_.mode != Mode::Idle) {
    status_.holdProgress = 0.f;
    updateMode(g, *hand, tMs);
    return;
  }

  const auto ev = detector_.update(g, *hand, tMs);
  status_.holdProgress = detector_.holdProgress(tMs);
  status_.holdGesture = detector_.holdGesture();
  if (ev) dispatch(*ev, tMs);
}

void GestureController::updateMode(Gesture g, const HandLandmarks& hand, double tMs) {
  const ModeConfig& m = cfg_.modes;
  if (g == m.exitGesture) {
    exitMode("exit gesture", tMs);
    return;
  }

  switch (status_.mode) {
    case Mode::Brightness:
      if (g == m.brightnessGesture) {
        const float raw = brightnessFromHand(hand, m.brightnessTop, m.brightnessBottom);
        if (auto v = brightnessFilter_.update(raw, tMs)) system_.setBrightness(*v);
        status_.level = brightnessFilter_.value();
      }
      break;
    case Mode::Volume:
      if (g == m.volumeGesture) {
        const float raw = volumeFromRoll(handRollDegrees(hand), m.volumeMinAngle, m.volumeMaxAngle);
        if (auto v = volumeFilter_.update(raw, tMs)) system_.setVolume(*v);
        status_.level = volumeFilter_.value();
      }
      break;
    case Mode::Depth:
      // Read-only for now: shows how close the hand is. See docs/LIMITATIONS_AND_ROADMAP.md.
      if (g == m.depthGesture) status_.level = mapToUnit(handDepthProxy(hand), 0.05f, 0.35f);
      break;
    case Mode::Idle:
      break;
  }
}

void GestureController::dispatch(const GestureEvent& ev, double tMs) {
  const std::string what =
      std::string(toString(ev.gesture)) + " " + std::string(toString(ev.type));
  for (const Binding& b : cfg_.bindings) {
    if (b.gesture != ev.gesture || b.event != ev.type) continue;
    if (!b.requireProcess.empty() && !system_.isProcessRunning(b.requireProcess)) {
      message(what + ": skipped (" + b.requireProcess + " not running)", tMs);
      return;
    }
    message(what + " -> " + b.action.describe(), tMs);
    execute(b.action, tMs);
    return;
  }
  message(what, tMs);  // Unbound, but still useful feedback.
}

void GestureController::execute(const Action& action, double tMs) {
  switch (action.kind) {
    case Action::Kind::Keys: system_.sendKeys(action.hotkey); break;
    case Action::Kind::Launch: system_.launch(action.target); break;
    case Action::Kind::EnterMode: enterMode(action.mode, tMs); break;
    case Action::Kind::Quit: quit_ = true; break;
  }
}

void GestureController::enterMode(Mode m, double tMs) {
  status_.mode = m;
  status_.level.reset();
  volumeFilter_.reset();
  brightnessFilter_.reset();
  detector_.reset();
  smoother_.reset();  // Don't let the entry gesture leak into the mode.
  message(std::string("entered ") + std::string(toString(m)) + " mode", tMs);
}

void GestureController::exitMode(const std::string& reason, double tMs) {
  const Mode old = status_.mode;
  status_.mode = Mode::Idle;
  status_.level.reset();
  detector_.reset();
  smoother_.reset();
  message(std::string("left ") + std::string(toString(old)) + " mode (" + reason + ")", tMs);
}

}  // namespace ht
