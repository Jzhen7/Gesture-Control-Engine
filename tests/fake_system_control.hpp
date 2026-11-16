#pragma once

#include <set>
#include <string>
#include <vector>

#include "handtracker/core/system_control.hpp"

namespace testutil {

/// Records every call so tests can assert on what the app *would* have done.
class FakeSystemControl final : public ht::SystemControl {
 public:
  std::vector<float> volumes;
  std::vector<float> brightness;
  std::vector<std::string> keys;
  std::vector<std::string> launches;
  std::vector<bool> buttons;
  int cursorMoves = 0;
  float lastX = 0.f;
  float lastY = 0.f;
  std::set<std::string> running;

  bool setVolume(float v) override { volumes.push_back(v); return true; }
  bool setBrightness(float v) override { brightness.push_back(v); return true; }
  bool sendKeys(const ht::Hotkey& k) override { keys.push_back(k.toString()); return true; }
  bool launch(const std::string& t) override { launches.push_back(t); return true; }
  bool moveCursor(float x, float y) override {
    ++cursorMoves;
    lastX = x;
    lastY = y;
    return true;
  }
  bool setMouseButton(bool p) override { buttons.push_back(p); return true; }
  bool isProcessRunning(const std::string& n) override { return running.count(n) > 0; }
  std::string name() const override { return "fake"; }
};

}  // namespace testutil
