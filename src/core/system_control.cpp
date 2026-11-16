#include "handtracker/core/system_control.hpp"

#include <iomanip>

namespace ht {

DryRunSystemControl::DryRunSystemControl(std::ostream& out, bool assumeProcessesRunning)
    : out_(out), assumeRunning_(assumeProcessesRunning) {}

bool DryRunSystemControl::setVolume(float level) {
  out_ << "[dry-run] volume -> " << std::fixed << std::setprecision(0) << level * 100.f << "%\n";
  return true;
}

bool DryRunSystemControl::setBrightness(float level) {
  out_ << "[dry-run] brightness -> " << std::fixed << std::setprecision(0) << level * 100.f
       << "%\n";
  return true;
}

bool DryRunSystemControl::sendKeys(const Hotkey& keys) {
  out_ << "[dry-run] keys " << keys.toString() << "\n";
  return true;
}

bool DryRunSystemControl::launch(const std::string& target) {
  out_ << "[dry-run] launch " << target << "\n";
  return true;
}

bool DryRunSystemControl::moveCursor(float, float) { return true; }  // Too chatty to log.

bool DryRunSystemControl::setMouseButton(bool pressed) {
  out_ << "[dry-run] mouse " << (pressed ? "down" : "up") << "\n";
  return true;
}

bool DryRunSystemControl::isProcessRunning(const std::string&) { return assumeRunning_; }

}  // namespace ht
