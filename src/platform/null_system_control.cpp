// Fallback backend for platforms without native integration yet (Linux, macOS).
// Every action is logged so the vision pipeline and gesture logic can still be
// developed and demonstrated. See docs/LIMITATIONS_AND_ROADMAP.md for the plan
// to add PulseAudio/PipeWire, ddcutil/backlight and XTest/uinput backends.
#include <memory>
#include <ostream>

#include "handtracker/core/system_control.hpp"

namespace ht {

std::unique_ptr<SystemControl> createPlatformSystemControl(std::ostream& log) {
  log << "note: no native system-control backend for this platform; actions are logged only\n";
  return std::make_unique<DryRunSystemControl>(log, /*assumeProcessesRunning=*/true);
}

void platformInit() {}

bool makeWindowTopmost(const std::string&) { return false; }

}  // namespace ht
