// Abstraction over everything the app does to the operating system.
//
// v1 called pycaw, screen_brightness_control, pyautogui and psutil directly from
// the main loop, which tied it to Windows and made it impossible to test. In v2
// the gesture logic only talks to this interface; the platform layer provides
// the real implementation and tests provide a recording fake.
#pragma once

#include <memory>
#include <ostream>
#include <string>

#include "handtracker/core/actions.hpp"

namespace ht {

class SystemControl {
 public:
  virtual ~SystemControl() = default;

  virtual bool setVolume(float level) = 0;      ///< Master output volume, 0..1.
  virtual bool setBrightness(float level) = 0;  ///< Display brightness, 0..1.
  virtual bool sendKeys(const Hotkey& keys) = 0;
  virtual bool launch(const std::string& target) = 0;
  virtual bool moveCursor(float x, float y) = 0;  ///< Normalised primary-screen coordinates.
  virtual bool setMouseButton(bool pressed) = 0;  ///< Left button.
  virtual bool isProcessRunning(const std::string& exeName) = 0;
  virtual std::string name() const = 0;
};

/// Logs every call instead of touching the OS. Used by --dry-run and as the
/// fallback on platforms without a native backend.
class DryRunSystemControl final : public SystemControl {
 public:
  explicit DryRunSystemControl(std::ostream& out, bool assumeProcessesRunning = true);

  bool setVolume(float level) override;
  bool setBrightness(float level) override;
  bool sendKeys(const Hotkey& keys) override;
  bool launch(const std::string& target) override;
  bool moveCursor(float x, float y) override;
  bool setMouseButton(bool pressed) override;
  bool isProcessRunning(const std::string& exeName) override;
  std::string name() const override { return "dry-run"; }

 private:
  std::ostream& out_;
  bool assumeRunning_;
};

/// Implemented by the platform library (win32 or null backend).
std::unique_ptr<SystemControl> createPlatformSystemControl(std::ostream& log);

/// One-time process setup (e.g. DPI awareness on Windows). Safe to call on any platform.
void platformInit();

/// Pins a window above all others. Returns false if unsupported or not found.
bool makeWindowTopmost(const std::string& windowTitle);

}  // namespace ht
