// Runs SystemControl calls on a worker thread.
//
// Some OS calls are slow: setting laptop brightness through WMI can take
// 50-150 ms. v1 made those calls inline on every frame, freezing the camera
// loop. This decorator queues commands for a background thread and coalesces
// continuous ones (volume, brightness, cursor) so only the latest value is applied.
#pragma once

#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "handtracker/core/system_control.hpp"

namespace ht {

class AsyncSystemControl final : public SystemControl {
 public:
  explicit AsyncSystemControl(std::unique_ptr<SystemControl> inner);
  ~AsyncSystemControl() override;

  AsyncSystemControl(const AsyncSystemControl&) = delete;
  AsyncSystemControl& operator=(const AsyncSystemControl&) = delete;

  bool setVolume(float level) override;
  bool setBrightness(float level) override;
  bool sendKeys(const Hotkey& keys) override;
  bool launch(const std::string& target) override;
  bool moveCursor(float x, float y) override;
  bool setMouseButton(bool pressed) override;
  /// Synchronous: callers need the answer before deciding what to do.
  bool isProcessRunning(const std::string& exeName) override;
  std::string name() const override;

  /// Blocks until every queued command has been executed.
  void flush();

 private:
  struct Command {
    enum class Kind { Volume, Brightness, Keys, Launch, Cursor, Button };
    explicit Command(Kind k) : kind(k) {}
    Kind kind;
    float a = 0.f;
    float b = 0.f;
    bool flag = false;
    Hotkey keys;
    std::string text;
  };

  void enqueue(Command cmd, bool coalesce);
  void run();
  void execute(const Command& cmd);

  std::unique_ptr<SystemControl> inner_;
  std::string name_;
  std::mutex innerMutex_;
  std::mutex queueMutex_;
  std::condition_variable wake_;
  std::condition_variable idle_;
  std::deque<Command> queue_;
  bool busy_ = false;
  bool stop_ = false;
  std::thread worker_;  // Declared last so everything above exists before it starts.
};

}  // namespace ht
