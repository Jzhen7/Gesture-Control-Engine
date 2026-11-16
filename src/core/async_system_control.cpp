#include "handtracker/core/async_system_control.hpp"

#include <utility>

namespace ht {

AsyncSystemControl::AsyncSystemControl(std::unique_ptr<SystemControl> inner)
    : inner_(std::move(inner)), name_(inner_->name() + " (async)"), worker_([this] { run(); }) {}

AsyncSystemControl::~AsyncSystemControl() {
  {
    std::lock_guard<std::mutex> lock(queueMutex_);
    stop_ = true;
  }
  wake_.notify_all();
  if (worker_.joinable()) worker_.join();
}

void AsyncSystemControl::enqueue(Command cmd, bool coalesce) {
  {
    std::lock_guard<std::mutex> lock(queueMutex_);
    bool merged = false;
    if (coalesce) {
      for (Command& pending : queue_) {
        if (pending.kind == cmd.kind) {
          pending = std::move(cmd);  // Keep its queue position, take the newest value.
          merged = true;
          break;
        }
      }
    }
    if (!merged) queue_.push_back(std::move(cmd));
  }
  wake_.notify_one();
}

void AsyncSystemControl::run() {
  std::unique_lock<std::mutex> lock(queueMutex_);
  for (;;) {
    wake_.wait(lock, [this] { return stop_ || !queue_.empty(); });
    if (queue_.empty()) {
      if (stop_) {
        // Destroy the backend on this thread: OS handles (e.g. COM objects)
        // it created here must also be released here.
        std::lock_guard<std::mutex> innerLock(innerMutex_);
        inner_.reset();
        return;
      }
      continue;
    }
    Command cmd = std::move(queue_.front());
    queue_.pop_front();
    busy_ = true;
    lock.unlock();
    execute(cmd);
    lock.lock();
    busy_ = false;
    if (queue_.empty()) idle_.notify_all();
  }
}

void AsyncSystemControl::execute(const Command& cmd) {
  std::lock_guard<std::mutex> lock(innerMutex_);
  if (!inner_) return;
  switch (cmd.kind) {
    case Command::Kind::Volume: inner_->setVolume(cmd.a); break;
    case Command::Kind::Brightness: inner_->setBrightness(cmd.a); break;
    case Command::Kind::Keys: inner_->sendKeys(cmd.keys); break;
    case Command::Kind::Launch: inner_->launch(cmd.text); break;
    case Command::Kind::Cursor: inner_->moveCursor(cmd.a, cmd.b); break;
    case Command::Kind::Button: inner_->setMouseButton(cmd.flag); break;
  }
}

void AsyncSystemControl::flush() {
  std::unique_lock<std::mutex> lock(queueMutex_);
  idle_.wait(lock, [this] { return queue_.empty() && !busy_; });
}

bool AsyncSystemControl::setVolume(float level) {
  Command c{Command::Kind::Volume};
  c.a = level;
  enqueue(std::move(c), true);
  return true;
}

bool AsyncSystemControl::setBrightness(float level) {
  Command c{Command::Kind::Brightness};
  c.a = level;
  enqueue(std::move(c), true);
  return true;
}

bool AsyncSystemControl::sendKeys(const Hotkey& keys) {
  Command c{Command::Kind::Keys};
  c.keys = keys;
  enqueue(std::move(c), false);
  return true;
}

bool AsyncSystemControl::launch(const std::string& target) {
  Command c{Command::Kind::Launch};
  c.text = target;
  enqueue(std::move(c), false);
  return true;
}

bool AsyncSystemControl::moveCursor(float x, float y) {
  Command c{Command::Kind::Cursor};
  c.a = x;
  c.b = y;
  enqueue(std::move(c), true);
  return true;
}

bool AsyncSystemControl::setMouseButton(bool pressed) {
  Command c{Command::Kind::Button};
  c.flag = pressed;
  enqueue(std::move(c), false);
  return true;
}

bool AsyncSystemControl::isProcessRunning(const std::string& exeName) {
  std::lock_guard<std::mutex> lock(innerMutex_);
  return inner_ && inner_->isProcessRunning(exeName);
}

std::string AsyncSystemControl::name() const { return name_; }

}  // namespace ht
