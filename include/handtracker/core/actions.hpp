// Actions that gestures can trigger, and the bindings that connect them.
#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "handtracker/core/gesture.hpp"
#include "handtracker/core/temporal.hpp"

namespace ht {

/// Continuous-control modes. While a mode is active, the hand drives a value
/// (brightness, volume, ...) instead of producing discrete events.
enum class Mode { Idle, Brightness, Volume, Depth };

std::string_view toString(Mode m);
std::optional<Mode> modeFromString(std::string_view name);

/// A key chord such as ctrl+t or alt+tab, or a single media key. Key names are
/// lowercase and platform-neutral; see isKnownKeyName() for the full list.
struct Hotkey {
  std::vector<std::string> keys;
  std::string toString() const;
  bool operator==(const Hotkey& o) const { return keys == o.keys; }
};

bool isKnownKeyName(std::string_view key);
/// Parses "ctrl+shift+t". Returns nullopt on unknown keys or an empty chord.
std::optional<Hotkey> parseHotkey(std::string_view text);

struct Action {
  enum class Kind { Keys, EnterMode, Launch, Quit };

  Kind kind = Kind::Keys;
  Hotkey hotkey;       ///< Kind::Keys
  Mode mode = Mode::Idle;  ///< Kind::EnterMode
  std::string target;  ///< Kind::Launch: program, file or URL

  std::string describe() const;
};

/// Parses an action spec:
///   keys:ctrl+t      media:next      mode:volume      launch:notepad.exe      quit
std::optional<Action> parseAction(std::string_view text);

struct Binding {
  Gesture gesture = Gesture::None;
  EventType event = EventType::Hold;
  Action action;
  /// If non-empty, the binding only fires while this process is running (e.g. spotify.exe).
  std::string requireProcess;
};

}  // namespace ht
