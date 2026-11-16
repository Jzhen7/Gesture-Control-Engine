#include "handtracker/core/actions.hpp"

#include <algorithm>
#include <array>
#include <cctype>

namespace ht {
namespace {

std::string lower(std::string_view s) {
  std::string out(s);
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return out;
}

std::string_view trim(std::string_view s) {
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.remove_prefix(1);
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.remove_suffix(1);
  return s;
}

constexpr std::array<std::string_view, 30> kNamedKeys{{
    "ctrl", "alt", "shift", "win",
    "tab", "enter", "esc", "space", "backspace", "delete", "insert",
    "home", "end", "pageup", "pagedown", "left", "right", "up", "down", "printscreen",
    "media_next", "media_prev", "media_playpause", "media_stop",
    "volume_mute", "volume_up", "volume_down",
    "browser_back", "browser_forward", "browser_refresh",
}};

/// Short names accepted after "media:".
std::optional<std::string> mediaKeyName(std::string_view name) {
  const std::string n = lower(name);
  if (n == "next") return "media_next";
  if (n == "prev" || n == "previous") return "media_prev";
  if (n == "play" || n == "pause" || n == "playpause") return "media_playpause";
  if (n == "stop") return "media_stop";
  if (n == "mute") return "volume_mute";
  if (n == "volup" || n == "volume_up") return "volume_up";
  if (n == "voldown" || n == "volume_down") return "volume_down";
  return std::nullopt;
}

}  // namespace

std::string_view toString(Mode m) {
  switch (m) {
    case Mode::Idle: return "idle";
    case Mode::Brightness: return "brightness";
    case Mode::Volume: return "volume";
    case Mode::Depth: return "depth";
  }
  return "?";
}

std::optional<Mode> modeFromString(std::string_view name) {
  const std::string n = lower(name);
  if (n == "idle") return Mode::Idle;
  if (n == "brightness") return Mode::Brightness;
  if (n == "volume") return Mode::Volume;
  if (n == "depth") return Mode::Depth;
  return std::nullopt;
}

bool isKnownKeyName(std::string_view key) {
  if (key.size() == 1) {
    const char c = key[0];
    return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9');
  }
  if (key.size() >= 2 && key.size() <= 3 && key[0] == 'f') {
    int n = 0;
    for (char c : key.substr(1)) {
      if (c < '0' || c > '9') return false;
      n = n * 10 + (c - '0');
    }
    return n >= 1 && n <= 24;
  }
  return std::find(kNamedKeys.begin(), kNamedKeys.end(), key) != kNamedKeys.end();
}

std::string Hotkey::toString() const {
  std::string out;
  for (std::size_t i = 0; i < keys.size(); ++i) {
    if (i) out += '+';
    out += keys[i];
  }
  return out;
}

std::optional<Hotkey> parseHotkey(std::string_view text) {
  Hotkey hk;
  std::string_view rest = trim(text);
  while (!rest.empty()) {
    const auto plus = rest.find('+');
    std::string key = lower(trim(rest.substr(0, plus)));
    if (key == "control") key = "ctrl";
    if (key == "escape") key = "esc";
    if (key == "return") key = "enter";
    if (key == "super" || key == "meta" || key == "cmd") key = "win";
    if (!isKnownKeyName(key)) return std::nullopt;
    hk.keys.push_back(key);
    if (plus == std::string_view::npos) break;
    rest = rest.substr(plus + 1);
    if (rest.empty()) return std::nullopt;  // Trailing '+'.
  }
  if (hk.keys.empty()) return std::nullopt;
  return hk;
}

std::string Action::describe() const {
  switch (kind) {
    case Kind::Keys: return "keys " + hotkey.toString();
    case Kind::EnterMode: return std::string("enter ") + std::string(toString(mode)) + " mode";
    case Kind::Launch: return "launch " + target;
    case Kind::Quit: return "quit";
  }
  return "?";
}

std::optional<Action> parseAction(std::string_view text) {
  text = trim(text);
  const std::string whole = lower(text);
  if (whole == "quit" || whole == "exit") {
    Action a;
    a.kind = Action::Kind::Quit;
    return a;
  }

  const auto colon = text.find(':');
  if (colon == std::string_view::npos) return std::nullopt;
  const std::string kind = lower(trim(text.substr(0, colon)));
  const std::string_view arg = trim(text.substr(colon + 1));
  if (arg.empty()) return std::nullopt;

  Action a;
  if (kind == "keys" || kind == "key" || kind == "hotkey") {
    auto hk = parseHotkey(arg);
    if (!hk) return std::nullopt;
    a.kind = Action::Kind::Keys;
    a.hotkey = *hk;
    return a;
  }
  if (kind == "media") {
    auto key = mediaKeyName(arg);
    if (!key) return std::nullopt;
    a.kind = Action::Kind::Keys;
    a.hotkey.keys = {*key};
    return a;
  }
  if (kind == "mode") {
    auto m = modeFromString(arg);
    if (!m || *m == Mode::Idle) return std::nullopt;
    a.kind = Action::Kind::EnterMode;
    a.mode = *m;
    return a;
  }
  if (kind == "launch" || kind == "open") {
    a.kind = Action::Kind::Launch;
    a.target = std::string(arg);  // Case preserved: paths/URLs may be case-sensitive.
    return a;
  }
  return std::nullopt;
}

}  // namespace ht
