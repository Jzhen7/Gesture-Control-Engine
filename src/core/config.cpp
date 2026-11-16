#include "handtracker/core/config.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace ht {
namespace {

std::string_view trim(std::string_view s) {
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.remove_prefix(1);
  while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.remove_suffix(1);
  return s;
}

std::string lower(std::string_view s) {
  std::string out(s);
  std::transform(out.begin(), out.end(), out.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return out;
}

/// Strips a trailing "  # comment" (a '#' or ';' preceded by whitespace).
std::string_view stripComment(std::string_view s) {
  for (std::size_t i = 1; i < s.size(); ++i) {
    if ((s[i] == '#' || s[i] == ';') && std::isspace(static_cast<unsigned char>(s[i - 1]))) {
      return s.substr(0, i);
    }
  }
  return s;
}

bool parseBool(std::string_view v, bool& out) {
  const std::string s = lower(v);
  if (s == "1" || s == "true" || s == "yes" || s == "on") return out = true, true;
  if (s == "0" || s == "false" || s == "no" || s == "off") return out = false, true;
  return false;
}

bool parseDouble(std::string_view v, double& out) {
  const std::string s(v);
  char* end = nullptr;
  const double d = std::strtod(s.c_str(), &end);
  if (s.empty() || end != s.c_str() + s.size()) return false;
  out = d;
  return true;
}

bool parseFloat(std::string_view v, float& out) {
  double d = 0;
  if (!parseDouble(v, d)) return false;
  out = static_cast<float>(d);
  return true;
}

bool parseInt(std::string_view v, int& out) {
  const auto* first = v.data();
  const auto* last = v.data() + v.size();
  const auto res = std::from_chars(first, last, out);
  return res.ec == std::errc() && res.ptr == last;
}

class Parser {
 public:
  Parser(AppConfig& cfg, std::vector<std::string>& warnings) : cfg_(cfg), warnings_(warnings) {}

  void parse(std::string_view text) {
    std::size_t lineNo = 0;
    while (!text.empty()) {
      const auto nl = text.find('\n');
      std::string_view line = text.substr(0, nl);
      text = nl == std::string_view::npos ? std::string_view{} : text.substr(nl + 1);
      ++lineNo;
      parseLine(line, lineNo);
    }
  }

 private:
  void warn(std::size_t lineNo, const std::string& msg) {
    warnings_.push_back("line " + std::to_string(lineNo) + ": " + msg);
  }

  void parseLine(std::string_view line, std::size_t lineNo) {
    line = trim(stripComment(trim(line)));
    if (line.empty() || line.front() == '#' || line.front() == ';') return;

    if (line.front() == '[') {
      if (line.back() != ']') return warn(lineNo, "malformed section header");
      section_ = lower(trim(line.substr(1, line.size() - 2)));
      if (section_ == "bindings" && !bindingsReplaced_) {
        cfg_.bindings.clear();  // A [bindings] section replaces the defaults.
        bindingsReplaced_ = true;
      }
      return;
    }

    const auto eq = line.find('=');
    if (eq == std::string_view::npos) return warn(lineNo, "expected key = value");
    const std::string_view key = trim(line.substr(0, eq));
    const std::string_view value = trim(line.substr(eq + 1));

    if (section_ == "bindings") return parseBinding(key, value, lineNo);
    if (!setValue(section_, lower(key), value, lineNo)) {
      warn(lineNo, "unknown key '" + std::string(key) + "' in [" + section_ + "]");
    }
  }

  void parseBinding(std::string_view key, std::string_view value, std::size_t lineNo) {
    // "<gesture> <event> = <action> [@process]"
    std::istringstream ks{std::string(key)};
    std::string gestureName, eventName, extra;
    ks >> gestureName >> eventName >> extra;
    if (gestureName.empty() || eventName.empty() || !extra.empty()) {
      return warn(lineNo, "binding key must be '<gesture> <event>'");
    }
    const auto gesture = gestureFromString(gestureName);
    if (!gesture || *gesture == Gesture::None) {
      return warn(lineNo, "unknown gesture '" + gestureName + "'");
    }
    const auto event = eventFromString(lower(eventName));
    if (!event) return warn(lineNo, "unknown event '" + eventName + "'");

    Binding b;
    b.gesture = *gesture;
    b.event = *event;
    std::string_view actionText = value;
    // " @process" must be preceded by whitespace so URLs like user@host survive.
    std::size_t at = std::string_view::npos;
    for (std::size_t i = value.size(); i-- > 1;) {
      if (value[i] == '@' && std::isspace(static_cast<unsigned char>(value[i - 1]))) {
        at = i;
        break;
      }
    }
    if (at != std::string_view::npos) {
      b.requireProcess = lower(trim(value.substr(at + 1)));
      actionText = trim(value.substr(0, at));
    }
    const auto action = parseAction(actionText);
    if (!action) return warn(lineNo, "invalid action '" + std::string(actionText) + "'");
    b.action = *action;
    cfg_.bindings.push_back(std::move(b));
  }

  bool gesture(std::string_view value, Gesture& out, std::size_t lineNo) {
    const auto g = gestureFromString(value);
    if (!g) {
      warn(lineNo, "unknown gesture '" + std::string(value) + "'");
      return true;  // Key was known; the value was bad.
    }
    out = *g;
    return true;
  }

  template <typename T, typename Fn>
  bool number(std::string_view value, T& out, Fn parseFn, std::size_t lineNo) {
    if (!parseFn(value, out)) warn(lineNo, "invalid value '" + std::string(value) + "'");
    return true;
  }

  bool setValue(const std::string& sec, const std::string& key, std::string_view v,
                std::size_t ln) {
    auto f = [&](float& out) { return number(v, out, parseFloat, ln); };
    auto d = [&](double& out) { return number(v, out, parseDouble, ln); };
    auto i = [&](int& out) { return number(v, out, parseInt, ln); };
    auto b = [&](bool& out) { return number(v, out, parseBool, ln); };

    if (sec == "camera") {
      if (key == "index") return i(cfg_.camera.index);
      if (key == "width") return i(cfg_.camera.width);
      if (key == "height") return i(cfg_.camera.height);
      if (key == "mirror") return b(cfg_.camera.mirror);
    } else if (sec == "models") {
      if (key == "palm") return cfg_.models.palmModel = std::string(v), true;
      if (key == "landmark") return cfg_.models.landmarkModel = std::string(v), true;
      if (key == "palm_score") return f(cfg_.models.palmScore);
      if (key == "nms_threshold") return f(cfg_.models.nmsThreshold);
      if (key == "landmark_score") return f(cfg_.models.landmarkScore);
      if (key == "max_hands") return i(cfg_.models.maxHands);
      if (key == "tracking") return b(cfg_.models.tracking);
    } else if (sec == "preview") {
      if (key == "enabled") return b(cfg_.preview.enabled);
      if (key == "topmost") return b(cfg_.preview.topmost);
      if (key == "show_stats") return b(cfg_.preview.showStats);
    } else if (sec == "classifier") {
      if (key == "finger_extension_ratio") return f(cfg_.classifier.fingerExtensionRatio);
      if (key == "thumb_extension_ratio") return f(cfg_.classifier.thumbExtensionRatio);
      if (key == "smoothing_frames") {
        int n = 0;
        if (parseInt(v, n) && n > 0) {
          cfg_.smoothingFrames = static_cast<std::size_t>(n);
        } else {
          warn(ln, "smoothing_frames must be a positive integer");
        }
        return true;
      }
    } else if (sec == "temporal") {
      if (key == "swipe_window_ms") return d(cfg_.temporal.swipeWindowMs);
      if (key == "swipe_distance") return f(cfg_.temporal.swipeDistance);
      if (key == "hold_ms") return d(cfg_.temporal.holdMs);
      if (key == "hold_max_motion") return f(cfg_.temporal.holdMaxMotion);
      if (key == "cooldown_ms") return d(cfg_.temporal.cooldownMs);
      if (key == "flip_min_separation") return f(cfg_.temporal.flipMinSeparation);
    } else if (sec == "modes") {
      auto& m = cfg_.modes;
      if (key == "brightness_gesture") return gesture(v, m.brightnessGesture, ln);
      if (key == "volume_gesture") return gesture(v, m.volumeGesture, ln);
      if (key == "depth_gesture") return gesture(v, m.depthGesture, ln);
      if (key == "exit_gesture") return gesture(v, m.exitGesture, ln);
      if (key == "hand_lost_timeout_ms") return d(m.handLostTimeoutMs);
      if (key == "volume_min_angle") return f(m.volumeMinAngle);
      if (key == "volume_max_angle") return f(m.volumeMaxAngle);
      if (key == "brightness_top") return f(m.brightnessTop);
      if (key == "brightness_bottom") return f(m.brightnessBottom);
      if (key == "volume_smoothing") return f(m.volumeFilter.smoothing);
      if (key == "brightness_smoothing") return f(m.brightnessFilter.smoothing);
      if (key == "volume_interval_ms") return d(m.volumeFilter.minIntervalMs);
      if (key == "brightness_interval_ms") return d(m.brightnessFilter.minIntervalMs);
    } else if (sec == "mouse") {
      if (key == "margin") return f(cfg_.mouse.margin);
      if (key == "pinch_on") return f(cfg_.mouse.pinchOn);
      if (key == "pinch_off") return f(cfg_.mouse.pinchOff);
      if (key == "min_cutoff") return d(cfg_.mouse.minCutoff);
      if (key == "beta") return d(cfg_.mouse.beta);
    }
    return false;
  }

  AppConfig& cfg_;
  std::vector<std::string>& warnings_;
  std::string section_;
  bool bindingsReplaced_ = false;
};

Binding makeBinding(Gesture g, EventType e, std::string_view action, std::string process = {}) {
  Binding b;
  b.gesture = g;
  b.event = e;
  b.action = *parseAction(action);
  b.requireProcess = std::move(process);
  return b;
}

}  // namespace

AppConfig defaultConfig() {
  AppConfig cfg;
  // Mirrors the hard-coded behaviour of v1 (hand_tracker_functional.py).
  cfg.bindings = {
      makeBinding(Gesture::Fist, EventType::Hold, "mode:brightness"),
      makeBinding(Gesture::Two, EventType::Hold, "mode:volume"),
      makeBinding(Gesture::Four, EventType::Hold, "mode:depth"),
      makeBinding(Gesture::MiddleFinger, EventType::Hold, "quit"),
      makeBinding(Gesture::One, EventType::SwipeLeft, "media:prev", "spotify.exe"),
      makeBinding(Gesture::Three, EventType::SwipeRight, "media:next", "spotify.exe"),
      makeBinding(Gesture::Four, EventType::SwipeRight, "keys:ctrl+t", "chrome.exe"),
      makeBinding(Gesture::Three, EventType::Flip, "keys:alt+tab"),
  };
  return cfg;
}

ConfigResult parseConfig(std::string_view text, AppConfig base) {
  ConfigResult result{std::move(base), {}};
  Parser(result.config, result.warnings).parse(text);

  auto& m = result.config.mouse;
  if (m.pinchOff < m.pinchOn) {
    result.warnings.push_back("mouse.pinch_off < pinch_on; swapping so release > press");
    std::swap(m.pinchOn, m.pinchOff);
  }
  if (result.config.models.maxHands < 1) {
    result.warnings.push_back("models.max_hands must be >= 1; using 1");
    result.config.models.maxHands = 1;
  }
  return result;
}

ConfigResult loadConfigFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    ConfigResult r{defaultConfig(), {}};
    r.warnings.push_back("config file '" + path + "' not found; using built-in defaults");
    return r;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  std::string text = ss.str();
  if (text.size() >= 3 && static_cast<unsigned char>(text[0]) == 0xEF &&
      static_cast<unsigned char>(text[1]) == 0xBB && static_cast<unsigned char>(text[2]) == 0xBF) {
    text.erase(0, 3);  // UTF-8 BOM written by some Windows editors.
  }
  return parseConfig(text);
}

}  // namespace ht
