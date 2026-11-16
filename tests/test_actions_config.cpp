#include "handtracker/core/actions.hpp"
#include "handtracker/core/config.hpp"

#include "test_framework.hpp"

using namespace ht;

TEST_CASE("actions: hotkeys parse and normalise") {
  auto hk = parseHotkey("Ctrl + Shift + T");
  REQUIRE(hk.has_value());
  CHECK_EQ(hk->toString(), std::string("ctrl+shift+t"));
  CHECK_EQ(parseHotkey("control+escape")->toString(), std::string("ctrl+esc"));
  CHECK_EQ(parseHotkey("f12")->toString(), std::string("f12"));
  CHECK(!parseHotkey("ctrl+").has_value());
  CHECK(!parseHotkey("ctrl+banana").has_value());
  CHECK(!parseHotkey("f25").has_value());
  CHECK(!parseHotkey("").has_value());
}

TEST_CASE("actions: action specs parse") {
  auto a = parseAction("keys:alt+tab");
  REQUIRE(a.has_value());
  CHECK_EQ(a->kind, Action::Kind::Keys);
  CHECK_EQ(a->hotkey.toString(), std::string("alt+tab"));

  a = parseAction("media:next");
  REQUIRE(a.has_value());
  CHECK_EQ(a->hotkey.toString(), std::string("media_next"));

  a = parseAction("mode:volume");
  REQUIRE(a.has_value());
  CHECK_EQ(a->kind, Action::Kind::EnterMode);
  CHECK_EQ(a->mode, Mode::Volume);

  a = parseAction("launch:C:\\Program Files\\App\\App.exe");
  REQUIRE(a.has_value());
  CHECK_EQ(a->target, std::string("C:\\Program Files\\App\\App.exe"));

  CHECK_EQ(parseAction("quit")->kind, Action::Kind::Quit);
  CHECK(!parseAction("mode:idle").has_value());
  CHECK(!parseAction("media:louder").has_value());
  CHECK(!parseAction("dance").has_value());
}

TEST_CASE("config: defaults reproduce v1 bindings") {
  const AppConfig cfg = defaultConfig();
  CHECK_EQ(cfg.bindings.size(), static_cast<std::size_t>(8));
  bool foundAltTab = false;
  for (const auto& b : cfg.bindings) {
    if (b.gesture == Gesture::Three && b.event == EventType::Flip) {
      foundAltTab = b.action.hotkey.toString() == "alt+tab";
    }
  }
  CHECK(foundAltTab);
}

TEST_CASE("config: values, comments and sections") {
  const auto r = parseConfig(R"(
# comment
[camera]
index = 2        # trailing comment
mirror = false
[temporal]
swipe_distance = 0.2
hold_ms = 750
[modes]
volume_gesture = four
[mouse]
margin = 0.1
)");
  CHECK(r.warnings.empty());
  CHECK_EQ(r.config.camera.index, 2);
  CHECK(!r.config.camera.mirror);
  CHECK_NEAR(r.config.temporal.swipeDistance, 0.2, 1e-6);
  CHECK_NEAR(r.config.temporal.holdMs, 750, 1e-9);
  CHECK_EQ(r.config.modes.volumeGesture, Gesture::Four);
  CHECK_NEAR(r.config.mouse.margin, 0.1, 1e-6);
  CHECK_EQ(r.config.bindings.size(), defaultConfig().bindings.size());  // untouched
}

TEST_CASE("config: [bindings] replaces defaults") {
  const auto r = parseConfig(R"(
[bindings]
palm swipe_up = keys:win+d
peace hold = launch:https://user@example.com @chrome.exe
fist hold = mode:brightness
)");
  CHECK(r.warnings.empty());
  REQUIRE(r.config.bindings.size() == 3);
  CHECK_EQ(r.config.bindings[0].gesture, Gesture::Palm);
  CHECK_EQ(r.config.bindings[0].event, EventType::SwipeUp);
  CHECK_EQ(r.config.bindings[0].action.hotkey.toString(), std::string("win+d"));
  CHECK_EQ(r.config.bindings[1].action.target, std::string("https://user@example.com"));
  CHECK_EQ(r.config.bindings[1].requireProcess, std::string("chrome.exe"));
}

TEST_CASE("config: bad lines produce warnings, not failures") {
  const auto r = parseConfig(R"(
[camera]
index = abc
fps = 60
[bindings]
wave hold = quit
palm jump = quit
palm hold = keys:ctrl+nope
three flip = keys:alt+tab
garbage line
)");
  CHECK_EQ(r.warnings.size(), static_cast<std::size_t>(6));
  CHECK_EQ(r.config.camera.index, 0);  // kept default
  CHECK_EQ(r.config.bindings.size(), static_cast<std::size_t>(1));
}

TEST_CASE("config: inverted pinch thresholds are fixed up") {
  const auto r = parseConfig("[mouse]\npinch_on = 0.4\npinch_off = 0.2\n");
  CHECK_EQ(r.warnings.size(), static_cast<std::size_t>(1));
  CHECK(r.config.mouse.pinchOn < r.config.mouse.pinchOff);
}

TEST_CASE("config: windows line endings") {
  const auto r = parseConfig("[camera]\r\nindex = 3\r\n");
  CHECK(r.warnings.empty());
  CHECK_EQ(r.config.camera.index, 3);
}

TEST_CASE("config: missing file falls back to defaults") {
  const auto r = loadConfigFile("definitely/not/here.ini");
  CHECK_EQ(r.warnings.size(), static_cast<std::size_t>(1));
  CHECK_EQ(r.config.bindings.size(), defaultConfig().bindings.size());
}
