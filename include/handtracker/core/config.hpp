// Application configuration, loaded from a small INI file.
//
// v1 hard-coded every threshold and binding in the main loop. v2 moves them
// into config/handtracker.ini so gestures can be remapped without recompiling.
#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "handtracker/core/actions.hpp"
#include "handtracker/core/filters.hpp"
#include "handtracker/core/gesture.hpp"
#include "handtracker/core/temporal.hpp"

namespace ht {

struct CameraConfig {
  int index = 0;
  int width = 640;
  int height = 480;
  bool mirror = true;  ///< Selfie view: moving your hand right moves it right on screen.
};

struct ModelConfig {
  std::string palmModel = "models/palm_detection_mediapipe_2023feb.onnx";
  std::string landmarkModel = "models/handpose_estimation_mediapipe_2023feb.onnx";
  float palmScore = 0.5f;
  float nmsThreshold = 0.3f;
  float landmarkScore = 0.8f;
  int maxHands = 1;
  bool tracking = true;  ///< Reuse last frame's landmarks instead of re-running palm detection.
};

struct PreviewConfig {
  bool enabled = true;
  bool topmost = true;
  bool showStats = true;
};

struct ModeConfig {
  Gesture brightnessGesture = Gesture::Peace;
  Gesture volumeGesture = Gesture::Three;
  Gesture depthGesture = Gesture::One;
  Gesture exitGesture = Gesture::Palm;
  double handLostTimeoutMs = 3000.0;  ///< Leave a mode if the hand disappears this long.
  float volumeMinAngle = -60.f;
  float volumeMaxAngle = 60.f;
  float brightnessTop = 0.2f;     ///< Normalised y at which brightness reaches 100%.
  float brightnessBottom = 0.8f;  ///< Normalised y at which brightness reaches 0%.
  LevelFilterConfig volumeFilter{0.4f, 0.01f, 40.0};
  LevelFilterConfig brightnessFilter{0.3f, 0.02f, 150.0};
};

struct MouseConfig {
  float margin = 0.12f;  ///< Fraction of the frame ignored at each edge.
  float pinchOn = 0.25f;
  float pinchOff = 0.35f;
  double minCutoff = 1.0;
  double beta = 0.02;
};

struct AppConfig {
  CameraConfig camera;
  ModelConfig models;
  PreviewConfig preview;
  ClassifierConfig classifier;
  std::size_t smoothingFrames = 5;
  TemporalConfig temporal;
  ModeConfig modes;
  MouseConfig mouse;
  std::vector<Binding> bindings;
};

/// Defaults, including bindings that reproduce v1's behaviour.
AppConfig defaultConfig();

struct ConfigResult {
  AppConfig config;
  std::vector<std::string> warnings;
};

/// Parses INI text on top of `base`. A [bindings] section replaces the base bindings.
ConfigResult parseConfig(std::string_view text, AppConfig base = defaultConfig());

/// Reads and parses a file. A missing file yields defaults plus a warning.
ConfigResult loadConfigFile(const std::string& path);

}  // namespace ht
