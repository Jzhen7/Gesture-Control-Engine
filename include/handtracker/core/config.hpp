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
#include "handtracker/core/hand_space.hpp"
#include "handtracker/core/landmarks.hpp"
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

/// How the hand drives the arm's gripper.
enum class GripperMode {
  Proportional,  ///< Dash 3: jaw opening follows the fingertip "claw" (0..1).
  Binary,        ///< Dash 2 MVP: fist = closed, anything else = open (closed claw not used).
};

/// Robot-arm front-end settings ([arm] section, normally in config/arm.ini).
struct ArmSettings {
  std::string model = "models/so101/scene.xml";  ///< MuJoCo scene file.
  HandSpaceConfig hand;                         ///< How far the hand moves to cover the workspace.
  /// Reachable box in the arm's base frame (metres; X forward, Y left, Z up).
  /// Defaults were checked in simulation: every point is reachable within 0.1 mm.
  Vec3 workspaceMin{0.17f, -0.13f, 0.06f};
  Vec3 workspaceMax{0.31f, 0.13f, 0.24f};
  double calibrationMs = 2000.0;    ///< How long to hold the open palm when calibrating.
  float calibrationMaxMotion = 0.05f;
  double minCutoff = 1.0;           ///< One Euro filter on the target (Hz).
  double beta = 5.0;                ///< One Euro speed coefficient (target in metres).
  float maxSpeed = 0.6f;            ///< Fastest the target may move, metres/second.
  float gripperOpen = 1.2f;         ///< Jaw joint angle (radians) for "open".
  float gripperClosed = 0.0f;       ///< Jaw joint angle (radians) for "closed".
  GripperMode gripperMode = GripperMode::Proportional;
  GripConfig grip;                  ///< Claw -> 0..1 (closed spread, dead band, straightness).
  double gripMinCutoff = 2.0;       ///< One Euro filter on the openness (Hz).
  double gripBeta = 2.0;            ///< One Euro speed coefficient (openness per second).
  int ikIterations = 10;            ///< IK refinement steps per frame.
  float ikDamping = 0.05f;          ///< IK damping: higher = smoother near singularities.
  float ikNullGain = 0.05f;         ///< Pull toward the ready pose with the arm's spare freedom.
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
  ArmSettings arm;
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
