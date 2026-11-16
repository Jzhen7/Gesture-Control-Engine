// Palm detection + landmark estimation with frame-to-frame tracking.
#pragma once

#include <vector>

#include <opencv2/core.hpp>

#include "handtracker/core/config.hpp"
#include "handtracker/core/landmarks.hpp"
#include "handtracker/vision/hand_landmarker.hpp"
#include "handtracker/vision/palm_detector.hpp"

namespace ht::vision {

class HandTracker {
 public:
  struct Timing {
    double palmMs = 0.0;
    double landmarkMs = 0.0;
    bool ranDetector = false;
  };

  /// Throws cv::Exception / std::runtime_error if a model can't be loaded.
  explicit HandTracker(const ModelConfig& cfg);

  /// Processes one BGR frame and returns up to maxHands hands.
  std::vector<HandLandmarks> process(const cv::Mat& bgr);

  const Timing& lastTiming() const { return timing_; }

 private:
  ModelConfig cfg_;
  PalmDetector detector_;
  HandLandmarker landmarker_;
  std::vector<LandmarkResult> tracked_;
  Timing timing_;
};

/// Converts the vision result into the core data model.
HandLandmarks toHandLandmarks(const LandmarkResult& lm, int imageWidth, int imageHeight);

}  // namespace ht::vision
