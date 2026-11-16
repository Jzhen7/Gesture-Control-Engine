// Stage 2 of the MediaPipe hand pipeline: 21 landmarks from a palm region.
#pragma once

#include <array>
#include <optional>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

#include "handtracker/vision/palm_detector.hpp"

namespace ht::vision {

struct LandmarkResult {
  std::array<cv::Point3f, 21> image{};  ///< Frame pixels; z relative to the wrist.
  std::array<cv::Point3f, 21> world{};  ///< Metric (metres), hand-centred.
  float handedness = 0.5f;              ///< Model output: 0 = left, 1 = right.
  float confidence = 0.f;
  cv::Rect2f box;                       ///< Enlarged hand bounding box.
};

class HandLandmarker {
 public:
  HandLandmarker(const std::string& modelPath, float confidenceThreshold = 0.8f,
                 int backend = cv::dnn::DNN_BACKEND_OPENCV, int target = cv::dnn::DNN_TARGET_CPU);

  /// Estimates landmarks for the hand around `palm`. nullopt if confidence is too low.
  std::optional<LandmarkResult> estimate(const cv::Mat& bgr, const PalmDetection& palm);

  /// Builds a pseudo palm detection from previous landmarks, so the next frame
  /// can skip palm detection entirely (MediaPipe-style tracking).
  static PalmDetection palmFromLandmarks(const LandmarkResult& lm);

 private:
  cv::dnn::Net net_;
  std::vector<std::string> outputNames_;
  float confidenceThreshold_;
};

}  // namespace ht::vision
