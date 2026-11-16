// Stage 1 of the MediaPipe hand pipeline: find palms in the full frame.
//
// Runs MediaPipe's palm detection model (exported to ONNX by the OpenCV Model
// Zoo) through OpenCV's DNN module, so no MediaPipe/Bazel build is required.
#pragma once

#include <array>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/dnn.hpp>

namespace ht::vision {

struct PalmDetection {
  cv::Point2f topLeft;
  cv::Point2f bottomRight;
  /// Wrist, index/middle/ring/pinky knuckles, thumb CMC and MCP (MediaPipe order).
  std::array<cv::Point2f, 7> keypoints{};
  float score = 0.f;
};

/// SSD anchor centres for the 192x192 palm model (2016 anchors, normalised).
std::vector<cv::Point2f> generatePalmAnchors();

class PalmDetector {
 public:
  PalmDetector(const std::string& modelPath, float scoreThreshold = 0.5f,
               float nmsThreshold = 0.3f, int topK = 5000,
               int backend = cv::dnn::DNN_BACKEND_OPENCV, int target = cv::dnn::DNN_TARGET_CPU);

  /// Detects palms in a BGR frame. Coordinates are in frame pixels, best first.
  std::vector<PalmDetection> detect(const cv::Mat& bgr);

 private:
  cv::dnn::Net net_;
  std::vector<cv::Point2f> anchors_;
  std::vector<std::string> outputNames_;
  float scoreThreshold_;
  float nmsThreshold_;
  int topK_;
};

}  // namespace ht::vision
