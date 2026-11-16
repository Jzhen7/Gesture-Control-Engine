#include "handtracker/vision/palm_detector.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace ht::vision {
namespace {

constexpr int kInputSize = 192;
constexpr int kNumAnchors = 2016;
constexpr int kRegressorStride = 18;  // 4 box values + 7 keypoints * 2

}  // namespace

std::vector<cv::Point2f> generatePalmAnchors() {
  // MediaPipe SSD anchor config for this model: strides {8, 16, 16, 16},
  // fixed anchor size, offset 0.5. Layers sharing a stride are merged, giving
  // 2 anchors per cell on the 24x24 grid and 6 per cell on the 12x12 grid.
  struct Layer {
    int stride;
    int anchorsPerCell;
  };
  constexpr Layer kLayers[] = {{8, 2}, {16, 6}};

  std::vector<cv::Point2f> anchors;
  anchors.reserve(kNumAnchors);
  for (const Layer& layer : kLayers) {
    const int grid = kInputSize / layer.stride;
    for (int y = 0; y < grid; ++y) {
      for (int x = 0; x < grid; ++x) {
        const cv::Point2f centre((static_cast<float>(x) + 0.5f) / static_cast<float>(grid),
                                 (static_cast<float>(y) + 0.5f) / static_cast<float>(grid));
        for (int a = 0; a < layer.anchorsPerCell; ++a) anchors.push_back(centre);
      }
    }
  }
  return anchors;
}

PalmDetector::PalmDetector(const std::string& modelPath, float scoreThreshold, float nmsThreshold,
                           int topK, int backend, int target)
    : net_(cv::dnn::readNet(modelPath)),
      anchors_(generatePalmAnchors()),
      scoreThreshold_(scoreThreshold),
      nmsThreshold_(nmsThreshold),
      topK_(topK) {
  if (net_.empty()) throw std::runtime_error("failed to load palm model: " + modelPath);
  net_.setPreferableBackend(backend);
  net_.setPreferableTarget(target);
  outputNames_ = net_.getUnconnectedOutLayersNames();
}

std::vector<PalmDetection> PalmDetector::detect(const cv::Mat& bgr) {
  if (bgr.empty()) return {};

  // Letterbox to 192x192, keeping the aspect ratio.
  const float ratio = std::min(static_cast<float>(kInputSize) / static_cast<float>(bgr.rows),
                               static_cast<float>(kInputSize) / static_cast<float>(bgr.cols));
  const int rw = static_cast<int>(static_cast<float>(bgr.cols) * ratio);
  const int rh = static_cast<int>(static_cast<float>(bgr.rows) * ratio);
  cv::Mat resized;
  cv::resize(bgr, resized, cv::Size(rw, rh));
  const int padW = kInputSize - rw;
  const int padH = kInputSize - rh;
  const int left = padW / 2;
  const int top = padH / 2;
  cv::Mat padded;
  cv::copyMakeBorder(resized, padded, top, padH - top, left, padW - left, cv::BORDER_CONSTANT,
                     cv::Scalar::all(0));
  cv::Mat rgb;
  cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);
  cv::Mat input;
  rgb.convertTo(input, CV_32FC3, 1.0 / 255.0);

  // The model takes NHWC, which is exactly the memory layout of a continuous
  // CV_32FC3 image, so wrap it without copying.
  const int shape[] = {1, kInputSize, kInputSize, 3};
  const cv::Mat blob(4, shape, CV_32F, input.ptr<float>());
  net_.setInput(blob);
  std::vector<cv::Mat> outputs;
  net_.forward(outputs, outputNames_);

  const float* regressors = nullptr;
  const float* scores = nullptr;
  for (const cv::Mat& out : outputs) {
    if (out.total() == static_cast<std::size_t>(kNumAnchors) * kRegressorStride) {
      regressors = out.ptr<float>();
    } else if (out.total() == static_cast<std::size_t>(kNumAnchors)) {
      scores = out.ptr<float>();
    }
  }
  if (!regressors || !scores) throw std::runtime_error("unexpected palm model output shapes");

  const float scale = static_cast<float>(std::max(bgr.cols, bgr.rows));
  const cv::Point2f padBias(static_cast<float>(static_cast<int>(static_cast<float>(left) / ratio)),
                            static_cast<float>(static_cast<int>(static_cast<float>(top) / ratio)));
  const float inv = 1.f / static_cast<float>(kInputSize);

  std::vector<cv::Rect2d> boxes;
  std::vector<float> confidences;
  std::vector<int> anchorIndex;
  for (int i = 0; i < kNumAnchors; ++i) {
    const float logit = std::clamp(scores[i], -100.f, 100.f);
    const float score = 1.f / (1.f + std::exp(-logit));
    if (score < scoreThreshold_) continue;

    const float* r = regressors + static_cast<std::ptrdiff_t>(i) * kRegressorStride;
    const cv::Point2f& anchor = anchors_[static_cast<std::size_t>(i)];
    const float cx = r[0] * inv + anchor.x;
    const float cy = r[1] * inv + anchor.y;
    const float w = r[2] * inv;
    const float h = r[3] * inv;
    const float x1 = (cx - w / 2.f) * scale - padBias.x;
    const float y1 = (cy - h / 2.f) * scale - padBias.y;
    boxes.emplace_back(x1, y1, w * scale, h * scale);
    confidences.push_back(score);
    anchorIndex.push_back(i);
  }

  std::vector<int> keep;
  cv::dnn::NMSBoxes(boxes, confidences, scoreThreshold_, nmsThreshold_, keep, 1.f, topK_);

  std::vector<PalmDetection> palms;
  palms.reserve(keep.size());
  for (int k : keep) {
    const auto ku = static_cast<std::size_t>(k);
    const int i = anchorIndex[ku];
    const float* r = regressors + static_cast<std::ptrdiff_t>(i) * kRegressorStride;
    const cv::Point2f& anchor = anchors_[static_cast<std::size_t>(i)];

    PalmDetection p;
    const cv::Rect2d& b = boxes[ku];
    p.topLeft = cv::Point2f(static_cast<float>(b.x), static_cast<float>(b.y));
    p.bottomRight = cv::Point2f(static_cast<float>(b.x + b.width), static_cast<float>(b.y + b.height));
    for (std::size_t j = 0; j < p.keypoints.size(); ++j) {
      const float kx = r[4 + 2 * j] * inv + anchor.x;
      const float ky = r[5 + 2 * j] * inv + anchor.y;
      p.keypoints[j] = cv::Point2f(kx * scale - padBias.x, ky * scale - padBias.y);
    }
    p.score = confidences[ku];
    palms.push_back(p);
  }
  std::sort(palms.begin(), palms.end(),
            [](const PalmDetection& a, const PalmDetection& b) { return a.score > b.score; });
  return palms;
}

}  // namespace ht::vision
