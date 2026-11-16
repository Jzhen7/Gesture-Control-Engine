#include "handtracker/vision/hand_landmarker.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <opencv2/imgproc.hpp>

namespace ht::vision {
namespace {

constexpr int kInputSize = 224;

// Region-of-interest heuristics from MediaPipe's hand landmark graph.
constexpr float kPalmShiftY = -0.4f;      // Move the ROI towards the fingers...
constexpr float kPalmEnlarge = 3.0f;      // ...and grow it to cover the whole hand.
constexpr float kPreRotationEnlarge = 4.0f;
constexpr float kHandShiftY = -0.1f;
constexpr float kHandEnlarge = 1.65f;

struct Crop {
  cv::Mat image;
  cv::Point2f boxTopLeft;      ///< Crop region (before padding) in source coordinates.
  cv::Point2f boxBottomRight;
  cv::Point2f bias;            ///< Source coordinate of the padded crop's origin.
};

/// Shifts/enlarges a box, crops it from `image` and pads it to a square.
/// For the pre-rotation crop the square is the box diagonal, so rotating it
/// never cuts off corners.
Crop cropAndPad(const cv::Mat& image, cv::Point2f tl, cv::Point2f br, bool forRotation) {
  cv::Point2f wh = br - tl;
  const cv::Point2f shift = forRotation ? cv::Point2f(0.f, 0.f) : cv::Point2f(0.f, kPalmShiftY * wh.y);
  tl += shift;
  br += shift;
  const cv::Point2f centre = (tl + br) * 0.5f;
  wh = br - tl;
  const float enlarge = forRotation ? kPreRotationEnlarge : kPalmEnlarge;
  const cv::Point2f half = wh * (enlarge * 0.5f);

  const int x1 = std::clamp(static_cast<int>(centre.x - half.x), 0, image.cols);
  const int y1 = std::clamp(static_cast<int>(centre.y - half.y), 0, image.rows);
  const int x2 = std::clamp(static_cast<int>(centre.x + half.x), 0, image.cols);
  const int y2 = std::clamp(static_cast<int>(centre.y + half.y), 0, image.rows);

  Crop out;
  out.boxTopLeft = cv::Point2f(static_cast<float>(x1), static_cast<float>(y1));
  out.boxBottomRight = cv::Point2f(static_cast<float>(x2), static_cast<float>(y2));
  if (x2 <= x1 || y2 <= y1) return out;  // Empty: palm entirely off-frame.

  const cv::Mat roi = image(cv::Range(y1, y2), cv::Range(x1, x2));
  const int h = roi.rows;
  const int w = roi.cols;
  const int side = forRotation ? static_cast<int>(std::sqrt(static_cast<double>(h) * h +
                                                            static_cast<double>(w) * w))
                               : std::max(h, w);
  const int padH = side - h;
  const int padW = side - w;
  const int left = padW / 2;
  const int top = padH / 2;
  cv::copyMakeBorder(roi, out.image, top, padH - top, left, padW - left, cv::BORDER_CONSTANT,
                     cv::Scalar::all(0));
  out.bias = cv::Point2f(static_cast<float>(x1 - left), static_cast<float>(y1 - top));
  return out;
}

cv::Point2f apply(const cv::Mat& m, const cv::Point2f& p) {
  return {static_cast<float>(m.at<double>(0, 0) * p.x + m.at<double>(0, 1) * p.y + m.at<double>(0, 2)),
          static_cast<float>(m.at<double>(1, 0) * p.x + m.at<double>(1, 1) * p.y + m.at<double>(1, 2))};
}

}  // namespace

HandLandmarker::HandLandmarker(const std::string& modelPath, float confidenceThreshold,
                               int backend, int target)
    : net_(cv::dnn::readNet(modelPath)), confidenceThreshold_(confidenceThreshold) {
  if (net_.empty()) throw std::runtime_error("failed to load landmark model: " + modelPath);
  net_.setPreferableBackend(backend);
  net_.setPreferableTarget(target);
  // Outputs are Identity (landmarks), Identity_1 (confidence), Identity_2
  // (handedness), Identity_3 (world landmarks). Sort so the order is explicit.
  outputNames_ = net_.getUnconnectedOutLayersNames();
  std::sort(outputNames_.begin(), outputNames_.end());
  if (outputNames_.size() != 4) {
    throw std::runtime_error("unexpected landmark model: expected 4 outputs");
  }
}

std::optional<LandmarkResult> HandLandmarker::estimate(const cv::Mat& bgr,
                                                       const PalmDetection& palm) {
  // 1. Crop generously around the palm.
  const Crop first = cropAndPad(bgr, palm.topLeft, palm.bottomRight, /*forRotation=*/true);
  if (first.image.empty()) return std::nullopt;
  cv::Mat rgb;
  cv::cvtColor(first.image, rgb, cv::COLOR_BGR2RGB);
  const cv::Point2f padBias = first.bias;

  // 2. Rotate so the wrist -> middle-knuckle axis points straight up.
  std::array<cv::Point2f, 7> kps{};
  for (std::size_t i = 0; i < kps.size(); ++i) kps[i] = palm.keypoints[i] - padBias;
  const cv::Point2f p1 = kps[0];  // wrist
  const cv::Point2f p2 = kps[2];  // middle finger knuckle
  double radians = CV_PI / 2.0 - std::atan2(-(p2.y - p1.y), p2.x - p1.x);
  radians -= 2.0 * CV_PI * std::floor((radians + CV_PI) / (2.0 * CV_PI));
  const double angle = radians * 180.0 / CV_PI;

  const cv::Point2f centre = ((first.boxTopLeft - padBias) + (first.boxBottomRight - padBias)) * 0.5f;
  const cv::Mat rotation = cv::getRotationMatrix2D(centre, angle, 1.0);
  cv::Mat rotated;
  cv::warpAffine(rgb, rotated, rotation, rgb.size());

  // 3. Tight, upright crop around the rotated palm keypoints.
  cv::Point2f lo(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  cv::Point2f hi(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
  for (const auto& kp : kps) {
    const cv::Point2f r = apply(rotation, kp);
    lo.x = std::min(lo.x, r.x);
    lo.y = std::min(lo.y, r.y);
    hi.x = std::max(hi.x, r.x);
    hi.y = std::max(hi.y, r.y);
  }
  const Crop second = cropAndPad(rotated, lo, hi, /*forRotation=*/false);
  if (second.image.empty()) return std::nullopt;

  cv::Mat resized;
  cv::resize(second.image, resized, cv::Size(kInputSize, kInputSize), 0, 0, cv::INTER_AREA);
  cv::Mat input;
  resized.convertTo(input, CV_32FC3, 1.0 / 255.0);
  const int shape[] = {1, kInputSize, kInputSize, 3};
  const cv::Mat blob(4, shape, CV_32F, input.ptr<float>());
  net_.setInput(blob);
  std::vector<cv::Mat> outputs;
  net_.forward(outputs, outputNames_);
  if (outputs.size() != 4 || outputs[0].total() != 63 || outputs[3].total() != 63) {
    throw std::runtime_error("unexpected landmark model output shapes");
  }

  LandmarkResult result;
  result.confidence = outputs[1].ptr<float>()[0];
  if (result.confidence < confidenceThreshold_) return std::nullopt;
  result.handedness = outputs[2].ptr<float>()[0];
  const float* lm = outputs[0].ptr<float>();
  const float* world = outputs[3].ptr<float>();

  // 4. Map landmarks from model space back to frame pixels.
  const cv::Point2f roiSize = second.boxBottomRight - second.boxTopLeft;
  const float s = std::max(roiSize.x, roiSize.y) / static_cast<float>(kInputSize);
  const cv::Mat undo = cv::getRotationMatrix2D(cv::Point2f(0.f, 0.f), angle, 1.0);
  const double a00 = undo.at<double>(0, 0), a01 = undo.at<double>(0, 1);
  const double a10 = undo.at<double>(1, 0), a11 = undo.at<double>(1, 1);

  // Centre of the rotated ROI, taken back through the inverse rotation.
  const double r00 = rotation.at<double>(0, 0), r01 = rotation.at<double>(0, 1);
  const double r10 = rotation.at<double>(1, 0), r11 = rotation.at<double>(1, 1);
  const double t0 = rotation.at<double>(0, 2), t1 = rotation.at<double>(1, 2);
  const cv::Point2f roiCentre = (second.boxTopLeft + second.boxBottomRight) * 0.5f;
  const double ox = r00 * roiCentre.x + r10 * roiCentre.y - (r00 * t0 + r10 * t1);
  const double oy = r01 * roiCentre.x + r11 * roiCentre.y - (r01 * t0 + r11 * t1);

  const float half = static_cast<float>(kInputSize) / 2.f;
  cv::Point2f bbLo(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  cv::Point2f bbHi(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
  for (std::size_t i = 0; i < 21; ++i) {
    const double x = (lm[3 * i] - half) * s;
    const double y = (lm[3 * i + 1] - half) * s;
    const float z = lm[3 * i + 2] * s;
    const double rx = x * a00 + y * a10;
    const double ry = x * a01 + y * a11;
    cv::Point3f& p = result.image[i];
    p.x = static_cast<float>(rx + ox + padBias.x);
    p.y = static_cast<float>(ry + oy + padBias.y);
    p.z = z;
    bbLo.x = std::min(bbLo.x, p.x);
    bbLo.y = std::min(bbLo.y, p.y);
    bbHi.x = std::max(bbHi.x, p.x);
    bbHi.y = std::max(bbHi.y, p.y);

    const double wx = world[3 * i], wy = world[3 * i + 1];
    result.world[i] = cv::Point3f(static_cast<float>(wx * a00 + wy * a10),
                                  static_cast<float>(wx * a01 + wy * a11), world[3 * i + 2]);
  }

  cv::Point2f wh = bbHi - bbLo;
  bbLo.y += kHandShiftY * wh.y;
  bbHi.y += kHandShiftY * wh.y;
  const cv::Point2f c = (bbLo + bbHi) * 0.5f;
  wh = (bbHi - bbLo) * (kHandEnlarge * 0.5f);
  result.box = cv::Rect2f(c - wh, c + wh);
  return result;
}

PalmDetection HandLandmarker::palmFromLandmarks(const LandmarkResult& lm) {
  // Same 7 points the palm detector predicts: wrist, 4 knuckles, thumb CMC/MCP.
  constexpr std::array<int, 7> kPalmIds{0, 5, 9, 13, 17, 1, 2};
  PalmDetection p;
  p.topLeft = cv::Point2f(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
  p.bottomRight =
      cv::Point2f(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());
  for (std::size_t i = 0; i < kPalmIds.size(); ++i) {
    const cv::Point3f& q = lm.image[static_cast<std::size_t>(kPalmIds[i])];
    p.keypoints[i] = cv::Point2f(q.x, q.y);
    p.topLeft.x = std::min(p.topLeft.x, q.x);
    p.topLeft.y = std::min(p.topLeft.y, q.y);
    p.bottomRight.x = std::max(p.bottomRight.x, q.x);
    p.bottomRight.y = std::max(p.bottomRight.y, q.y);
  }
  p.score = lm.confidence;
  return p;
}

}  // namespace ht::vision
