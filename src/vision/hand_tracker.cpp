#include "handtracker/vision/hand_tracker.hpp"

#include <opencv2/core/utility.hpp>

namespace ht::vision {
namespace {

double msSince(int64 start) {
  return static_cast<double>(cv::getTickCount() - start) * 1000.0 / cv::getTickFrequency();
}

bool insideAny(const cv::Point2f& p, const std::vector<LandmarkResult>& hands) {
  for (const auto& h : hands) {
    if (h.box.contains(p)) return true;
  }
  return false;
}

}  // namespace

HandTracker::HandTracker(const ModelConfig& cfg)
    : cfg_(cfg),
      detector_(cfg.palmModel, cfg.palmScore, cfg.nmsThreshold),
      landmarker_(cfg.landmarkModel, cfg.landmarkScore) {}

std::vector<HandLandmarks> HandTracker::process(const cv::Mat& bgr) {
  timing_ = {};
  std::vector<LandmarkResult> found;
  const auto maxHands = static_cast<std::size_t>(cfg_.maxHands);

  // 1. Track: re-run the landmark model on last frame's hands. This skips the
  //    (more expensive) palm detector while a hand stays in view.
  int64 start = cv::getTickCount();
  if (cfg_.tracking) {
    for (const auto& prev : tracked_) {
      if (found.size() >= maxHands) break;
      if (auto lm = landmarker_.estimate(bgr, HandLandmarker::palmFromLandmarks(prev))) {
        found.push_back(*lm);
      }
    }
  }
  timing_.landmarkMs += msSince(start);

  // 2. Detect: look for new palms only if we still have capacity.
  if (found.size() < maxHands) {
    start = cv::getTickCount();
    const auto palms = detector_.detect(bgr);
    timing_.palmMs = msSince(start);
    timing_.ranDetector = true;

    start = cv::getTickCount();
    for (const auto& palm : palms) {
      if (found.size() >= maxHands) break;
      const cv::Point2f centre = (palm.topLeft + palm.bottomRight) * 0.5f;
      if (insideAny(centre, found)) continue;  // Already tracking this hand.
      if (auto lm = landmarker_.estimate(bgr, palm)) found.push_back(*lm);
    }
    timing_.landmarkMs += msSince(start);
  }

  tracked_ = found;
  std::vector<HandLandmarks> hands;
  hands.reserve(found.size());
  for (const auto& lm : found) hands.push_back(toHandLandmarks(lm, bgr.cols, bgr.rows));
  return hands;
}

HandLandmarks toHandLandmarks(const LandmarkResult& lm, int imageWidth, int imageHeight) {
  HandLandmarks h;
  HandLandmarks::Points world{};
  for (std::size_t i = 0; i < kNumLandmarks; ++i) {
    h.image[i] = Vec3(lm.image[i].x, lm.image[i].y, lm.image[i].z);
    world[i] = Vec3(lm.world[i].x, lm.world[i].y, lm.world[i].z);
  }
  h.world = world;
  h.imageWidth = static_cast<float>(imageWidth);
  h.imageHeight = static_cast<float>(imageHeight);
  h.handedness = lm.handedness >= 0.5f ? Handedness::Right : Handedness::Left;
  h.score = lm.confidence;
  return h;
}

}  // namespace ht::vision
