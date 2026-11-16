// Vision tests that need OpenCV but not the model files.
#include <cmath>

#include "handtracker/vision/hand_landmarker.hpp"
#include "handtracker/vision/hand_tracker.hpp"
#include "handtracker/vision/palm_detector.hpp"
#include "test_framework.hpp"

using namespace ht;
using namespace ht::vision;

TEST_CASE("vision: palm anchors match MediaPipe's SSD layout") {
  const auto anchors = generatePalmAnchors();
  REQUIRE(anchors.size() == 2016u);
  // First cell of the 24x24 grid, twice.
  CHECK_NEAR(anchors[0].x, 0.5 / 24, 1e-6);
  CHECK_NEAR(anchors[0].y, 0.5 / 24, 1e-6);
  CHECK_NEAR(anchors[1].x, 0.5 / 24, 1e-6);
  CHECK_NEAR(anchors[2].x, 1.5 / 24, 1e-6);
  // First cell of the 12x12 grid, six times, starting at index 24*24*2.
  CHECK_NEAR(anchors[1152].x, 0.5 / 12, 1e-6);
  CHECK_NEAR(anchors[1157].y, 0.5 / 12, 1e-6);
  CHECK_NEAR(anchors[1158].x, 1.5 / 12, 1e-6);
  // Last anchor: bottom-right cell.
  CHECK_NEAR(anchors.back().x, 11.5 / 12, 1e-6);
  CHECK_NEAR(anchors.back().y, 11.5 / 12, 1e-6);
}

TEST_CASE("vision: palm from landmarks uses the 7 palm keypoints") {
  LandmarkResult lm;
  for (std::size_t i = 0; i < 21; ++i) {
    lm.image[i] = cv::Point3f(100.f + static_cast<float>(i), 200.f + 2.f * static_cast<float>(i), 0.f);
  }
  lm.confidence = 0.9f;
  const PalmDetection p = HandLandmarker::palmFromLandmarks(lm);
  CHECK_NEAR(p.keypoints[0].x, 100, 1e-6);  // wrist
  CHECK_NEAR(p.keypoints[2].x, 109, 1e-6);  // middle knuckle
  CHECK_NEAR(p.keypoints[6].x, 102, 1e-6);  // thumb MCP
  CHECK_NEAR(p.topLeft.x, 100, 1e-6);
  CHECK_NEAR(p.bottomRight.x, 117, 1e-6);   // pinky knuckle (17)
  CHECK_NEAR(p.score, 0.9, 1e-6);
}

TEST_CASE("vision: conversion keeps frame size and world points") {
  LandmarkResult lm;
  lm.image[9] = cv::Point3f(320.f, 240.f, 0.f);
  lm.world[9] = cv::Point3f(0.01f, 0.02f, 0.03f);
  lm.handedness = 0.9f;
  const HandLandmarks h = toHandLandmarks(lm, 640, 480);
  CHECK_NEAR(h.normalized(LandmarkId::MiddleMcp).x, 0.5, 1e-6);
  CHECK_NEAR(h.normalized(LandmarkId::MiddleMcp).y, 0.5, 1e-6);
  REQUIRE(h.world.has_value());
  CHECK_NEAR((*h.world)[9].z, 0.03, 1e-6);
  CHECK(h.handedness == Handedness::Right);
}
