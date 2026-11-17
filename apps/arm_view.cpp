#include "arm_view.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>

#include <opencv2/imgproc.hpp>

namespace ht::app {
namespace {

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
const cv::Scalar kBackground(38, 32, 28);
const cv::Scalar kGrid(70, 62, 55);
const cv::Scalar kBox(120, 110, 100);
const cv::Scalar kArm(60, 150, 255);     // orange (BGR)
const cv::Scalar kJoint(235, 235, 235);
const cv::Scalar kShadow(25, 22, 20);
const cv::Scalar kTarget(80, 220, 255);  // yellow
const cv::Scalar kClosed(80, 80, 230);   // red
const cv::Scalar kOpen(120, 220, 80);    // green
const cv::Scalar kText(220, 220, 220);

Vec3 floorOf(const Vec3& p) { return {p.x, p.y, 0.f}; }

/// Red when shut, green when open, blended in between.
cv::Scalar gripColor(float opening) {
  const double t = std::clamp(static_cast<double>(opening), 0.0, 1.0);
  return kClosed * (1.0 - t) + kOpen * t;
}

}  // namespace

ArmView::ArmView(int width, int height) : width_(width), height_(height) { updateCamera(); }

void ArmView::orbit(double degrees) {
  azimuthDeg_ = std::fmod(azimuthDeg_ + degrees, 360.0);
  updateCamera();
}

void ArmView::updateCamera() {
  const double az = azimuthDeg_ * kDegToRad;
  const double el = elevationDeg_ * kDegToRad;
  // Start behind the arm (-X) so the user's left/right match the arm's on screen.
  const Vec3 offset{static_cast<float>(-std::cos(el) * std::cos(az) * distance_),
                    static_cast<float>(-std::cos(el) * std::sin(az) * distance_),
                    static_cast<float>(std::sin(el) * distance_)};
  eye_ = lookAt_ + offset;
  forward_ = normalized(lookAt_ - eye_);
  right_ = normalized(cross(forward_, Vec3{0.f, 0.f, 1.f}));
  up_ = cross(right_, forward_);
}

bool ArmView::project(const Vec3& p, cv::Point& out) const {
  const Vec3 rel = p - eye_;
  const float depth = dot(rel, forward_);
  if (depth < 0.02f) return false;
  const float focal = static_cast<float>(height_) * 1.3f;
  out.x = static_cast<int>(std::lround(width_ / 2.0 + focal * dot(rel, right_) / depth));
  out.y = static_cast<int>(std::lround(height_ / 2.0 - focal * dot(rel, up_) / depth));
  return true;
}

void ArmView::line3d(cv::Mat& img, const Vec3& a, const Vec3& b, const cv::Scalar& color,
                     int thickness) const {
  cv::Point pa, pb;
  if (project(a, pa) && project(b, pb)) cv::line(img, pa, pb, color, thickness, cv::LINE_AA);
}

cv::Mat ArmView::render(const arm::ArmState& st, const arm::WorkspaceBox& box,
                        const arm::ArmStatus& status) const {
  cv::Mat img(height_, width_, CV_8UC3, kBackground);

  // Floor grid, 5 cm squares.
  for (int i = -6; i <= 10; ++i) {
    const float x = 0.05f * static_cast<float>(i);
    line3d(img, {x, -0.30f, 0.f}, {x, 0.30f, 0.f}, kGrid, 1);
  }
  for (int i = -6; i <= 6; ++i) {
    const float y = 0.05f * static_cast<float>(i);
    line3d(img, {-0.30f, y, 0.f}, {0.50f, y, 0.f}, kGrid, 1);
  }

  // Workspace box: the region the hand is mapped onto.
  const Vec3& a = box.min;
  const Vec3& b = box.max;
  const std::array<Vec3, 8> c{{{a.x, a.y, a.z}, {b.x, a.y, a.z}, {b.x, b.y, a.z}, {a.x, b.y, a.z},
                               {a.x, a.y, b.z}, {b.x, a.y, b.z}, {b.x, b.y, b.z}, {a.x, b.y, b.z}}};
  for (int i = 0; i < 4; ++i) {
    const auto u = static_cast<std::size_t>(i);
    const auto n = static_cast<std::size_t>((i + 1) % 4);
    line3d(img, c[u], c[n], kBox, 1);
    line3d(img, c[u + 4], c[n + 4], kBox, 1);
    line3d(img, c[u], c[u + 4], kBox, 1);
  }

  // Shadows on the floor give a sense of depth.
  for (std::size_t i = 1; i < st.skeleton.size(); ++i) {
    line3d(img, floorOf(st.skeleton[i - 1]), floorOf(st.skeleton[i]), kShadow, 6);
  }
  line3d(img, st.target, floorOf(st.target), kTarget, 1);  // drop line from target

  // Target marker.
  cv::Point tp;
  if (project(st.target, tp)) {
    cv::circle(img, tp, 7, kTarget, 2, cv::LINE_AA);
    cv::drawMarker(img, tp, kTarget, cv::MARKER_CROSS, 18, 1, cv::LINE_AA);
  }

  // Arm links and joints.
  for (std::size_t i = 1; i < st.skeleton.size(); ++i) {
    line3d(img, st.skeleton[i - 1], st.skeleton[i], kArm, 7);
  }
  for (std::size_t i = 0; i + 1 < st.skeleton.size(); ++i) {
    cv::Point p;
    if (project(st.skeleton[i], p)) cv::circle(img, p, 5, kJoint, cv::FILLED, cv::LINE_AA);
  }

  // Gripper: two jaws whose spread shows the measured opening.
  if (st.skeleton.size() >= 2) {
    cv::Point tip, prev;
    if (project(st.skeleton.back(), tip) && project(st.skeleton[st.skeleton.size() - 2], prev)) {
      cv::Point2f dir(static_cast<float>(tip.x - prev.x), static_cast<float>(tip.y - prev.y));
      const float len = std::max(1.f, std::hypot(dir.x, dir.y));
      dir *= 1.f / len;
      const cv::Point2f perp(-dir.y, dir.x);
      const float spread = 4.f + 14.f * st.gripperOpening;
      const cv::Scalar jaw = gripColor(st.gripperCommand);
      for (float side : {-1.f, 1.f}) {
        const cv::Point2f base = cv::Point2f(tip) + perp * (side * spread * 0.5f);
        const cv::Point2f end = base + dir * 16.f + perp * (side * spread * 0.4f);
        cv::line(img, base, end, jaw, 4, cv::LINE_AA);
      }
    }
  }

  // Text.
  char buf[128];
  cv::putText(img, "SO-101 (MuJoCo simulation)", {12, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.55, kText,
              1, cv::LINE_AA);
  std::snprintf(buf, sizeof(buf), "gripper: %3.0f%% open (jaws at %3.0f%%)",
                st.gripperCommand * 100.f, st.gripperOpening * 100.f);
  cv::putText(img, buf, {12, 48}, cv::FONT_HERSHEY_SIMPLEX, 0.55, gripColor(st.gripperCommand), 2,
              cv::LINE_AA);
  std::snprintf(buf, sizeof(buf), "target  x %4.1f  y %5.1f  z %4.1f cm", st.target.x * 100.f,
                st.target.y * 100.f, st.target.z * 100.f);
  cv::putText(img, buf, {12, 72}, cv::FONT_HERSHEY_SIMPLEX, 0.45, kText, 1, cv::LINE_AA);
  std::snprintf(buf, sizeof(buf), "reach error %.1f mm", (st.gripper - st.target).norm() * 1000.f);
  cv::putText(img, buf, {12, 92}, cv::FONT_HERSHEY_SIMPLEX, 0.45, kText, 1, cv::LINE_AA);
  if (status.phase == arm::Phase::Calibrating) {
    cv::putText(img, "waiting for calibration...", {12, height_ - 16}, cv::FONT_HERSHEY_SIMPLEX,
                0.5, kTarget, 1, cv::LINE_AA);
  }
  return img;
}

}  // namespace ht::app
