// A lightweight 3D view of the simulated arm, drawn with plain OpenCV.
//
// MuJoCo has its own OpenGL renderer, but using it needs a window/GL library
// (GLFW) and a GPU context. For the MVP we instead project the 3D joint
// positions MuJoCo computes onto a 2D image ourselves (a "pinhole camera":
// screen = focal * (x, y) / depth), which works everywhere OpenCV works,
// including headless CI.
#pragma once

#include <opencv2/core.hpp>

#include "handtracker/arm/arm_backend.hpp"
#include "handtracker/arm/arm_controller.hpp"
#include "handtracker/arm/workspace.hpp"

namespace ht::app {

class ArmView {
 public:
  ArmView(int width, int height);

  cv::Mat render(const arm::ArmState& state, const arm::WorkspaceBox& box,
                 const arm::ArmStatus& status) const;

  /// Rotate the virtual camera around the arm (degrees, positive = counter-clockwise).
  void orbit(double degrees);

 private:
  /// World point -> pixel. Returns false if the point is behind the camera.
  bool project(const Vec3& p, cv::Point& out) const;
  void line3d(cv::Mat& img, const Vec3& a, const Vec3& b, const cv::Scalar& color,
              int thickness) const;

  int width_;
  int height_;
  double azimuthDeg_ = 0.0;     ///< 0 = looking over the arm's "shoulder" from behind.
  double elevationDeg_ = 28.0;
  double distance_ = 0.75;      ///< Metres from the look-at point.
  Vec3 lookAt_{0.20f, 0.f, 0.12f};
  // Camera basis, recomputed by orbit().
  Vec3 eye_;
  Vec3 right_;
  Vec3 up_;
  Vec3 forward_;
  void updateCamera();
};

}  // namespace ht::app
