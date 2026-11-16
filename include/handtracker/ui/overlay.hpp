// On-frame UI drawn with OpenCV. Replaces v1's separate tkinter bar windows,
// which needed their own event pump and cluttered the desktop.
#pragma once

#include <deque>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "handtracker/core/landmarks.hpp"

namespace ht::ui {

void drawHand(cv::Mat& frame, const HandLandmarks& hand);

/// Vertical level meter at the right edge (slot 0 = rightmost).
void drawLevelBar(cv::Mat& frame, const std::string& title, float level, int slot = 0);

/// Text lines in the top-left corner with a translucent backdrop.
void drawTextPanel(cv::Mat& frame, const std::vector<std::string>& lines);

/// Ring that fills while a gesture is being held.
void drawHoldRing(cv::Mat& frame, const HandLandmarks& hand, float progress);

/// Short-lived message at the bottom of the frame.
void drawToast(cv::Mat& frame, const std::string& text, float alpha);

void drawActiveRegion(cv::Mat& frame, float margin);

/// Rolling frames-per-second and latency statistics.
class FrameStats {
 public:
  void addFrame(double tMs, double inferenceMs);
  double fps() const;
  double avgInferenceMs() const;

 private:
  std::deque<double> times_;
  std::deque<double> inference_;
};

}  // namespace ht::ui
