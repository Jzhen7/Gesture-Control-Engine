#include "handtracker/ui/overlay.hpp"

#include <algorithm>
#include <cmath>

#include <opencv2/imgproc.hpp>

namespace ht::ui {
namespace {

const cv::Scalar kBone(255, 255, 255);
const cv::Scalar kJoint(80, 200, 255);
const cv::Scalar kTip(60, 120, 255);
const cv::Scalar kAccent(120, 220, 80);
const cv::Scalar kPanel(30, 30, 30);
constexpr int kFont = cv::FONT_HERSHEY_SIMPLEX;

cv::Point px(const Vec3& v) {
  return {static_cast<int>(std::lround(v.x)), static_cast<int>(std::lround(v.y))};
}

void blendRect(cv::Mat& frame, cv::Rect r, const cv::Scalar& color, double alpha) {
  r &= cv::Rect(0, 0, frame.cols, frame.rows);
  if (r.empty()) return;
  cv::Mat roi = frame(r);
  cv::Mat fill(roi.size(), roi.type(), color);
  cv::addWeighted(fill, alpha, roi, 1.0 - alpha, 0.0, roi);
}

}  // namespace

void drawHand(cv::Mat& frame, const HandLandmarks& hand) {
  for (const auto& [a, b] : kHandConnections) {
    cv::line(frame, px(hand.image[static_cast<std::size_t>(a)]),
             px(hand.image[static_cast<std::size_t>(b)]), kBone, 2, cv::LINE_AA);
  }
  for (std::size_t i = 0; i < kNumLandmarks; ++i) {
    const bool tip = i == 4 || i == 8 || i == 12 || i == 16 || i == 20;
    cv::circle(frame, px(hand.image[i]), tip ? 5 : 3, tip ? kTip : kJoint, cv::FILLED, cv::LINE_AA);
  }
}

void drawLevelBar(cv::Mat& frame, const std::string& title, float level, int slot) {
  level = std::clamp(level, 0.f, 1.f);
  const int barW = 28;
  const int barH = std::max(60, frame.rows / 2);
  const int x = frame.cols - 24 - barW - slot * (barW + 56);
  const int y = (frame.rows - barH) / 2;

  blendRect(frame, cv::Rect(x - 26, y - 46, barW + 52, barH + 72), kPanel, 0.55);
  const int filled = static_cast<int>(std::lround(level * static_cast<float>(barH)));
  cv::rectangle(frame, cv::Rect(x, y + barH - filled, barW, filled), kAccent, cv::FILLED);
  cv::rectangle(frame, cv::Rect(x, y, barW, barH), kBone, 1);

  const std::string pct = std::to_string(static_cast<int>(std::lround(level * 100.f))) + "%";
  int base = 0;
  const cv::Size ts = cv::getTextSize(title, kFont, 0.45, 1, &base);
  cv::putText(frame, title, {x + barW / 2 - ts.width / 2, y - 26}, kFont, 0.45, kBone, 1, cv::LINE_AA);
  const cv::Size ps = cv::getTextSize(pct, kFont, 0.55, 2, &base);
  cv::putText(frame, pct, {x + barW / 2 - ps.width / 2, y - 8}, kFont, 0.55, kAccent, 2, cv::LINE_AA);
}

void drawTextPanel(cv::Mat& frame, const std::vector<std::string>& lines) {
  if (lines.empty()) return;
  int width = 0;
  int base = 0;
  for (const auto& l : lines) width = std::max(width, cv::getTextSize(l, kFont, 0.5, 1, &base).width);
  const int lineH = 20;
  blendRect(frame, cv::Rect(8, 8, width + 16, static_cast<int>(lines.size()) * lineH + 10), kPanel, 0.55);
  for (std::size_t i = 0; i < lines.size(); ++i) {
    cv::putText(frame, lines[i], {16, 26 + static_cast<int>(i) * lineH}, kFont, 0.5, kBone, 1,
                cv::LINE_AA);
  }
}

void drawHoldRing(cv::Mat& frame, const HandLandmarks& hand, float progress) {
  if (progress <= 0.f) return;
  const Vec3 c = (hand[LandmarkId::Wrist] + hand[LandmarkId::MiddleMcp]) * 0.5f;
  const int r = std::max(12, static_cast<int>(hand.palmSize() * 0.35f));
  cv::ellipse(frame, px(c), {r, r}, -90.0, 0.0, 360.0 * std::clamp(progress, 0.f, 1.f), kAccent, 4,
              cv::LINE_AA);
}

void drawToast(cv::Mat& frame, const std::string& text, float alpha) {
  if (text.empty() || alpha <= 0.f) return;
  int base = 0;
  const cv::Size ts = cv::getTextSize(text, kFont, 0.6, 2, &base);
  const int x = (frame.cols - ts.width) / 2;
  const int y = frame.rows - 24;
  blendRect(frame, cv::Rect(x - 12, y - ts.height - 12, ts.width + 24, ts.height + 22), kPanel,
            0.6 * alpha);
  cv::Mat layer = frame.clone();
  cv::putText(layer, text, {x, y}, kFont, 0.6, kBone, 2, cv::LINE_AA);
  cv::addWeighted(layer, alpha, frame, 1.0 - alpha, 0.0, frame);
}

void drawActiveRegion(cv::Mat& frame, float margin) {
  const int mx = static_cast<int>(margin * static_cast<float>(frame.cols));
  const int my = static_cast<int>(margin * static_cast<float>(frame.rows));
  cv::rectangle(frame, cv::Rect(mx, my, frame.cols - 2 * mx, frame.rows - 2 * my), kBone, 1);
}

void FrameStats::addFrame(double tMs, double inferenceMs) {
  times_.push_back(tMs);
  inference_.push_back(inferenceMs);
  while (times_.size() > 2 && times_.back() - times_.front() > 2000.0) {
    times_.pop_front();
    inference_.pop_front();
  }
}

double FrameStats::fps() const {
  if (times_.size() < 2) return 0.0;
  const double span = times_.back() - times_.front();
  return span > 0.0 ? 1000.0 * static_cast<double>(times_.size() - 1) / span : 0.0;
}

double FrameStats::avgInferenceMs() const {
  if (inference_.empty()) return 0.0;
  double sum = 0.0;
  for (double v : inference_) sum += v;
  return sum / static_cast<double>(inference_.size());
}

}  // namespace ht::ui
