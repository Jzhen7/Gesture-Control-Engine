#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "app.hpp"
#include "handtracker/core/gesture.hpp"
#include "handtracker/ui/overlay.hpp"

namespace ht::app {
namespace {

std::atomic<bool> gInterrupted{false};

void onSignal(int) { gInterrupted = true; }

double nowMs() {
  using namespace std::chrono;
  return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

bool openCamera(cv::VideoCapture& cap, const CameraConfig& cam) {
#ifdef _WIN32
  // DirectShow opens in well under a second; the default MSMF backend can take several.
  if (!cap.open(cam.index, cv::CAP_DSHOW)) cap.open(cam.index, cv::CAP_ANY);
#else
  cap.open(cam.index, cv::CAP_ANY);
#endif
  if (!cap.isOpened()) return false;
  cap.set(cv::CAP_PROP_FRAME_WIDTH, cam.width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, cam.height);
  return true;
}

}  // namespace

int runCamera(App& app, vision::HandTracker& tracker, const AppConfig& cfg,
              const std::string& windowTitle) {
  cv::VideoCapture cap;
  if (!openCamera(cap, cfg.camera)) {
    std::cerr << "error: could not open camera " << cfg.camera.index << "\n";
    return 2;
  }
  std::signal(SIGINT, onSignal);

  if (cfg.preview.enabled) cv::namedWindow(windowTitle, cv::WINDOW_AUTOSIZE);
  bool pinned = false;

  ui::FrameStats stats;
  double lastReport = nowMs();
  int readFailures = 0;
  cv::Mat frame;
  std::cout << "running - press q or Esc in the preview window (or Ctrl+C) to quit\n";

  while (!gInterrupted) {
    if (!cap.read(frame) || frame.empty()) {
      if (++readFailures > 50) {
        std::cerr << "error: camera stopped delivering frames\n";
        break;
      }
      continue;
    }
    readFailures = 0;
    if (cfg.camera.mirror) cv::flip(frame, frame, 1);

    const double t = nowMs();
    const auto hands = tracker.process(frame);
    std::optional<HandLandmarks> hand;
    if (!hands.empty()) hand = hands.front();
    app.update(hand, t);

    const auto& timing = tracker.lastTiming();
    stats.addFrame(t, timing.palmMs + timing.landmarkMs);
    if (t - lastReport >= 5000.0) {  // Same periodic report as v1's benchmark script.
      std::printf("fps %.1f | inference %.1f ms\n", stats.fps(), stats.avgInferenceMs());
      std::fflush(stdout);
      lastReport = t;
    }

    if (app.quitRequested()) break;
    if (!cfg.preview.enabled) continue;

    app.draw(frame, hand, t);
    if (cfg.preview.showStats) {
      char buf[96];
      std::snprintf(buf, sizeof(buf), "%.0f fps  %.1f ms%s", stats.fps(), stats.avgInferenceMs(),
                    timing.ranDetector ? "" : "  (tracking)");
      cv::putText(frame, buf, {frame.cols - 230, frame.rows - 12}, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                  cv::Scalar(200, 200, 200), 1, cv::LINE_AA);
    }
    cv::imshow(windowTitle, frame);
    const int key = cv::waitKey(1) & 0xFF;
    if (!pinned && cfg.preview.topmost) {
      makeWindowTopmost(windowTitle);  // Window exists only after the first imshow.
      pinned = true;
    }
    if (key == 'q' || key == 27) break;
    if (cv::getWindowProperty(windowTitle, cv::WND_PROP_VISIBLE) < 1) break;
  }

  app.shutdown();
  if (cfg.preview.enabled) cv::destroyAllWindows();
  return 0;
}

int runImage(vision::HandTracker& tracker, const AppConfig& cfg, const std::string& inputPath,
             const std::string& outputPath, int expectHands, const std::string& expectGesture) {
  cv::Mat image = cv::imread(inputPath, cv::IMREAD_COLOR);
  if (image.empty()) {
    std::cerr << "error: could not read image " << inputPath << "\n";
    return 2;
  }
  const auto hands = tracker.process(image);
  std::cout << "image " << image.cols << "x" << image.rows << ", hands=" << hands.size() << "\n";
  for (std::size_t i = 0; i < hands.size(); ++i) {
    const auto& h = hands[i];
    const auto f = computeFingerState(h, cfg.classifier);
    std::cout << "  hand " << i << ": gesture=" << toString(classifyGesture(h, cfg.classifier))
              << " fingers=" << f.thumb << f.index << f.middle << f.ring << f.pinky
              << " handedness=" << (h.handedness == Handedness::Right ? "right" : "left")
              << " score=" << h.score << " palm_px=" << h.palmSize() << "\n";
    ui::drawHand(image, h);
  }
  if (!outputPath.empty()) {
    if (!cv::imwrite(outputPath, image)) {
      std::cerr << "error: could not write " << outputPath << "\n";
      return 2;
    }
    std::cout << "annotated image written to " << outputPath << "\n";
  }
  if (static_cast<int>(hands.size()) < expectHands) {
    std::cerr << "error: expected at least " << expectHands << " hand(s)\n";
    return 3;
  }
  if (!expectGesture.empty()) {
    const auto want = gestureFromString(expectGesture);
    const Gesture got = hands.empty() ? Gesture::None : classifyGesture(hands[0], cfg.classifier);
    if (!want || *want != got) {
      std::cerr << "error: expected gesture '" << expectGesture << "', got '" << toString(got)
                << "'\n";
      return 4;
    }
  }
  return 0;
}

}  // namespace ht::app
