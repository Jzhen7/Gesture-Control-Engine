// handtracker - hand gesture control for the desktop.
//
//   handtracker [control|mouse|orientation] [options]
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "app.hpp"
#include "handtracker/core/async_system_control.hpp"
#include "handtracker/core/config.hpp"
#include "handtracker/core/system_control.hpp"
#include "handtracker/vision/hand_tracker.hpp"

namespace fs = std::filesystem;

namespace {

struct Options {
  std::string command = "control";
  std::string configPath;
  std::optional<int> camera;
  std::string modelsDir;
  std::string imagePath;
  std::string outputPath;
  int expectHands = 0;
  std::string expectGesture;
  bool dryRun = false;
  bool noPreview = false;
};

void printUsage() {
  std::cout <<
      "handtracker " HT_VERSION " - hand gesture control for the desktop\n"
      "\n"
      "usage: handtracker [command] [options]\n"
      "\n"
      "commands:\n"
      "  control       gestures trigger actions: media keys, volume, brightness... (default)\n"
      "  mouse         move the cursor with your hand, pinch to click\n"
      "  orientation   visualise the 3D rotation of your hand\n"
      "\n"
      "options:\n"
      "  -c, --config <file>    config file (default: config/handtracker.ini)\n"
      "      --camera <index>   camera index (overrides config)\n"
      "      --models <dir>     directory containing the .onnx models\n"
      "      --dry-run          log actions instead of performing them\n"
      "      --no-preview       run without the preview window\n"
      "      --image <file>     run detection on a still image and exit\n"
      "      --output <file>    with --image: write an annotated copy\n"
      "      --expect-hands <n> with --image: exit with code 3 if fewer hands are found\n"
      "      --expect-gesture <name>  with --image: exit with code 4 if the first hand differs\n"
      "  -h, --help            show this help\n"
      "      --version          show the version\n";
}

bool parseArgs(int argc, char** argv, Options& opt) {
  std::vector<std::string> args(argv + 1, argv + argc);
  for (std::size_t i = 0; i < args.size(); ++i) {
    const std::string& a = args[i];
    auto value = [&](std::string& out) {
      if (i + 1 >= args.size()) {
        std::cerr << "error: " << a << " needs a value\n";
        return false;
      }
      out = args[++i];
      return true;
    };
    std::string v;
    if (a == "-h" || a == "--help") {
      printUsage();
      std::exit(0);
    } else if (a == "--version") {
      std::cout << "handtracker " HT_VERSION "\n";
      std::exit(0);
    } else if (a == "-c" || a == "--config") {
      if (!value(opt.configPath)) return false;
    } else if (a == "--camera") {
      if (!value(v)) return false;
      try {
        opt.camera = std::stoi(v);
      } catch (const std::exception&) {
        std::cerr << "error: invalid camera index '" << v << "'\n";
        return false;
      }
    } else if (a == "--models") {
      if (!value(opt.modelsDir)) return false;
    } else if (a == "--image") {
      if (!value(opt.imagePath)) return false;
    } else if (a == "--output") {
      if (!value(opt.outputPath)) return false;
    } else if (a == "--expect-hands") {
      if (!value(v)) return false;
      try {
        opt.expectHands = std::stoi(v);
      } catch (const std::exception&) {
        std::cerr << "error: invalid number '" << v << "'\n";
        return false;
      }
    } else if (a == "--expect-gesture") {
      if (!value(opt.expectGesture)) return false;
    } else if (a == "--dry-run") {
      opt.dryRun = true;
    } else if (a == "--no-preview") {
      opt.noPreview = true;
    } else if (a == "control" || a == "mouse" || a == "orientation") {
      opt.command = a;
    } else {
      std::cerr << "error: unknown argument '" << a << "' (see --help)\n";
      return false;
    }
  }
  return true;
}

/// Finds a file relative to the working directory, the executable, or the
/// source tree layout (build/<config>/handtracker.exe -> repo root).
std::string resolve(const std::string& path, const fs::path& exeDir) {
  const fs::path p(path);
  if (p.is_absolute()) return path;
  const std::vector<fs::path> bases = {fs::current_path(), exeDir, exeDir / "..",
                                       exeDir / ".." / ".."};
  for (const auto& base : bases) {
    std::error_code ec;
    if (fs::exists(base / p, ec)) return fs::weakly_canonical(base / p, ec).string();
  }
  return path;
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  if (!parseArgs(argc, argv, opt)) return 1;

  std::error_code ec;
  const fs::path exeDir = fs::absolute(fs::path(argv[0]), ec).parent_path();

  const std::string configPath =
      resolve(opt.configPath.empty() ? "config/handtracker.ini" : opt.configPath, exeDir);
  ht::ConfigResult loaded = ht::loadConfigFile(configPath);
  for (const auto& w : loaded.warnings) std::cerr << "config: " << w << "\n";
  ht::AppConfig cfg = loaded.config;

  if (opt.camera) cfg.camera.index = *opt.camera;
  if (opt.noPreview) cfg.preview.enabled = false;
  if (!opt.modelsDir.empty()) {
    cfg.models.palmModel = (fs::path(opt.modelsDir) / fs::path(cfg.models.palmModel).filename()).string();
    cfg.models.landmarkModel =
        (fs::path(opt.modelsDir) / fs::path(cfg.models.landmarkModel).filename()).string();
  }
  cfg.models.palmModel = resolve(cfg.models.palmModel, exeDir);
  cfg.models.landmarkModel = resolve(cfg.models.landmarkModel, exeDir);

  ht::platformInit();

  std::unique_ptr<ht::vision::HandTracker> tracker;
  try {
    tracker = std::make_unique<ht::vision::HandTracker>(cfg.models);
  } catch (const std::exception& e) {
    std::cerr << "error: could not load the hand models: " << e.what() << "\n"
              << "  - Missing files? Run scripts/download_models.ps1 (Windows) or\n"
              << "    scripts/download_models.sh (Linux/macOS), or pass --models <dir>.\n"
              << "  - ONNX import errors usually mean OpenCV is older than 4.7.\n";
    return 2;
  }

  try {
    if (!opt.imagePath.empty()) {
      return ht::app::runImage(*tracker, cfg, opt.imagePath, opt.outputPath, opt.expectHands,
                               opt.expectGesture);
    }

    std::unique_ptr<ht::SystemControl> system;
    if (opt.dryRun) {
      system = std::make_unique<ht::DryRunSystemControl>(std::cout);
    } else {
      system = std::make_unique<ht::AsyncSystemControl>(ht::createPlatformSystemControl(std::cerr));
    }
    std::cout << "handtracker " HT_VERSION " | " << opt.command << " | backend: " << system->name()
              << " | camera " << cfg.camera.index << "\n";

    std::unique_ptr<ht::app::App> app;
    if (opt.command == "mouse") {
      app = ht::app::makeMouseApp(cfg, *system);
    } else if (opt.command == "orientation") {
      app = ht::app::makeOrientationApp(cfg);
    } else {
      app = ht::app::makeControlApp(cfg, *system);
    }
    return ht::app::runCamera(*app, *tracker, cfg, "Hand Tracker");
  } catch (const std::exception& e) {
    std::cerr << "fatal: " << e.what() << "\n";
    return 1;
  }
}
