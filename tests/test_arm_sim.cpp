// Runs the real MuJoCo simulation headless (no window) and checks the arm
// actually gets where it is told to go.
#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "handtracker/arm/mujoco_arm.hpp"
#include "handtracker/arm/workspace.hpp"
#include "test_framework.hpp"

using namespace ht;
using namespace ht::arm;

namespace {

ArmSettings settings() {
  ArmSettings s;
  s.model = HT_TEST_ARM_MODEL;  // set by CMake to models/so101/scene.xml
  return s;
}

}  // namespace

TEST_CASE("sim: model loads and starts in the ready pose") {
  MujocoArm arm(settings());
  const ArmState st = arm.state();
  CHECK_EQ(st.joints.size(), static_cast<std::size_t>(5));
  CHECK(st.skeleton.size() >= 6u);
  CHECK(st.gripper.x > 0.2f);  // in front of the base
  CHECK(st.gripperOpening > 0.5f);
}

TEST_CASE("sim: reaches every corner and the centre of the workspace within 1 cm") {
  const ArmSettings s = settings();
  const WorkspaceBox box{s.workspaceMin, s.workspaceMax};
  MujocoArm arm(s);
  for (float fx : {0.f, 1.f}) {
    for (float fy : {0.f, 1.f}) {
      for (float fz : {0.f, 1.f}) {
        const Vec3 target{box.min.x + fx * (box.max.x - box.min.x),
                          box.min.y + fy * (box.max.y - box.min.y),
                          box.min.z + fz * (box.max.z - box.min.z)};
        arm.setTarget(target);
        for (int i = 0; i < 45; ++i) arm.advance(1.0 / 30.0);  // 1.5 s at 30 fps
        const double err = (arm.state().gripper - target).norm();
        if (err > 0.01) {
          std::cerr << "  target " << target.x << "," << target.y << "," << target.z
                    << " error " << err * 1000 << " mm\n";
        }
        CHECK(err < 0.01);
      }
    }
  }
  arm.setTarget(box.center());
  for (int i = 0; i < 45; ++i) arm.advance(1.0 / 30.0);
  const float centreErr = (arm.state().gripper - box.center()).norm();
  if (centreErr >= 0.01f) std::cerr << "  centre error " << centreErr * 1000 << " mm\n";
  CHECK(centreErr < 0.01f);
}

TEST_CASE("sim: long random sequence of targets never gets stuck") {
  // Regression test: without the IK null-space bias the arm drifted against a
  // joint limit after a few far targets and stopped reaching (300+ mm error).
  const ArmSettings s = settings();
  const WorkspaceBox box{s.workspaceMin, s.workspaceMax};
  MujocoArm arm(s);
  unsigned seed = 12345;
  auto rnd = [&seed] {
    seed = seed * 1103515245u + 12345u;
    return static_cast<float>((seed >> 8) & 0xFFFF) / 65535.f;
  };
  double worst = 0.0;
  for (int n = 0; n < 40; ++n) {
    const Vec3 target{box.min.x + rnd() * (box.max.x - box.min.x),
                      box.min.y + rnd() * (box.max.y - box.min.y),
                      box.min.z + rnd() * (box.max.z - box.min.z)};
    arm.setTarget(target);
    for (int i = 0; i < 45; ++i) arm.advance(1.0 / 30.0);
    worst = std::max(worst, static_cast<double>((arm.state().gripper - target).norm()));
  }
  if (worst >= 0.01) std::cerr << "  worst error " << worst * 1000 << " mm\n";
  CHECK(worst < 0.01);
}

TEST_CASE("sim: gripper closes and opens") {
  MujocoArm arm(settings());
  arm.setGripperOpening(0.f);
  for (int i = 0; i < 30; ++i) arm.advance(1.0 / 30.0);
  CHECK(arm.state().gripperOpening < 0.1f);
  arm.setGripperOpening(1.f);
  for (int i = 0; i < 30; ++i) arm.advance(1.0 / 30.0);
  CHECK(arm.state().gripperOpening > 0.9f);
}

TEST_CASE("sim: jaws settle at in-between openings") {
  MujocoArm arm(settings());
  for (float want : {0.25f, 0.5f, 0.75f}) {
    arm.setGripperOpening(want);
    for (int i = 0; i < 30; ++i) arm.advance(1.0 / 30.0);
    std::cerr << "  commanded " << want << " -> measured " << arm.state().gripperOpening << "\n";
    CHECK_NEAR(arm.state().gripperOpening, want, 0.05);
    CHECK_NEAR(arm.state().gripperCommand, want, 1e-6);
  }
}

TEST_CASE("sim: jaws go from open to shut in well under half a second") {
  MujocoArm arm(settings());
  for (int i = 0; i < 30; ++i) arm.advance(1.0 / 30.0);
  arm.setGripperOpening(0.f);
  double t = 0.0;
  while (arm.state().gripperOpening > 0.1f && t < 2.0) {
    arm.advance(0.005);
    t += 0.005;
  }
  std::cerr << "  open -> 10% open took " << t * 1000.0 << " ms (simulated)\n";
  CHECK(t < 0.5);
}

TEST_CASE("sim: out-of-range openings are clamped") {
  MujocoArm arm(settings());
  arm.setGripperOpening(7.f);
  CHECK_NEAR(arm.state().gripperCommand, 1, 1e-6);
  arm.setGripperOpening(-3.f);
  CHECK_NEAR(arm.state().gripperCommand, 0, 1e-6);
}

TEST_CASE("sim: IK solve does not move the arm") {
  MujocoArm arm(settings());
  const Vec3 before = arm.state().gripper;
  std::vector<double> q;
  const double err = arm.solveIk({0.24f, 0.05f, 0.15f}, q);
  CHECK(err < 0.02);
  CHECK((arm.state().gripper - before).norm() < 1e-6f);
}

TEST_CASE("sim: a missing model gives a clear error") {
  ArmSettings s;
  s.model = "does/not/exist.xml";
  bool threw = false;
  try {
    MujocoArm arm(s);
  } catch (const std::runtime_error& e) {
    threw = std::string(e.what()).find("download_arm") != std::string::npos;
  }
  CHECK(threw);
}
