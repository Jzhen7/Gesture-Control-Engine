// Runs the real MuJoCo simulation headless (no window) and checks the arm
// actually gets where it is told to go.
#include <algorithm>
#include <cmath>
#include <cstdio>
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

// ---------------------------------------------------------------------------
// Dash 4: claw orientation
// ---------------------------------------------------------------------------

TEST_CASE("sim: pitch reachability across the workspace (report)") {
  // Not a pass/fail test of the numbers themselves: it prints the table used
  // in docs/DASH-4_CLAW_ORIENTATION.md, and checks position always wins.
  const ArmSettings s = settings();
  MujocoArm arm(s);
  const WorkspaceBox box{s.workspaceMin, s.workspaceMax};
  std::cerr << "  pitch | reached (pos<2mm, pitch<2deg) /125 | worst pos mm | worst pitch err deg\n";
  double worstPosOverall = 0.0, freeWorst = 0.0;
  for (int deg = -90; deg <= 45; deg += 15) {
    int ok = 0;
    double worstPos = 0.0, worstPitch = 0.0;
    for (int i = 0; i < 5; ++i) {
      for (int j = 0; j < 5; ++j) {
        for (int k = 0; k < 5; ++k) {
          const Vec3 f{static_cast<float>(i) / 4.f, static_cast<float>(j) / 4.f,
                       static_cast<float>(k) / 4.f};
          const Vec3 t{box.min.x + (box.max.x - box.min.x) * f.x,
                       box.min.y + (box.max.y - box.min.y) * f.y,
                       box.min.z + (box.max.z - box.min.z) * f.z};
          WristTarget w;
          w.pitch = static_cast<float>(deg) * 3.14159265f / 180.f;
          std::vector<double> q;
          const auto r = arm.solveIk(t, w, q, 300);
          if (deg == -90) freeWorst = std::max(freeWorst, arm.solveIk(t, WristTarget{}, q, 300).positionError);
          const double pitchErr = std::fabs(r.pitch - *w.pitch) * 180.0 / 3.14159265;
          worstPos = std::max(worstPos, r.positionError);
          worstPitch = std::max(worstPitch, pitchErr);
          ok += (r.positionError < 0.002 && pitchErr < 2.0);
        }
      }
    }
    worstPosOverall = std::max(worstPosOverall, worstPos);
    char buf[160];
    std::snprintf(buf, sizeof(buf), "  %5d | %3d | %7.2f | %6.1f\n", deg, ok, worstPos * 1000.0,
                  worstPitch);
    std::cerr << buf;
  }
  std::cerr << "  position-only worst error: " << freeWorst * 1000.0 << " mm\n";
  CHECK(worstPosOverall < 0.005);  // position is never sacrificed for pitch
}


namespace {
constexpr float kDegSim = 3.14159265f / 180.f;
void run(MujocoArm& arm, double seconds) {
  for (int i = 0; i < static_cast<int>(seconds * 30.0); ++i) arm.advance(1.0 / 30.0);
}
}  // namespace

TEST_CASE("sim: claw holds the commanded pitch at the workspace centre") {
  const ArmSettings s = settings();
  MujocoArm arm(s);
  const WorkspaceBox box{s.workspaceMin, s.workspaceMax};
  arm.setTarget(box.center());
  for (float deg : {-35.f, -60.f, -15.f}) {
    WristTarget w;
    w.pitch = deg * kDegSim;
    arm.setWristTarget(w);
    run(arm, 1.5);
    const ArmState st = arm.state();
    std::cerr << "  pitch " << deg << " -> " << st.pitch / kDegSim << " deg, reach error "
              << (st.gripper - box.center()).norm() * 1000.f << " mm\n";
    CHECK_NEAR(st.pitch, *w.pitch, 3 * kDegSim);
    CHECK((st.gripper - box.center()).norm() < 0.01f);
  }
}

TEST_CASE("sim: roll turns the claw about its own pointing axis, right-hand rule") {
  MujocoArm arm(settings());
  WristTarget w;
  w.pitch = -35 * kDegSim;
  arm.setWristTarget(w);
  run(arm, 1.0);
  const float pitchBefore = arm.state().pitch;
  const Vec3 up0 = arm.state().clawUp;
  w.roll = 45 * kDegSim;
  arm.setWristTarget(w);
  run(arm, 1.0);
  const ArmState st = arm.state();
  CHECK_NEAR(st.roll, 45 * kDegSim, 2 * kDegSim);
  // Physically: the jaw direction turned +45 deg about the pointing axis
  // (right-hand rule), not just the joint reading.
  const float turned = std::atan2(dot(cross(up0, st.clawUp), st.clawAxis), dot(up0, st.clawUp));
  std::cerr << "  roll +45 deg turned the jaws " << turned / kDegSim << " deg about the claw axis\n";
  CHECK_NEAR(turned, 45 * kDegSim, 3 * kDegSim);
  CHECK_NEAR(st.pitch, pitchBefore, 3 * kDegSim);  // rolling doesn't change pitch
}

TEST_CASE("sim: out-of-range roll is clamped to the joint") {
  MujocoArm arm(settings());
  WristTarget w;
  w.roll = 4.f;  // beyond the ~157 deg joint limit
  arm.setWristTarget(w);
  run(arm, 2.0);
  CHECK(arm.state().roll < 2.8f);
}

TEST_CASE("sim: an unreachable pitch gives way, the position doesn't") {
  const ArmSettings s = settings();
  MujocoArm arm(s);
  const Vec3 nearTop{s.workspaceMin.x, 0.f, s.workspaceMax.z};  // close in and high up
  WristTarget w;
  w.pitch = -90 * kDegSim;  // straight down: not possible there
  std::vector<double> q;
  const auto r = arm.solveIk(nearTop, w, q, 300);
  std::cerr << "  asked -90 deg, got " << r.pitch / kDegSim << " deg, position error "
            << r.positionError * 1000.0 << " mm\n";
  CHECK(r.pitchLimited);
  CHECK(r.positionError < 0.005);
}

TEST_CASE("sim: tracking moving targets with moving pitch and roll") {
  const ArmSettings s = settings();
  MujocoArm arm(s);
  const WorkspaceBox box{s.workspaceMin, s.workspaceMax};
  unsigned seed = 7;
  auto rnd = [&]() {
    seed = seed * 1103515245u + 12345u;
    return static_cast<float>((seed >> 8) & 0xFFFF) / 65535.f;
  };
  double worstPos = 0.0, worstPitch = 0.0;
  for (int k = 0; k < 30; ++k) {
    const Vec3 target{box.min.x + (box.max.x - box.min.x) * rnd(),
                      box.min.y + (box.max.y - box.min.y) * rnd(),
                      box.min.z + (box.max.z - box.min.z) * rnd()};
    WristTarget w;
    w.pitch = (-60.f + 40.f * rnd()) * kDegSim;  // -60..-20 deg
    w.roll = (-90.f + 180.f * rnd()) * kDegSim;
    arm.setTarget(target);
    arm.setWristTarget(w);
    run(arm, 1.5);
    std::vector<double> q;
    const auto ik = arm.solveIk(target, w, q);
    const ArmState st = arm.state();
    worstPos = std::max(worstPos, static_cast<double>((st.gripper - target).norm()));
    if (!ik.pitchLimited) worstPitch = std::max(worstPitch, std::fabs(static_cast<double>(st.pitch - *w.pitch)));
  }
  std::cerr << "  worst reach " << worstPos * 1000.0 << " mm, worst pitch (where reachable) "
            << worstPitch / kDegSim << " deg\n";
  CHECK(worstPos < 0.01);
  CHECK(worstPitch < 5 * kDegSim);
}
