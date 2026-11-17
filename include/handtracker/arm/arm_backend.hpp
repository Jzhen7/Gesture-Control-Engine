// The one interface every robot arm implements: simulated (MuJoCo) today,
// real hardware later. Code above this line never knows which one it talks to.
//
// Coordinates are the arm's base frame in metres, the robotics convention:
//   X forward (away from the arm's base), Y left, Z up.
#pragma once

#include <string>
#include <vector>

#include "handtracker/core/landmarks.hpp"

namespace ht::arm {

struct ArmState {
  Vec3 gripper;                 ///< Where the gripper fingertips actually are.
  Vec3 target;                  ///< Where we asked them to go.
  bool gripperClosed = false;   ///< Commanded gripper state.
  float gripperOpening = 0.f;   ///< Measured jaw opening, 0 = shut, 1 = fully open.
  std::vector<double> joints;   ///< Joint angles in radians, base to wrist.
  std::vector<Vec3> skeleton;   ///< Joint positions base -> fingertips, for drawing.
  double time = 0.0;            ///< Simulated/elapsed seconds.
};

class ArmBackend {
 public:
  virtual ~ArmBackend() = default;

  /// Where the gripper should go (base frame, metres). The backend solves the joint angles.
  virtual void setTarget(const Vec3& position) = 0;
  virtual void setGripper(bool closed) = 0;
  /// Advance the arm by `seconds` of real time (runs physics for a simulator).
  virtual void advance(double seconds) = 0;
  virtual ArmState state() const = 0;
  virtual std::string name() const = 0;
};

}  // namespace ht::arm
