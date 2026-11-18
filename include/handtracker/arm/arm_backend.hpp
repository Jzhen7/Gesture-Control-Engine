// The one interface every robot arm implements: simulated (MuJoCo) today,
// real hardware later. Code above this line never knows which one it talks to.
//
// Coordinates are the arm's base frame in metres, the robotics convention:
//   X forward (away from the arm's base), Y left, Z up.
#pragma once

#include <optional>
#include <string>
#include <vector>

#include "handtracker/core/landmarks.hpp"

namespace ht::arm {

/// Desired gripper orientation (Dash 4). The SO-101 has 5 joints, so of the
/// three ways a gripper can turn only two are free once its position is fixed:
///   pitch  how far the claw points up (+) or down (-) from horizontal, radians.
///          Measured in the arm's vertical plane; -pi/2 = pointing straight down.
///          Best effort: position always wins, so pitch gives way where the
///          requested angle can't be reached at that spot.
///   roll   turn about the claw's own pointing axis, radians, right-hand rule
///          (counter-clockwise looking back at the claw from in front of its tip).
///          0 = jaws open in the vertical plane. Driven directly by wrist_roll.
/// Yaw (pointing left/right) isn't listed: it is fixed by where the claw is,
/// because only the base pan joint turns the arm sideways.
struct WristTarget {
  std::optional<float> pitch;  ///< nullopt = let the arm choose (Dash 2/3 behaviour).
  float roll = 0.f;
};

struct ArmState {
  Vec3 gripper;                 ///< Where the gripper fingertips actually are.
  Vec3 target;                  ///< Where we asked them to go.
  float gripperCommand = 1.f;   ///< Commanded jaw opening, 0 = shut, 1 = configured "open".
  float gripperOpening = 0.f;   ///< Measured jaw opening, same scale (lags the command;
                                ///< stays above it when the jaws are blocked by an object).
  float pitch = 0.f;            ///< Measured claw pitch (radians, see WristTarget).
  float roll = 0.f;             ///< Measured claw roll (radians).
  WristTarget wristTarget;      ///< What was asked for.
  bool pitchLimited = false;    ///< The asked-for pitch isn't reachable at this position.
  Vec3 clawAxis{1.f, 0.f, 0.f}; ///< Direction the claw points (unit, base frame).
  Vec3 clawUp{0.f, 0.f, 1.f};   ///< Direction the moving jaw opens towards (unit).
  std::vector<double> joints;   ///< Joint angles in radians, base to wrist.
  std::vector<Vec3> skeleton;   ///< Joint positions base -> fingertips, for drawing.
  double time = 0.0;            ///< Simulated/elapsed seconds.
};

class ArmBackend {
 public:
  virtual ~ArmBackend() = default;

  /// Where the gripper should go (base frame, metres). The backend solves the joint angles.
  virtual void setTarget(const Vec3& position) = 0;
  /// How far open the jaws should be: 0 = closed, 1 = open, anything in between.
  /// (Was setGripper(bool) in Dash 2. Renamed rather than overloaded so old
  /// calls fail to compile instead of silently turning `true` = closed into 1 = open.)
  virtual void setGripperOpening(float opening) = 0;
  /// Orientation of the claw (Dash 4). Arms that can't do it may ignore it.
  virtual void setWristTarget(const WristTarget& wrist) = 0;
  /// Advance the arm by `seconds` of real time (runs physics for a simulator).
  virtual void advance(double seconds) = 0;
  virtual ArmState state() const = 0;
  virtual std::string name() const = 0;
};

}  // namespace ht::arm
