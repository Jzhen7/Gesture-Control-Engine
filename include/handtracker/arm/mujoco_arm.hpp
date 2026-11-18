// Simulated SO-101 arm running in the MuJoCo physics engine.
//
// MuJoCo keeps two structs:
//   mjModel  the robot description loaded from XML (links, joints, motors) - constant
//   mjData   the changing state (joint angles, velocities, motor commands)
// Each mj_step() advances physics by one timestep (5 ms for this model). The
// arm's motors are *position servos*: we write the joint angle we want into
// mjData::ctrl and the simulated servo drives the joint there, like the real
// STS3215 servos on the physical SO-101.
#pragma once

#include <array>
#include <string>
#include <vector>

#include "handtracker/arm/arm_backend.hpp"
#include "handtracker/core/config.hpp"

// Forward declarations so users of this header don't need MuJoCo's headers.
struct mjModel_;
struct mjData_;

namespace ht::arm {

class MujocoArm final : public ArmBackend {
 public:
  /// Loads the MuJoCo scene. Throws std::runtime_error if the model is missing or invalid.
  explicit MujocoArm(const ArmSettings& settings);
  ~MujocoArm() override;

  MujocoArm(const MujocoArm&) = delete;
  MujocoArm& operator=(const MujocoArm&) = delete;

  void setTarget(const Vec3& position) override;
  void setGripperOpening(float opening) override;
  void setWristTarget(const WristTarget& wrist) override;
  void advance(double seconds) override;
  ArmState state() const override;
  std::string name() const override;

  struct IkResult {
    double positionError = 0.0;  ///< Remaining fingertip error, metres.
    double pitch = 0.0;          ///< Pitch the solution actually has, radians.
    double pitchTarget = 0.0;    ///< Pitch it aimed for after giving way (= request if reachable).
    bool pitchLimited = false;   ///< The requested pitch wasn't reachable here.
  };

  /// Solves inverse kinematics from the current pose without moving anything.
  /// Position first; if `wrist.pitch` is set, pitch second (in the spare joint
  /// motion that doesn't move the fingertip); roll is set directly.
  IkResult solveIk(const Vec3& target, const WristTarget& wrist,
                   std::vector<double>& jointsOut, int iterations = -1) const;
  /// Position only (the Dash 2 form). Returns the remaining fingertip error in metres.
  double solveIk(const Vec3& target, std::vector<double>& jointsOut) const;

  /// Claw pitch for the given scratch/real state (radians).
  double pitchOf(const mjData_* data) const;

 private:
  /// One damped-least-squares solve from the current pose (no position check).
  IkResult solveCore(const Vec3& target, std::optional<float> pitch, float roll,
                     std::vector<double>& jointsOut, int iterations) const;

 public:

 private:
  static constexpr int kArmJoints = 5;  // pan, lift, elbow, wrist flex, wrist roll
  static constexpr int kIkJoints = 4;   // wrist roll doesn't move the fingertip

  ArmSettings settings_;
  mjModel_* model_ = nullptr;
  mjData_* data_ = nullptr;
  mjData_* scratch_ = nullptr;  ///< Separate state for IK "what if" calculations.
  int site_ = -1;               ///< Fingertip reference point ("gripperframe").
  std::array<int, kArmJoints> qposAdr_{};
  std::array<int, kArmJoints> dofAdr_{};
  std::array<int, kArmJoints> joint_{};
  std::array<int, kArmJoints> actuator_{};
  int gripperJoint_ = -1;
  int gripperActuator_ = -1;
  std::vector<int> skeletonBodies_;
  Vec3 target_;
  float opening_ = 1.f;  ///< Commanded jaw opening, 0..1.
  WristTarget wrist_;
  bool pitchLimited_ = false;
  /// Last frame's IK solution. Each frame's solve starts from it rather than
  /// from the physical pose (which lags behind the servos), so the iterations
  /// add up across frames instead of restarting every time.
  std::vector<double> ikSeed_;
  int liftJoint_ = -1;   ///< shoulder_lift: its axis is the claw's pitch axis.
  double time_ = 0.0;
};

}  // namespace ht::arm
