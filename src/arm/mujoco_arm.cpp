#include "handtracker/arm/mujoco_arm.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <mujoco/mujoco.h>

namespace ht::arm {
namespace {

constexpr const char* kArmJointNames[] = {"shoulder_pan", "shoulder_lift", "elbow_flex",
                                          "wrist_flex", "wrist_roll"};
constexpr const char* kSkeletonBodies[] = {"base", "shoulder", "upper_arm", "lower_arm", "wrist",
                                           "gripper"};
/// A comfortable "elbow bent" starting pose (radians), instead of fully stretched out.
constexpr double kReadyPose[] = {0.0, -0.3, 0.6, 0.6, 0.0};

int requireId(const mjModel* m, int type, const char* name) {
  const int id = mj_name2id(m, type, name);
  if (id < 0) throw std::runtime_error(std::string("arm model has no '") + name + "'");
  return id;
}

Vec3 vec(const mjtNum* p) {
  return {static_cast<float>(p[0]), static_cast<float>(p[1]), static_cast<float>(p[2])};
}

}  // namespace

MujocoArm::MujocoArm(const ArmSettings& settings) : settings_(settings) {
  char error[1024] = "";
  model_ = mj_loadXML(settings_.model.c_str(), nullptr, error, sizeof(error));
  if (!model_) {
    throw std::runtime_error("could not load arm model '" + settings_.model + "': " + error +
                             " (run scripts/download_arm.ps1 or .sh)");
  }
  data_ = mj_makeData(model_);
  scratch_ = mj_makeData(model_);

  site_ = requireId(model_, mjOBJ_SITE, "gripperframe");
  for (int i = 0; i < kArmJoints; ++i) {
    joint_[static_cast<std::size_t>(i)] = requireId(model_, mjOBJ_JOINT, kArmJointNames[i]);
    actuator_[static_cast<std::size_t>(i)] = requireId(model_, mjOBJ_ACTUATOR, kArmJointNames[i]);
    qposAdr_[static_cast<std::size_t>(i)] = model_->jnt_qposadr[joint_[static_cast<std::size_t>(i)]];
    dofAdr_[static_cast<std::size_t>(i)] = model_->jnt_dofadr[joint_[static_cast<std::size_t>(i)]];
  }
  gripperJoint_ = requireId(model_, mjOBJ_JOINT, "gripper");
  gripperActuator_ = requireId(model_, mjOBJ_ACTUATOR, "gripper");
  for (const char* b : kSkeletonBodies) skeletonBodies_.push_back(requireId(model_, mjOBJ_BODY, b));

  // Start in the ready pose, servos holding it, gripper open.
  mj_resetData(model_, data_);
  for (int i = 0; i < kArmJoints; ++i) {
    const auto u = static_cast<std::size_t>(i);
    data_->qpos[qposAdr_[u]] = kReadyPose[i];
    data_->ctrl[actuator_[u]] = kReadyPose[i];
  }
  data_->qpos[model_->jnt_qposadr[gripperJoint_]] = settings_.gripperOpen;
  data_->ctrl[gripperActuator_] = settings_.gripperOpen;
  mj_forward(model_, data_);
  target_ = vec(data_->site_xpos + 3 * site_);
}

MujocoArm::~MujocoArm() {
  if (scratch_) mj_deleteData(scratch_);
  if (data_) mj_deleteData(data_);
  if (model_) mj_deleteModel(model_);
}

std::string MujocoArm::name() const { return std::string("MuJoCo ") + mj_versionString() + " SO-101"; }

void MujocoArm::setTarget(const Vec3& position) { target_ = position; }

void MujocoArm::setGripperOpening(float opening) { opening_ = std::clamp(opening, 0.f, 1.f); }

double MujocoArm::solveIk(const Vec3& target, std::vector<double>& jointsOut) const {
  // Damped least squares ("Levenberg-Marquardt") inverse kinematics:
  //   error e = target - fingertip                      (3 numbers)
  //   J       = how the fingertip moves per joint (3 x 4 Jacobian, from MuJoCo)
  //   dq      = J^T (J J^T + lambda^2 I)^-1 e           (joint change that best reduces e)
  // The damping lambda keeps steps sane near singular (fully stretched) poses.
  const int nv = static_cast<int>(model_->nv);  // mjtSize is 64-bit in MuJoCo 3.x
  std::vector<mjtNum> jacp(static_cast<std::size_t>(3 * nv));
  const int nq = static_cast<int>(model_->nq);
  for (int k = 0; k < nq; ++k) scratch_->qpos[k] = data_->qpos[k];

  const double lambda2 = static_cast<double>(settings_.ikDamping) * settings_.ikDamping;
  double errNorm = 0.0;
  for (int iter = 0; iter <= settings_.ikIterations; ++iter) {
    mj_kinematics(model_, scratch_);
    mj_comPos(model_, scratch_);
    const mjtNum* tip = scratch_->site_xpos + 3 * site_;
    const double e[3] = {target.x - tip[0], target.y - tip[1], target.z - tip[2]};
    errNorm = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
    if (iter == settings_.ikIterations || errNorm < 1e-4) break;

    mj_jacSite(model_, scratch_, jacp.data(), nullptr, site_);
    double J[3][kIkJoints];
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < kIkJoints; ++c) {
        J[r][c] = jacp[static_cast<std::size_t>(r * nv + dofAdr_[static_cast<std::size_t>(c)])];
      }
    }
    // A = J J^T + lambda^2 I  (3x3, symmetric)
    double A[3][3];
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        double s = 0.0;
        for (int k = 0; k < kIkJoints; ++k) s += J[r][k] * J[c][k];
        A[r][c] = s + (r == c ? lambda2 : 0.0);
      }
    }
    // Ainv = A^-1 via the adjugate (fine for 3x3).
    const double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                       A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                       A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    if (std::fabs(det) < 1e-12) break;
    double Ainv[3][3];
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        // Cofactor of A[c][r] (transposed), divided by det.
        const int r1 = (c + 1) % 3, r2 = (c + 2) % 3, c1 = (r + 1) % 3, c2 = (r + 2) % 3;
        Ainv[r][c] = (A[r1][c1] * A[r2][c2] - A[r1][c2] * A[r2][c1]) / det;
      }
    }
    // Jp = J^T Ainv  (4x3 damped pseudo-inverse)
    double Jp[kIkJoints][3];
    for (int k = 0; k < kIkJoints; ++k) {
      for (int c = 0; c < 3; ++c) Jp[k][c] = J[0][k] * Ainv[0][c] + J[1][k] * Ainv[1][c] + J[2][k] * Ainv[2][c];
    }
    // The arm has 4 joints but a position only needs 3, so one "spare" motion
    // doesn't move the fingertip. Use it to pull gently back toward the ready
    // pose (null-space bias). Without this the arm drifts against its joint
    // limits and gets stuck: 313 mm worst-case error vs 4 mm with it (tested).
    double bias[kIkJoints];
    for (int k = 0; k < kIkJoints; ++k) {
      bias[k] = settings_.ikNullGain *
                (kReadyPose[k] - scratch_->qpos[qposAdr_[static_cast<std::size_t>(k)]]);
    }
    for (int k = 0; k < kIkJoints; ++k) {
      const auto u = static_cast<std::size_t>(k);
      // dq = Jp e + (I - Jp J) bias
      double dq = Jp[k][0] * e[0] + Jp[k][1] * e[1] + Jp[k][2] * e[2] + bias[k];
      for (int j = 0; j < kIkJoints; ++j) {
        const double JpJ = Jp[k][0] * J[0][j] + Jp[k][1] * J[1][j] + Jp[k][2] * J[2][j];
        dq -= JpJ * bias[j];
      }
      const mjtNum* range = model_->jnt_range + 2 * joint_[u];
      scratch_->qpos[qposAdr_[u]] = std::clamp(scratch_->qpos[qposAdr_[u]] + dq, range[0], range[1]);
    }
  }

  jointsOut.resize(kArmJoints);
  for (int i = 0; i < kArmJoints; ++i) {
    jointsOut[static_cast<std::size_t>(i)] = scratch_->qpos[qposAdr_[static_cast<std::size_t>(i)]];
  }
  return errNorm;
}

void MujocoArm::advance(double seconds) {
  // 1. Decide where each joint should be (IK), and tell the servos.
  std::vector<double> q;
  solveIk(target_, q);
  for (int i = 0; i < kArmJoints; ++i) {
    const auto u = static_cast<std::size_t>(i);
    data_->ctrl[actuator_[u]] = q[u];
  }
  // Proportional jaw: a position servo, so the command is simply an angle
  // between the configured "closed" and "open" angles.
  data_->ctrl[gripperActuator_] =
      settings_.gripperClosed + opening_ * (settings_.gripperOpen - settings_.gripperClosed);

  // 2. Run physics for the elapsed time (capped, so a stall doesn't make it lurch).
  seconds = std::clamp(seconds, 0.0, 0.1);
  const int steps = static_cast<int>(std::lround(seconds / model_->opt.timestep));
  for (int s = 0; s < steps; ++s) mj_step(model_, data_);
  time_ = data_->time;
}

ArmState MujocoArm::state() const {
  ArmState st;
  st.gripper = vec(data_->site_xpos + 3 * site_);
  st.target = target_;
  st.gripperCommand = opening_;
  // Measured on the same scale as the command (1 = the configured open angle),
  // so "commanded 50%, measured 50%" means the jaws got there.
  const double g = data_->qpos[model_->jnt_qposadr[gripperJoint_]];
  const double span = settings_.gripperOpen - settings_.gripperClosed;
  st.gripperOpening = std::fabs(span) < 1e-9
                          ? 0.f
                          : static_cast<float>(std::clamp((g - settings_.gripperClosed) / span, 0.0, 1.0));
  for (int i = 0; i < kArmJoints; ++i) {
    st.joints.push_back(data_->qpos[qposAdr_[static_cast<std::size_t>(i)]]);
  }
  for (int b : skeletonBodies_) st.skeleton.push_back(vec(data_->xpos + 3 * b));
  st.skeleton.push_back(st.gripper);
  st.time = time_;
  return st;
}

}  // namespace ht::arm
