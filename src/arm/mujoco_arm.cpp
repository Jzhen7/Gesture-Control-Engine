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
  liftJoint_ = joint_[1];
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

void MujocoArm::setWristTarget(const WristTarget& wrist) { wrist_ = wrist; }

double MujocoArm::pitchOf(const mjData_* d) const {
  // Pitch is measured in the arm's vertical plane: the claw's pointing axis
  // (the gripperframe site's x axis) against the horizontal heading of the arm.
  // The heading comes from the shoulder_lift axis (the arm's "left"), not from
  // the claw itself, so the angle stays defined when the claw points straight down.
  const mjtNum* R = d->site_xmat + 9 * site_;  // row-major; column 0 = pointing axis
  const double a[3] = {R[0], R[3], R[6]};
  const mjtNum* l = d->xaxis + 3 * liftJoint_;  // arm's left
  const double h[2] = {l[1], -l[0]};            // heading = left x up (horizontal)
  const double forward = a[0] * h[0] + a[1] * h[1];
  return std::atan2(a[2], forward);
}

double MujocoArm::solveIk(const Vec3& target, std::vector<double>& jointsOut) const {
  return solveIk(target, WristTarget{}, jointsOut).positionError;
}

MujocoArm::IkResult MujocoArm::solveIk(const Vec3& target, const WristTarget& wrist,
                                       std::vector<double>& jointsOut, int iterations) const {
  // Position must always win over pitch. The null-space step alone doesn't
  // guarantee it: an unreachable pitch parks a joint on its limit, and the
  // other three can't always reach the point without it (up to 190 mm off in
  // testing). So check the result against a solve with no pitch request (the
  // Dash 2/3 behaviour, which reaches the whole workspace), and if pitch cost
  // accuracy, search for the closest pitch that doesn't.
  const int iters = iterations >= 0 ? iterations : settings_.ikIterations;
  IkResult r = solveCore(target, wrist.pitch, wrist.roll, jointsOut, iters);
  if (!wrist.pitch) return r;
  r.pitchTarget = *wrist.pitch;

  constexpr double kTolerance = 0.001;  // 1 mm worse than the free solve is acceptable
  std::vector<double> freeQ;
  const IkResult free = solveCore(target, std::nullopt, wrist.roll, freeQ, iters);
  if (r.positionError <= free.positionError + kTolerance) return r;

  // Bisect between the free pitch (reachable) and the requested one (not).
  double good = free.pitch, bad = *wrist.pitch;
  IkResult best = free;
  best.pitchTarget = free.pitch;
  std::vector<double> bestQ = freeQ, q;
  for (int i = 0; i < settings_.ikPitchSearchSteps; ++i) {
    const double mid = 0.5 * (good + bad);
    const IkResult m = solveCore(target, static_cast<float>(mid), wrist.roll, q, iters);
    if (m.positionError <= free.positionError + kTolerance) {
      good = mid;
      best = m;
      best.pitchTarget = mid;
      bestQ = q;
    } else {
      bad = mid;
    }
  }
  best.pitchLimited = true;
  jointsOut = bestQ;
  return best;
}

MujocoArm::IkResult MujocoArm::solveCore(const Vec3& target, std::optional<float> pitch,
                                         float roll, std::vector<double>& jointsOut,
                                         int iters) const {
  // Damped least squares ("Levenberg-Marquardt") inverse kinematics with two
  // priorities:
  //   1. position  e = target - fingertip                  (3 numbers, 4 joints)
  //   2. pitch     only through the "spare" joint motion that doesn't move the
  //                fingertip (the null space of the position task), so it can
  //                never pull the fingertip off target.
  // Without a pitch target, priority 2 is the Dash 2 pull toward a comfortable
  // pose. Roll (wrist_roll) is set directly: it turns the claw about its own
  // pointing axis and is not part of the solve.
  //
  // Joint limits: a joint that would be pushed past its limit is parked on the
  // limit and the step is re-solved with the remaining joints (a small "active
  // set" loop). Clamping after the fact instead let an unreachable pitch drag
  // the fingertip up to 150 mm off target (Python prototype, Dash 4).
  const int nv = static_cast<int>(model_->nv);  // mjtSize is 64-bit in MuJoCo 3.x
  const int nq = static_cast<int>(model_->nq);
  std::vector<mjtNum> jacp(static_cast<std::size_t>(3 * nv));
  std::vector<mjtNum> jacr(static_cast<std::size_t>(3 * nv));
  for (int k = 0; k < nq; ++k) scratch_->qpos[k] = data_->qpos[k];
  if (ikSeed_.size() == static_cast<std::size_t>(kArmJoints)) {
    for (int i = 0; i < kArmJoints; ++i) {
      const auto u = static_cast<std::size_t>(i);
      scratch_->qpos[qposAdr_[u]] = ikSeed_[u];
    }
  }

  // Roll first, so the position solve accounts for the small offset it causes.
  {
    const auto u = static_cast<std::size_t>(kArmJoints - 1);
    const mjtNum* range = model_->jnt_range + 2 * joint_[u];
    // The wrist_roll joint axis points back along the claw, hence the minus.
    scratch_->qpos[qposAdr_[u]] = std::clamp(static_cast<double>(-roll), range[0], range[1]);
  }

  const double lambda2 = static_cast<double>(settings_.ikDamping) * settings_.ikDamping;
  const double pitchLambda2 = static_cast<double>(settings_.ikPitchDamping) * settings_.ikPitchDamping;
  constexpr int n = kIkJoints;
  IkResult result;
  for (int iter = 0; iter <= iters; ++iter) {
    mj_kinematics(model_, scratch_);
    mj_comPos(model_, scratch_);
    const mjtNum* tip = scratch_->site_xpos + 3 * site_;
    const double e[3] = {target.x - tip[0], target.y - tip[1], target.z - tip[2]};
    result.positionError = std::sqrt(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
    result.pitch = pitchOf(scratch_);
    const double pitchErr = pitch ? *pitch - result.pitch : 0.0;
    if (iter == iters) break;
    if (result.positionError < 1e-4 && std::fabs(pitchErr) < 1e-3) break;

    mj_jacSite(model_, scratch_, jacp.data(), jacr.data(), site_);
    double Jfull[3][n];
    double jPitch[n];  // how fast the pitch changes per joint
    const mjtNum* left = scratch_->xaxis + 3 * liftJoint_;
    for (int c = 0; c < n; ++c) {
      const auto col = static_cast<std::size_t>(dofAdr_[static_cast<std::size_t>(c)]);
      for (int r = 0; r < 3; ++r) Jfull[r][c] = jacp[static_cast<std::size_t>(r) * nv + col];
      // Turning about the arm's "left" axis tips the claw down, so pitch rate =
      // -(joint's angular velocity . left). Exact for this arm: lift, elbow and
      // wrist flex all turn about that axis; the pan turns about vertical (0).
      double w = 0.0;
      for (int r = 0; r < 3; ++r) w += jacr[static_cast<std::size_t>(r) * nv + col] * left[r];
      jPitch[c] = -w;
    }

    bool active[n] = {true, true, true, true};
    double dq[n] = {0, 0, 0, 0};
    for (int attempt = 0; attempt < n; ++attempt) {
      double J[3][n];
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < n; ++c) J[r][c] = active[c] ? Jfull[r][c] : 0.0;
      }
      // A = J J^T + lambda^2 I  (3x3, symmetric)
      double A[3][3];
      for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
          double s = 0.0;
          for (int k = 0; k < n; ++k) s += J[r][k] * J[c][k];
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
      double Jp[n][3];
      for (int k = 0; k < n; ++k) {
        for (int c = 0; c < 3; ++c) {
          Jp[k][c] = J[0][k] * Ainv[0][c] + J[1][k] * Ainv[1][c] + J[2][k] * Ainv[2][c];
        }
      }
      // Priority 1: dq1 = Jp e.   Null-space projector N = (I - Jp J), active joints only.
      double dq1[n];
      for (int k = 0; k < n; ++k) dq1[k] = active[k] ? Jp[k][0] * e[0] + Jp[k][1] * e[1] + Jp[k][2] * e[2] : 0.0;
      double N[n][n];
      for (int r = 0; r < n; ++r) {
        for (int c = 0; c < n; ++c) {
          const double JpJ = Jp[r][0] * J[0][c] + Jp[r][1] * J[1][c] + Jp[r][2] * J[2][c];
          N[r][c] = (active[r] && active[c]) ? (r == c ? 1.0 : 0.0) - JpJ : 0.0;
        }
      }

      // Priority 2, in the null space only.
      double dq2[n] = {0, 0, 0, 0};
      if (pitch) {
        // dq2 = N j^T (e2 - j dq1) / (j N j^T + mu^2): the smallest null-space
        // motion that fixes the pitch, damped so an unreachable pitch can't
        // command huge steps. The pitch error per iteration is capped too.
        const double step = settings_.ikPitchStep;
        const double e2 = std::clamp(pitchErr, -step, step);
        double jdq1 = 0.0, jNj = 0.0, Njt[n];
        for (int k = 0; k < n; ++k) jdq1 += jPitch[k] * dq1[k];
        for (int r = 0; r < n; ++r) {
          Njt[r] = 0.0;
          for (int c = 0; c < n; ++c) Njt[r] += N[r][c] * jPitch[c];
          jNj += jPitch[r] * Njt[r];
        }
        const double scale = (e2 - jdq1) / (jNj + pitchLambda2);
        for (int k = 0; k < n; ++k) dq2[k] = Njt[k] * scale;
      } else {
        // Dash 2: pull gently back toward the ready pose. Without this the arm
        // drifts against its joint limits and gets stuck (313 mm vs 4 mm worst error).
        double bias[n];
        for (int k = 0; k < n; ++k) {
          bias[k] = settings_.ikNullGain *
                    (kReadyPose[k] - scratch_->qpos[qposAdr_[static_cast<std::size_t>(k)]]);
        }
        for (int r = 0; r < n; ++r) {
          for (int c = 0; c < n; ++c) dq2[r] += N[r][c] * bias[c];
        }
      }

      // Would any active joint leave its range? Park it on the limit and re-solve.
      bool parked = false;
      for (int k = 0; k < n; ++k) {
        dq[k] = dq1[k] + dq2[k];
        if (!active[k]) continue;
        const auto u = static_cast<std::size_t>(k);
        const mjtNum* range = model_->jnt_range + 2 * joint_[u];
        const double next = scratch_->qpos[qposAdr_[u]] + dq[k];
        if (next < range[0] || next > range[1]) {
          scratch_->qpos[qposAdr_[u]] = std::clamp(next, range[0], range[1]);
          active[k] = false;
          parked = true;
        }
      }
      if (!parked) break;
      for (int k = 0; k < n; ++k) dq[k] = 0.0;  // re-solve with the parked joints removed
    }
    for (int k = 0; k < n; ++k) {
      if (!active[k]) continue;
      const auto u = static_cast<std::size_t>(k);
      const mjtNum* range = model_->jnt_range + 2 * joint_[u];
      scratch_->qpos[qposAdr_[u]] = std::clamp(scratch_->qpos[qposAdr_[u]] + dq[k], range[0], range[1]);
    }
  }

  jointsOut.resize(kArmJoints);
  for (int i = 0; i < kArmJoints; ++i) {
    jointsOut[static_cast<std::size_t>(i)] = scratch_->qpos[qposAdr_[static_cast<std::size_t>(i)]];
  }
  return result;
}

void MujocoArm::advance(double seconds) {
  // 1. Decide where each joint should be (IK), and tell the servos.
  std::vector<double> q;
  pitchLimited_ = solveIk(target_, wrist_, q).pitchLimited;
  ikSeed_ = q;
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
  st.pitch = static_cast<float>(pitchOf(data_));
  st.roll = static_cast<float>(-data_->qpos[qposAdr_[static_cast<std::size_t>(kArmJoints - 1)]]);  // axis points back along the claw
  st.wristTarget = wrist_;
  st.pitchLimited = pitchLimited_;
  const mjtNum* R = data_->site_xmat + 9 * site_;  // row-major: column 0 = x, column 2 = z
  st.clawAxis = {static_cast<float>(R[0]), static_cast<float>(R[3]), static_cast<float>(R[6])};
  st.clawUp = {static_cast<float>(R[2]), static_cast<float>(R[5]), static_cast<float>(R[8])};
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
