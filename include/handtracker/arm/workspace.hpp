// Maps the normalised hand cube onto the arm's reachable workspace box.
#pragma once

#include "handtracker/core/landmarks.hpp"

namespace ht::arm {

struct WorkspaceBox {
  Vec3 min;
  Vec3 max;

  Vec3 center() const { return (min + max) * 0.5f; }
  Vec3 clamp(const Vec3& p) const;
  bool contains(const Vec3& p, float tolerance = 0.f) const;
};

/// Hand cube (x right, y up, z towards camera; each -1..1) -> arm base frame
/// (X forward, Y left, Z up), clamped to the box.
///
/// The user is assumed to stand *behind* the arm, facing the same way it does
/// (the default viewer camera shows it like that), so:
///   hand right   -> arm -Y (its right)
///   hand up      -> arm +Z
///   hand forward -> arm +X
Vec3 handToWorkspace(const Vec3& hand, const WorkspaceBox& box);

}  // namespace ht::arm
