#include "handtracker/arm/workspace.hpp"

#include <algorithm>

namespace ht::arm {

Vec3 WorkspaceBox::clamp(const Vec3& p) const {
  return {std::clamp(p.x, min.x, max.x), std::clamp(p.y, min.y, max.y),
          std::clamp(p.z, min.z, max.z)};
}

bool WorkspaceBox::contains(const Vec3& p, float tol) const {
  return p.x >= min.x - tol && p.x <= max.x + tol && p.y >= min.y - tol && p.y <= max.y + tol &&
         p.z >= min.z - tol && p.z <= max.z + tol;
}

Vec3 handToWorkspace(const Vec3& hand, const WorkspaceBox& box) {
  const Vec3 c = box.center();
  const Vec3 half = (box.max - box.min) * 0.5f;
  const Vec3 p{c.x + hand.z * half.x,    // forward
               c.y - hand.x * half.y,    // user's right is the arm's -Y
               c.z + hand.y * half.z};   // up
  return box.clamp(p);
}

}  // namespace ht::arm
