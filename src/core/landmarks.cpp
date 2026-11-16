#include "handtracker/core/landmarks.hpp"

namespace ht {

Vec3 HandLandmarks::normalized(LandmarkId id) const {
  const Vec3& p = (*this)[id];
  return {p.x / imageWidth, p.y / imageHeight, p.z / imageWidth};
}

float HandLandmarks::palmSize() const {
  return distance((*this)[LandmarkId::Wrist], (*this)[LandmarkId::MiddleMcp]);
}

}  // namespace ht
