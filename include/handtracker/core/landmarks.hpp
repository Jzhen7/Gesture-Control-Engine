// Hand landmark data model shared by every layer of the project.
//
// Landmarks follow the 21-point MediaPipe hand topology. Image-space points are
// stored in *pixels* of the frame they were detected in (x right, y down), with
// z being a relative depth expressed in roughly the same scale as x. Keeping the
// frame size alongside the points lets consumers work in either pixels or
// normalised [0, 1] coordinates without losing the aspect ratio.
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <utility>

namespace ht {

struct Vec3 {
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;

  constexpr Vec3() = default;
  constexpr Vec3(float x_, float y_, float z_ = 0.f) : x(x_), y(y_), z(z_) {}

  constexpr Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  constexpr Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  constexpr Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
  constexpr Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }

  float norm() const { return std::sqrt(x * x + y * y + z * z); }
};

constexpr float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

constexpr Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

inline float distance(const Vec3& a, const Vec3& b) { return (a - b).norm(); }

inline Vec3 normalized(const Vec3& v) {
  const float n = v.norm();
  return n > 1e-9f ? v / n : v;
}

/// Index of each of the 21 hand landmarks (MediaPipe ordering).
enum class LandmarkId : std::size_t {
  Wrist = 0,
  ThumbCmc, ThumbMcp, ThumbIp, ThumbTip,
  IndexMcp, IndexPip, IndexDip, IndexTip,
  MiddleMcp, MiddlePip, MiddleDip, MiddleTip,
  RingMcp, RingPip, RingDip, RingTip,
  PinkyMcp, PinkyPip, PinkyDip, PinkyTip,
};

inline constexpr std::size_t kNumLandmarks = 21;

/// Bones of the hand skeleton, for drawing.
inline constexpr std::array<std::pair<int, int>, 21> kHandConnections{{
    {0, 1}, {1, 2}, {2, 3}, {3, 4},          // thumb
    {0, 5}, {5, 6}, {6, 7}, {7, 8},          // index
    {5, 9}, {9, 10}, {10, 11}, {11, 12},     // middle
    {9, 13}, {13, 14}, {14, 15}, {15, 16},   // ring
    {13, 17}, {17, 18}, {18, 19}, {19, 20},  // pinky
    {0, 17},                                 // palm edge
}};

enum class Handedness { Unknown, Left, Right };

struct HandLandmarks {
  using Points = std::array<Vec3, kNumLandmarks>;

  Points image{};               ///< Pixel coordinates in the source frame.
  std::optional<Points> world;  ///< Metric 3D coordinates (metres), if the model provides them.
  float imageWidth = 1.f;
  float imageHeight = 1.f;
  Handedness handedness = Handedness::Unknown;
  float score = 0.f;  ///< Landmark model confidence in [0, 1].

  const Vec3& operator[](LandmarkId id) const { return image[static_cast<std::size_t>(id)]; }
  Vec3& operator[](LandmarkId id) { return image[static_cast<std::size_t>(id)]; }

  /// Landmark in normalised image coordinates: x / width, y / height.
  Vec3 normalized(LandmarkId id) const;

  /// Scale reference for the hand: wrist to middle-finger knuckle distance in pixels.
  /// Used to make thresholds independent of how far the hand is from the camera.
  float palmSize() const;
};

}  // namespace ht
