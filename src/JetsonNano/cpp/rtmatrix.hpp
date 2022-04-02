// rt_matrix.hpp

#pragma once

#include <cmath>

// Use the following single-header matrix library
// https://github.com/sgorsten/linalg
#include "linalg.h"

using namespace linalg::aliases;

struct Position {
  float x { 0.0 };
  float y { 0.0 };
  float z { 0.0 };
};

struct Orientation {
  float pitch { 0.0 };
  float roll  { 0.0 };
  float yaw   { 0.0 };
};

class RTMatrix {

  Orientation orientation_;
  Position    position_;

public:

  explicit RTMatrix(const Orientation& orientation, const Position& position) {
    update(orientation, position);
  }

  void update(const Orientation& orientation, const Position& position) {
    orientation_ = orientation;
    position_    = position;
  }

  // Rotation matrix around X (roll)
  // roll = np.radians(roll)
  auto X() -> float4x4 {
    return float4x4{
      { 1.0, 0.0, 0.0, 0.0 },
      { 0.0, std::cos(orientation_.roll), -std::sin(orientation_.roll), 0.0 },
      { 0.0, std::sin(orientation_.roll), std::cos(orientation_.roll),  0.0 },
      { 0.0, 0.0, 0.0, 1.0 }
    };
  }

  // Rotation matrix around Y (pitch)
  // pitch = np.radians(pitch)
  auto Y() -> float4x4 {
    return float4x4{
      { std::cos(orientation_.pitch),  0.0, std::sin(orientation_.pitch), 0.0 },
      { 0.0, 1.0, 0.0, 0.0 },
      { -std::sin(orientation_.pitch), 0.0, std::cos(orientation_.pitch), 0.0 },
      { 0.0, 0.0, 0.0, 1.0 }
    };
  }

  // Rotation matrix around Z (yaw)
  // yaw = np.radians(yaw)
  auto Z() -> float4x4 {
    return float4x4{
      { std::cos(orientation_.yaw), 0.0, -std::sin(orientation_.yaw), 0.0 },
      { std::sin(orientation_.yaw), 0.0, std::cos(orientation_.yaw),  0.0 },
      { 0.0, 0.0, 1.0, 0.0 },
      { 0.0, 0.0, 0.0, 1.0 }
    };
  }

  auto rotation() -> float4x4 {
    if ((FP_ZERO == std::fpclassify(orientation_.roll)) &&
        (FP_ZERO == std::fpclassify(orientation_.pitch)) &&
        (FP_ZERO == std::fpclassify(orientation_.yaw))) {
      return linalg::identity;
    }
    else {
      return mul(mul(X(), Y()), Z());
    }
  }

  auto rotate() -> float4x4 {
    const auto translation {
      float4x4 {
        { 1.0, 0.0, 0.0, position_.x },
        { 0.0, 1.0, 0.0, position_.y },
        { 0.0, 0.0, 1.0, position_.z },
        { 0.0, 0.0, 0.0, 1.0         }
      }
    };
    return mul(translation, rotation());
  }
};
