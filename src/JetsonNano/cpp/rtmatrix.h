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
  float roll  { 0.0 };
  float pitch { 0.0 };
  float yaw   { 0.0 };
};

using Coordinate = Position;

class RTMatrix {

  Orientation orientation_;
  Position    position_;
  Coordinate  coordinate_;

public:

  explicit RTMatrix(const Orientation& orientation, const Position& position, const Coordinate& coordinate) {
    update(orientation, position, coordinate);
  }

  void update(const Orientation& orientation, const Position& position, const Coordinate& coordinate) {
    orientation_ = orientation;
    position_    = position;
    coordinate_  = coordinate;
  }

  auto transform() -> float4 {
    return mul(rotation(), translation(), float4 { coordinate_.x, coordinate_.y, coordinate_.z, 1.0 });
  }

private:

  // Rotation matrix around X (roll)
  // roll = np.radians(roll)
  auto Rx() -> float4x4 {
    return float4x4{
      { 1.0, 0.0, 0.0, 0.0 },
      { 0.0, std::cos(orientation_.roll),  -std::sin(orientation_.roll), 0.0 },
      { 0.0, std::sin(orientation_.roll), std::cos(orientation_.roll), 0.0 },
      { 0.0, 0.0, 0.0, 1.0 }
    };
  }

  // Rotation matrix around Y (pitch)
  // pitch = np.radians(pitch)
  auto Ry() -> float4x4 {
    return float4x4{
      { std::cos(orientation_.pitch), 0.0, std::sin(orientation_.pitch), 0.0 },
      { 0.0, 1.0, 0.0, 0.0 },
      { -std::sin(orientation_.pitch), 0.0, std::cos(orientation_.pitch),  0.0 },
      { 0.0, 0.0, 0.0, 1.0 }
    };
  }

  // Rotation matrix around Z (yaw)
  // yaw = np.radians(yaw)
  auto Rz() -> float4x4 {
    return float4x4{
      { std::cos(orientation_.yaw),  -std::sin(orientation_.yaw), 0.0, 0.0 },
      { std::sin(orientation_.yaw), std::cos(orientation_.yaw), 0.0, 0.0 },
      { 0.0, 0.0, 1.0, 0.0 },
      { 0.0, 0.0, 0.0, 1.0 }
    };
  }

  auto rotation() -> float4x4 {
    if (orientation_.roll != 0 || orientation_.pitch != 0 || orientation_.yaw != 0) {
      return mul(transpose(Rx()), transpose(Ry()), transpose(Rz()));
    }
    else {
      return linalg::identity;
    }
  }

  auto translation() -> float4x4 {
    return float4x4 {
      { 1.0, 0.0, 0.0, position_.x },
      { 0.0, 1.0, 0.0, position_.y },
      { 0.0, 0.0, 1.0, position_.z },
      { 0.0, 0.0, 0.0, 1.0         }
    };
  }
};
