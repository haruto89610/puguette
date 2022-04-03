// rt.hpp

#pragma once

struct IK {

  static auto solve_R(const Position& coord, float coxa, float femur, float tibia) -> float3 {
    auto domain { get_domain(coord, coxa, femur, tibia) };
    auto gamma  { get_gamma(domain) };
    auto theta  { get_theta(coord, domain, -coxa) };
    auto alpha  { get_alpha(coord, domain, gamma, coxa, femur, tibia) };
    return float3{ -theta, alpha, gamma };
  }

  static auto solve_L(const Position& coord, float coxa, float femur, float tibia) -> float3 {
    auto domain { get_domain(coord, coxa, femur, tibia) };
    auto gamma  { get_gamma(domain) };
    auto theta  { get_theta(coord, domain, coxa) };
    auto alpha  { get_alpha(coord, domain, gamma, coxa, femur, tibia) };
    return float3{ -theta, alpha, gamma };
  }

private:

  static auto get_domain(const Position& coord, float coxa, float femur, float tibia) -> float {
    auto d { (coord.x*coord.x + coord.y*coord.y + coord.z*coord.z - coxa*coxa - femur*femur - tibia*tibia) / (2 * tibia * femur) };
    return checkdomain(d);
  }

  static auto get_gamma(float d) -> float {
    return std::atan2(-std::sqrt(1 - d*d), d);
  }

  static auto get_theta(const Position& coord, float d, float coxa) -> float {
    return -std::atan2(coord.z, coord.y) - std::atan2(std::sqrt(coord.y*coord.y + coord.z*coord.z - coxa*coxa), coxa);
  }

  static auto get_alpha(const Position& coord, float d, float gamma, float coxa, float femur, float tibia) -> float {
    return std::atan2(-coord.x, std::sqrt(coord.y*coord.y + coord.z*coord.z - coxa*coxa)) - std::atan2(tibia * sin(gamma), femur + tibia * std::cos(gamma));
  }

  static auto checkdomain(const float& domain) -> float {
    if (domain < -1.0) {
      return -0.99;
    }
    if (domain > 1.0) {
      return 0.99;
    }
    return domain;
  }
};
