#pragma once

#include <array>
#include <eigen3/Eigen/Core>

#include "common/constants.hpp"

struct FeetTargets {
  std::array<Eigen::Vector3d, N_LEGS> positions;
  std::array<Eigen::Vector3d, N_LEGS> velocities;
  std::array<Eigen::Vector3d, N_LEGS> accelerations;
};
