#pragma once
#include <array>

#include "Eigen/Core"
#include "common/constants.hpp"

struct JointTorqueVelocityPositionCommands {
  std::array<std::array<double, N_JOINTS_PER_LEG>, N_LEGS> torque;
  std::array<std::array<double, N_JOINTS_PER_LEG>, N_LEGS> position;
  std::array<std::array<double, N_JOINTS_PER_LEG>, N_LEGS> velocity;
};

struct JointTorqueCommands {
  std::array<std::array<double, N_JOINTS_PER_LEG>, N_LEGS> torque;
};

struct CartesianCommands {
  std::array<Eigen::Vector3d, N_LEGS> position;
  std::array<Eigen::Vector3d, N_LEGS> velocity;
  std::array<Eigen::Vector3d, N_LEGS> force;
};
