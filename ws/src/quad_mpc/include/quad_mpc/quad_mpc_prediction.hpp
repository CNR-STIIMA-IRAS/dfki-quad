#pragma once
#include <array>

#include <Eigen/Core>
#include <Eigen/Geometry>

struct MPCPredictionInterface {
  virtual ~MPCPredictionInterface() = default;
};

template<
  size_t PREDICTION_HORIZON, size_t STATE_SIZE
>
struct MPCPrediction : public MPCPredictionInterface {
  std::array<Eigen::Quaterniond, PREDICTION_HORIZON + 1> orientation;
  std::array<Eigen::Vector3d, PREDICTION_HORIZON + 1> position;
  std::array<Eigen::Vector3d, PREDICTION_HORIZON + 1> angular_velocity;
  std::array<Eigen::Vector3d, PREDICTION_HORIZON + 1> linear_velocity;
  std::array<Eigen::Matrix<double, STATE_SIZE, 1, Eigen::ColMajor>, PREDICTION_HORIZON + 1> raw_data;
};