#ifndef QUAD_MPC_WRENCH_SEQUENCE_HPP
#define QUAD_MPC_WRENCH_SEQUENCE_HPP

#include <array>

#include <Eigen/Dense>

namespace quad_mpc {

struct WrenchSequenceInterface {
  virtual ~WrenchSequenceInterface() = default;
};

template<size_t N_LEGS, size_t PREDICTION_HORIZON>
struct WrenchSequence : public WrenchSequenceInterface {
  std::array<std::array<Eigen::Vector3d, N_LEGS>, PREDICTION_HORIZON> forces;
};

}  // namespace quad_mpc

#endif // QUAD_MPC_WRENCH_SEQUENCE_HPP