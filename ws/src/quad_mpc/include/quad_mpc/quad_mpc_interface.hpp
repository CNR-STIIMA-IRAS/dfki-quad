#ifndef MPC_INTERFACE_HPP
#define MPC_INTERFACE_HPP

#include <common/model_interface.hpp>
#include <memory>

#include "gait_controller/gait_sequence.hpp"
#include "quad_mpc/quad_mpc_prediction.hpp"
#include "quad_mpc/wrench_sequence.hpp"

namespace quad_mpc {

struct SolverInformation {
  bool success;
  int return_code;
  double total_solver_time;

  double acados_solve_QP_time;
  double acados_condensing_time;
  double acados_interface_time;
  int acados_num_iter;
  int acados_t_computed;

  double qp_objective_value;
  double qp_residuals[4]; //inf norm res: stat, dyn, ineq, comp
};

class MPCInterface {
 protected:
  MPCInterface() = default;  // protected, as there cant be any Object from an Interface
 public:
  virtual void UpdateState(const StateInterface &quad_state) = 0;
  virtual void UpdateModel(const ModelInterface &quad_model) = 0;
  virtual void UpdateGaitSequence(const GaitSequence &gait_sequence) = 0;
  virtual void GetWrenchSequence(WrenchSequenceInterface* wrench_sequence,
                                 MPCPredictionInterface* state_prediction,
                                 SolverInformation &solver_information) = 0;
  virtual ~MPCInterface() = default;
};

}  // namespace quad_mpc

#endif  // MPC_INTERFACE_HPP