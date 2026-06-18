#ifndef WBC_INTERFACE_HPP
#define WBC_INTERFACE_HPP

#include "common/feet_targets.hpp"
#include "common/model_interface.hpp"

struct WBCReturn {
  bool success;
  double qp_update_time;
  double qp_solve_time;
};

template <class JointCommandType>
class WBCInterface {
 protected:
  WBCInterface() = default;  // protected, as there cant be any Object from an Interface
  virtual ~WBCInterface() = default;
 public:
  typedef JointCommandType JOINT_COMMAND_TYPE;
  typedef std::array<Eigen::Vector3d, ModelInterface::N_LEGS> Wrenches;
  typedef std::array<bool, ModelInterface::N_LEGS> FootContact;

  virtual void UpdateState(const StateInterface& quad_state) = 0;
  virtual void UpdateModel(const ModelInterface& quad_model) = 0;
  virtual void UpdateFeetTarget(const FeetTargets& feet_targets) = 0;
  virtual void UpdateWrenches(const Wrenches& wrenches) = 0;
  virtual void UpdateFootContact(const FootContact& foot_contact) = 0;
  virtual void UpdateTarget(const Eigen::Quaterniond& orientation,
                            const Eigen::Vector3d& position,
                            const Eigen::Vector3d& lin_vel,
                            const Eigen::Vector3d& ang_vel) = 0;
  virtual WBCReturn GetJointCommand(JointCommandType& joint_command) = 0;
};

#endif  // WBC_INTERFACE_HPP
