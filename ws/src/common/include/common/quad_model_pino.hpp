#pragma once

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/regressor.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/frame.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <rclcpp/rclcpp.hpp>

#include "model_interface.hpp"
#include "sequence_containers.hpp"

class QuadModelPino : public ModelInterface {
 private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  std::array<pinocchio::FrameIndex, N_LEGS> feet_frame_idxs_;
  std::array<std::array<pinocchio::JointIndex, N_JOINTS_PER_LEG>, N_LEGS> joint_indxs_;
  std::array<pinocchio::JointIndex, NUM_JOINTS> joint_indxs_flat;
  pinocchio::JointIndex base_link_idx_;
  std::array<std::array<pinocchio::JointIndex, N_JOINTS_PER_LEG>, N_LEGS> joint_frame_indxs_;
  Eigen::Vector<double, (1 + ModelInterface::NUM_JOINTS) * 10> dynamic_parameters_;

  pinocchio::FrameIndex imu_frame_id_;
  pinocchio::FrameIndex bellyb_frame_id_;

  typedef Eigen::Vector<double, 3 + 4 + NUM_JOINTS> QVec;
  typedef Eigen::Vector<double, 3 + 3 + NUM_JOINTS> VVec;
  typedef Eigen::Matrix<double, 6, 3 + 3 + NUM_JOINTS> FRAMEJac;

  QVec default_q_vec_;

  template <typename Derived>
  inline void SetJointConfigSpace(unsigned int foot_idx, const Eigen::MatrixBase<Derived>& joint_val, QVec& q) const {
    for (unsigned int jdx = 0; jdx < N_JOINTS_PER_LEG; jdx++) {
      q(model_.idx_qs[joint_indxs_[foot_idx][jdx]]) = joint_val(jdx);
    }
  }

  template <typename Derived>
  inline void SetJointConfigSpace(const Eigen::MatrixBase<Derived>& joint_val, QVec& q) const {
    // First 7 elements from joint_val to first 7 in q, floating base
    q.segment<7>(0) = joint_val.segment(0, 7);
    for (unsigned int i = 0; i < ModelInterface::N_LEGS; i++) {
      SetJointConfigSpace(
          i, joint_val.segment(ModelInterface::N_JOINTS_PER_LEG * i, ModelInterface::N_JOINTS_PER_LEG), q);
    }
  }

  template <typename Derived>
  inline void SetJointTangentConfigSpace(unsigned int foot_idx,
                                         const Eigen::MatrixBase<Derived>& joint_val,
                                         VVec& q) const {
    for (unsigned int jdx = 0; jdx < N_JOINTS_PER_LEG; jdx++) {
      q(model_.idx_vs[joint_indxs_[foot_idx][jdx]]) = joint_val(jdx);
    }
  }

  template <typename Derived>
  inline void SetJointTangentConfigSpace(const Eigen::MatrixBase<Derived>& joint_val, VVec& q) const {
    q.segment<6>(0) = joint_val.segment(0, 6);
    for (unsigned int i = 0; i < ModelInterface::N_LEGS; i++) {
      SetJointTangentConfigSpace(
          i, joint_val.segment(ModelInterface::N_JOINTS_PER_LEG * i, ModelInterface::N_JOINTS_PER_LEG), q);
    }
  }

  inline Eigen::Vector<double, ModelInterface::N_JOINTS_PER_LEG> GetJointConfigSpace(const QVec& q,
                                                                                     unsigned int leg_idx) const {
    Eigen::Vector<double, ModelInterface::N_JOINTS_PER_LEG> ret;
    for (unsigned int i = 0; i < ModelInterface::N_JOINTS_PER_LEG; i++) {
      ret(i) = q(model_.idx_qs[joint_indxs_[leg_idx][i]]);
    }
    return ret;
  }

  inline Eigen::Vector<double, ModelInterface::N_JOINTS_PER_LEG> GetJointTangentConfigSpace(
      const VVec& v, unsigned int leg_idx) const {
    Eigen::Vector<double, ModelInterface::N_JOINTS_PER_LEG> ret;
    for (unsigned int i = 0; i < ModelInterface::N_JOINTS_PER_LEG; i++) {
      ret(i) = v(model_.idx_vs[joint_indxs_[leg_idx][i]]);
    }
    return ret;
  }

  void Initialize(const std::string& urdf_path,
                  const std::array<std::string, N_LEGS>& feet_frame_names,
                  const std::string& base_link_name,
                  const std::array<std::string, NUM_JOINTS>& joint_names,
                  const std::array<double, NUM_JOINTS>& joint_default_positions);

  /**
   * This will update the data_ struct to provide the newest model properties.
   */
  void UpdateModel();

 public:
  QuadModelPino(const std::string& urdf_path,
                const std::array<std::string, N_LEGS>& feet_frame_names,
                const std::string& base_link_name,
                const std::array<std::string, NUM_JOINTS>& joint_names,
                const std::array<double, NUM_JOINTS>& joint_default_positions);

  /**
   * Creates a quad model and automatically declares ROS 2 parameters to load the paramaters.
   * @param owning_node Node at which the parameters are declared
   */
  explicit QuadModelPino(rclcpp::Node& owning_node);

  void CalcFootForceVelocityInBodyFrame(
      int leg_index,
      const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>& joint_positions,
      const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>& joint_velocities,
      const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>& joint_accelerations,
      const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>& joint_torques,
      Eigen::Ref<Eigen::Vector3d> f_ee,
      Eigen::Ref<Eigen::Vector3d> v_ee) const override;
  void CalcFootForceVelocityBodyFrame(int leg_index,
                                      const StateInterface& state,
                                      Eigen::Vector3d& f_ee,
                                      Eigen::Vector3d& v_ee) const override;
  void CalcLegInverseKinematicsInBody(int leg_index,
                                      const Eigen::Vector3d& p_ee_B,
                                      const Eigen::Vector3d& joint_state_init_guess,
                                      Eigen::Vector3d& theta) const override;
  void CalcLegInverseKinematicsInLegFrame(int leg_index,
                                          const Eigen::Vector3d& foot_pos_legbase,
                                          const Eigen::Vector3d& joint_state_init_guess,
                                          Eigen::Vector3d& joint_state) const;
  void CalcLegDiffKinematicsBodyFrame(int leg_index,
                                      const StateInterface& state,
                                      Eigen::Vector3d& f_ee_goal,
                                      Eigen::Vector3d& v_ee_goal,
                                      Eigen::Vector3d& tau_goal,
                                      Eigen::Vector3d& qd_goal) const override;
  void CalcJacobianLegBase(int leg_index, Eigen::Vector3d joint_pos, Eigen::Matrix3d& Jac_legBaseToFoot) const override;
  void CalcFwdKinLegBody(int leg_indx,
                         const Eigen::Vector3d& joint_pos,
                         Eigen::Matrix4d& T_BodyToFoot,
                         Eigen::Vector3d& foot_pos_body) const override;
  Eigen::Vector3d CalcFootPositionInWorld(unsigned int foot_idx, const StateInterface& state) const override;
  Eigen::Vector3d CalcFootPositionInWorld(unsigned int foot_idx,
                                          const Eigen::Vector3d& body_pos,
                                          const Eigen::Quaterniond& body_orientation,
                                          const Eigen::Vector3d& joint_positions) const override;
  Eigen::Vector3d CalcFootPositionInBodyFrame(unsigned int foot_idx,
                                              const Eigen::Vector3d& joint_positions) const override;
  Eigen::Matrix3d GetInertia() const override;
  double GetInertia(const int row, const int column) const override;
  Eigen::Matrix3d getBaseInertia() const override;
  void getLegInertia(
      const std::array<Eigen::Vector<double, ModelInterface::N_JOINTS_PER_LEG>, ModelInterface::N_LEGS>& joint_pos,
      Eigen::Ref<Eigen::Matrix3d> leg_inertia) const override;
  double GetMass() const override;
  double GetLegMass(int foot_idx) const override;
  double getBaseMass() const override;
  double GetG() const override;
  Eigen::Translation3d GetBodyToIMU() const override;
  Eigen::Translation3d GetBodyToBellyBottom() const override;
  Eigen::Translation3d GetBodyToBellyBottom(unsigned int leg) const override;
  Eigen::Translation3d GetBodyToCOM() const override;
  Eigen::Translation3d GetBodyToLegCOM(int foot_idx) const override;
  Eigen::Translation3d GetBodyToLegBase(int foot_idx) const override;
  Eigen::Vector3d getBaseCOM() const override;
  void CalcBaseHeight(const std::array<bool, ModelInterface::N_LEGS>& feet_contacts,
                      const std::array<const Eigen::Vector3d, ModelInterface::N_LEGS>& joint_states,
                      const Eigen::Quaterniond& body_orientation,
                      double& base_height) const override;
  bool IsLyingDown(const std::array<const Eigen::Vector3d, ModelInterface::N_LEGS>& joint_states,
                   const std::array<double, ModelInterface::N_LEGS>& distance_threshold) const override;

  double ComputeKineticEnergy(int leg_index,
                              const Eigen::Vector3d& joint_pos,
                              const Eigen::Vector3d& joint_vel) const override;
  double ComputeEnergyDerivative(int leg_index,
                                 const Eigen::Vector3d& joint_vel,
                                 const Eigen::Vector3d& tau) const override;

  void ComputeMomentumSignal(const StateInterface& state,
                             Eigen::Vector<double, ModelInterface::NUM_JOINTS + 6>& momentum_signal) const override;
  void ComputeGeneralizedMomentum(
      const Eigen::Vector<double, ModelInterface::NUM_JOINTS + 7>& joint_pos,
      const Eigen::Vector<double, ModelInterface::NUM_JOINTS + 6>& joint_vel,
      Eigen::Vector<double, ModelInterface::NUM_JOINTS + 6>& generalized_momentum) const override;
  void ComputeEstimatedForces(int leg_index,
                              const Eigen::Vector<double, ModelInterface::N_JOINTS_PER_LEG>& tau_disturbed,
                              const Eigen::Vector<double, ModelInterface::N_JOINTS_PER_LEG>& joint_pos,
                              Eigen::Vector3d& estimated_forces) const override;
  void ComputeRegressorMatrix(
      const StateInterface& state,
      Eigen::Matrix<double, ModelInterface::NUM_JOINTS + 6, (ModelInterface::NUM_JOINTS + 1) * 10>& regressor)
      const override;

  void SetInertia(const Eigen::Matrix3d& inertia_tensor) override;
  void SetInertia(const int row, const int column, const double value) override;
  void SetMass(double mass) override;
  void SetCOM(const Eigen::Vector3d& com) override;
  void SetCOM(const int idx, const double val) override;
  QuadModelPino& operator=(const ModelInterface& other) override;
  const Eigen::Vector<double, (1 + ModelInterface::NUM_JOINTS) * 10>& GetAllDynamicParameters() override;
};
