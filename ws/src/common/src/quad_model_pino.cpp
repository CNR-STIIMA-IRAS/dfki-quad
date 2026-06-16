#include "quad_model_pino.hpp"

#include <filesystem>

#include "ament_index_cpp/get_package_share_path.hpp"

namespace {
std::string ResolveModelPath(const std::string& model_path) {
  namespace fs = std::filesystem;

  const fs::path path(model_path);
  if (path.is_absolute() || fs::exists(path)) {
    return path.string();
  }

  const std::string package_uri_prefix = "package://";
  if (model_path.rfind(package_uri_prefix, 0) == 0) {
    const auto package_path = model_path.substr(package_uri_prefix.size());
    const auto separator = package_path.find('/');
    if (separator != std::string::npos) {
      const auto package_name = package_path.substr(0, separator);
      const auto package_relative_path = package_path.substr(separator + 1);
      return (ament_index_cpp::get_package_share_path(package_name) / package_relative_path).string();
    }
  }

  const std::string workspace_source_prefix = "src/";
  if (model_path.rfind(workspace_source_prefix, 0) == 0) {
    const auto package_start = workspace_source_prefix.size();
    const auto separator = model_path.find('/', package_start);
    if (separator != std::string::npos) {
      const auto package_name = model_path.substr(package_start, separator - package_start);
      const auto package_relative_path = model_path.substr(separator + 1);
      return (ament_index_cpp::get_package_share_path(package_name) / package_relative_path).string();
    }
  }

  return model_path;
}
}  // namespace

QuadModelPino::QuadModelPino(rclcpp::Node& owning_node) {
  owning_node.declare_parameter("model_urdf", rclcpp::ParameterType::PARAMETER_STRING);
  owning_node.declare_parameter("model_feet_link_names", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  owning_node.declare_parameter("model_base_link_name", rclcpp::ParameterType::PARAMETER_STRING);
  owning_node.declare_parameter("model_joint_names", rclcpp::ParameterType::PARAMETER_STRING_ARRAY);
  owning_node.declare_parameter("model_joint_default_positions", rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY);

  assert(owning_node.get_parameter("model_feet_link_names").as_string_array().size() == ModelInterface::N_LEGS);
  assert(owning_node.get_parameter("model_joint_names").as_string_array().size() == ModelInterface::NUM_JOINTS);
  Initialize(owning_node.get_parameter("model_urdf").as_string(),
             to_array<ModelInterface::N_LEGS>(owning_node.get_parameter("model_feet_link_names").as_string_array()),
             owning_node.get_parameter("model_base_link_name").as_string(),
             to_array<ModelInterface::NUM_JOINTS>(owning_node.get_parameter("model_joint_names").as_string_array()),
             to_array<ModelInterface::NUM_JOINTS>(
                 owning_node.get_parameter("model_joint_default_positions").as_double_array()));
}

QuadModelPino::QuadModelPino(const std::string& urdf_path,
                             const std::array<std::string, N_LEGS>& feet_frame_names,
                             const std::string& base_link_name,
                             const std::array<std::string, NUM_JOINTS>& joint_names,
                             const std::array<double, NUM_JOINTS>& joint_default_positions) {
  Initialize(urdf_path, feet_frame_names, base_link_name, joint_names, joint_default_positions);
}

void QuadModelPino::Initialize(const std::string& urdf_path,
                               const std::array<std::string, N_LEGS>& feet_frame_names,
                               const std::string& base_link_name,
                               const std::array<std::string, NUM_JOINTS>& joint_names,
                               const std::array<double, NUM_JOINTS>& joint_default_positions) {
  const auto resolved_urdf_path = ResolveModelPath(urdf_path);
  pinocchio::urdf::buildModel(resolved_urdf_path, pinocchio::JointModelFreeFlyer(), model_);
  std::cout << "Loaded URDF model_ with " << model_.nq << " joints and " << model_.nv << " degrees of freedom."
            << std::endl;
  // Prepare data
  data_ = pinocchio::Data(model_);
  std::cout << "data mass " << data_.mass[0] << std::endl;
  std::cout << "data inertia " << data_.Ig.inertia().matrix() << std::endl;
  std::cout << "Joint names:" << std::endl;
  for (const auto& name : model_.names) {
    std::cout << "- " << name << std::endl;
  }
  // Get leg frames
  for (unsigned int i = 0; i < N_LEGS; i++) {
    if (!model_.existFrame(feet_frame_names[i])) {
      throw std::runtime_error("feet frame name not found in urdf: " + feet_frame_names[i]);
    }
    feet_frame_idxs_[i] = model_.getFrameId(feet_frame_names[i]);
  }
  assert(model_.existFrame(base_link_name));
  // Get joint idxs
  for (unsigned int leg_idx = 0; leg_idx < N_LEGS; leg_idx++) {
    for (unsigned int joint_idx = 0; joint_idx < N_JOINTS_PER_LEG; joint_idx++) {
      auto joint_name = joint_names[leg_idx * N_JOINTS_PER_LEG + joint_idx];
      assert(model_.existJointName(joint_name));
      assert(model_.existFrame(joint_name));
      auto joint_idx_val = model_.getJointId(joint_name);
      joint_indxs_[leg_idx][joint_idx] = joint_idx_val;
      joint_indxs_flat[leg_idx * N_JOINTS_PER_LEG + joint_idx] = joint_idx_val;
      joint_frame_indxs_[leg_idx][joint_idx] = model_.getFrameId(joint_name);
      std::cout << "Joint " << joint_idx_val << ": " << joint_name << std::endl;
    }
  }
  assert(model_.existFrame(base_link_name));
  base_link_idx_ = model_.getFrameId(base_link_name);
  assert(model_.existFrame("joint_imu"));
  imu_frame_id_ = model_.getFrameId("joint_imu");
  assert(model_.existFrame("joint_belly_bottom"));
  bellyb_frame_id_ = model_.getFrameId("joint_belly_bottom");
  QVec default_q_vec;
  default_q_vec.setZero();
  default_q_vec.segment<4>(3) = Eigen::Quaterniond::Identity().coeffs();
  default_q_vec.segment<ModelInterface::NUM_JOINTS>(7) =
      Eigen::Vector<double, NUM_JOINTS>::Map(joint_default_positions.data());
  SetJointConfigSpace(default_q_vec, default_q_vec_);
  // Now initialize the data_ as much as possible
  UpdateModel();
}

void QuadModelPino::UpdateModel() {
  VVec init_v_config;
  init_v_config.setZero();
  pinocchio::forwardKinematics(model_, data_, default_q_vec_);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::computeTotalMass(model_, data_);
  pinocchio::centerOfMass(model_, data_);
  pinocchio::ccrba(model_, data_, default_q_vec_, init_v_config);
  dynamic_parameters_.segment<10>(0) =
      model_.inertias[model_.frames[base_link_idx_].parentJoint].toDynamicParameters();
  for (unsigned int leg_idx = 0; leg_idx < ModelInterface::N_LEGS; leg_idx++) {
    for (unsigned int joint_indx = 0; joint_indx < ModelInterface::N_JOINTS_PER_LEG; joint_indx++) {
      dynamic_parameters_.segment<10>((1 + leg_idx * ModelInterface::N_JOINTS_PER_LEG + joint_indx) * 10) =
          model_.inertias[model_.frames[joint_frame_indxs_[leg_idx][joint_indx]].parentJoint].toDynamicParameters();
      std::cout << "Inertia element idx: " << model_.frames[joint_frame_indxs_[leg_idx][joint_indx]].parentJoint
                << std::endl;
    }
  }
}

// TODO: seperat velocity and force?
void QuadModelPino::CalcFootForceVelocityInBodyFrame(
    int leg_index,
    const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>& joint_positions,
    const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>& joint_velocities,
    const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>& joint_accelerations,
    const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>& joint_torques,
    Eigen::Ref<Eigen::Vector3d> f_ee,
    Eigen::Ref<Eigen::Vector3d> v_ee) const {
  pinocchio::Data data(model_);
  QVec q;
  VVec v;
  VVec a;
  VVec torque_measured;
  q.setZero();
  q.segment<4>(3) = Eigen::Quaterniond::Identity().coeffs();
  v.setZero();
  a.setZero();
  torque_measured.setZero();
  SetJointConfigSpace(leg_index, joint_positions, q);
  SetJointTangentConfigSpace(leg_index, joint_velocities, v);
  SetJointTangentConfigSpace(leg_index, joint_accelerations, a);
  SetJointTangentConfigSpace(leg_index, joint_torques, torque_measured);

  pinocchio::forwardKinematics(model_, data, q, v, a);
  pinocchio::computeJointJacobians(model_, data);
  pinocchio::updateFramePlacement(model_, data, feet_frame_idxs_[leg_index]);
  v_ee = pinocchio::getFrameVelocity(
             model_, data, feet_frame_idxs_[leg_index], pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED)
             .linear();

  FRAMEJac frame_jacobian;
  frame_jacobian.setZero();
  pinocchio::getFrameJacobian(
      model_, data, feet_frame_idxs_[leg_index], pinocchio::LOCAL_WORLD_ALIGNED, frame_jacobian);
  // auto taus = pinocchio::rnea(model_, data, q, v, a);
  // auto idyn = GetJointTangentConfigSpace(taus, leg_index);

  auto frame_leg_jacobian = frame_jacobian.block<3, 3>(0, model_.idx_vs[joint_indxs_[leg_index][0]]);
  // idyn = -idyn;  // TODO: why that!

  f_ee = (frame_leg_jacobian.transpose().completeOrthogonalDecomposition().pseudoInverse() * (joint_torques));
}

void QuadModelPino::CalcFootForceVelocityBodyFrame(int leg_index,
                                                   const StateInterface& state,
                                                   Eigen::Vector3d& f_ee,
                                                   Eigen::Vector3d& v_ee) const {
  CalcFootForceVelocityInBodyFrame(
      leg_index,
      Eigen::Map<const Eigen::Vector<double, N_JOINTS_PER_LEG>>(state.GetJointPositions()[leg_index].data()),
      Eigen::Map<const Eigen::Vector<double, N_JOINTS_PER_LEG>>(state.GetJointVelocities()[leg_index].data()),
      Eigen::Map<const Eigen::Vector<double, N_JOINTS_PER_LEG>>(state.GetJointAccelerations()[leg_index].data()),
      Eigen::Map<const Eigen::Vector<double, N_JOINTS_PER_LEG>>(state.GetJointTorques()[leg_index].data()),
      f_ee,
      v_ee);
}

void QuadModelPino::CalcLegInverseKinematicsInBody(int leg_index,
                                                   const Eigen::Vector3d& p_ee_B,
                                                   const Eigen::Vector3d& joint_state_init_guess,
                                                   Eigen::Vector3d& theta) const {
  // initializations
  FRAMEJac Jac;
  Eigen::Vector3d joint_state_prior_est, foot_pos_current;
  pinocchio::Data data(model_);
  joint_state_prior_est = joint_state_init_guess;
  theta.setZero();
  QVec q;
  q.setZero();
  q.segment<4>(3) = Eigen::Quaterniond::Identity().coeffs();
  Jac.setZero();
  SetJointConfigSpace(leg_index, joint_state_init_guess, q);
  pinocchio::forwardKinematics(model_, data, q);
  foot_pos_current = pinocchio::updateFramePlacement(model_, data, feet_frame_idxs_[leg_index]).translation();
  // std::cout << " foot pos current : \n" << foot_pos_current << "\np_ee_B : \n" << T_des << std::endl;
  Eigen::Vector3d error = p_ee_B - foot_pos_current;
  int max_iter = 1000, iter = 0;

  assert(joint_indxs_[leg_index][0] + 1 == joint_indxs_[leg_index][1]);
  assert(joint_indxs_[leg_index][1] + 1 == joint_indxs_[leg_index][2]);

  while (error.norm() > 1e-8 && iter < max_iter) {
    pinocchio::computeJointJacobians(model_, data, q);
    pinocchio::updateFramePlacements(model_, data);
    pinocchio::getFrameJacobian(model_, data, feet_frame_idxs_[leg_index], pinocchio::LOCAL_WORLD_ALIGNED, Jac);
    joint_state_prior_est +=
        Jac.block<3, 3>(0, model_.idx_vs[joint_indxs_[leg_index][0]]).completeOrthogonalDecomposition().pseudoInverse()
        * error;
    SetJointConfigSpace(leg_index, joint_state_prior_est, q);
    pinocchio::forwardKinematics(model_, data, q);
    foot_pos_current = pinocchio::updateFramePlacement(model_, data, feet_frame_idxs_[leg_index]).translation();
    // std::cout << " foot pos current : \n" << foot_pos_current << "\ni : \n" << iter << std::endl;
    error = p_ee_B - foot_pos_current;
    iter++;
  }
  if (iter == max_iter) {
    std::cerr << "Inv Kin: Max iteration reached."
              << "\n";
  }

  auto joint_state = GetJointConfigSpace(q, leg_index);
  auto upperLim = GetJointConfigSpace(model_.upperPositionLimit, leg_index);
  auto lowerLim = GetJointConfigSpace(model_.lowerPositionLimit, leg_index);

  if (joint_state(0) > upperLim(0) || joint_state(0) < lowerLim(0)) {
    joint_state(0) = std::max(std::min(joint_state(0), upperLim(0)), lowerLim(0));
    std::cerr << "Abad limits exceeded, clamping to" << joint_state(0) << "\n";
  }
  if (joint_state(1) > upperLim(1) || joint_state(1) < lowerLim(1)) {
    joint_state(1) = std::max(std::min(joint_state(1), upperLim(1)), lowerLim(1));
    std::cerr << "Hip limits exceeded, clamping to" << joint_state(1) << " \n";
  }
  if (joint_state(2) > upperLim(2) || joint_state(2) < lowerLim(2)) {
    joint_state(2) = std::max(std::min(joint_state(2), upperLim(2)), lowerLim(2));
    std::cerr << "Knee limit exceeded, clamping to" << joint_state(2) << " \n";
  }

  theta = joint_state;
}

void QuadModelPino::CalcLegDiffKinematicsBodyFrame(int leg_index,
                                                   const StateInterface& state,
                                                   Eigen::Vector3d& f_ee_goal,
                                                   Eigen::Vector3d& v_ee_goal,
                                                   Eigen::Vector3d& tau_goal,
                                                   Eigen::Vector3d& qd_goal) const {
  Eigen::Matrix3d J_leg, J_leg_inv;
  // int leg_index = get_LegIndex(leg);
  CalcJacobianLegBase(leg_index, Eigen::Map<const Eigen::Vector3d>(state.GetJointPositions()[leg_index].data()), J_leg);
  J_leg_inv = J_leg.inverse();
  // J_leg_inv = J_leg.transpose().completeOrthogonalDecomposition().pseudoInverse();

  // cartesian to joint mappings
  tau_goal = J_leg.transpose() * f_ee_goal;  // f must be in body frame
  qd_goal = J_leg_inv * v_ee_goal;
}

void QuadModelPino::CalcJacobianLegBase(int leg_index,
                                        Eigen::Vector3d joint_pos,
                                        Eigen::Matrix3d& Jac_legBaseToFoot) const {
  pinocchio::Data data(model_);
  FRAMEJac jac;
  jac.setZero();
  QVec q;
  q.setZero();
  q.segment<4>(3) = Eigen::Quaterniond::Identity().coeffs();
  SetJointConfigSpace(leg_index, joint_pos, q);
  pinocchio::computeJointJacobians(model_, data, q);
  pinocchio::updateFramePlacements(model_, data);
  pinocchio::getFrameJacobian(model_, data, feet_frame_idxs_[leg_index], pinocchio::LOCAL_WORLD_ALIGNED, jac);
  Jac_legBaseToFoot = jac.block<3, 3>(0, model_.idx_vs[joint_indxs_[leg_index][0]]);
};

[[deprecated("Use CalcFootPositionInBodyFrame() instead")]] void QuadModelPino::CalcFwdKinLegBody(
    int leg_indx,
    const Eigen::Vector3d& joint_pos,
    Eigen::Matrix4d& T_BodyToFoot,
    Eigen::Vector3d& foot_pos_body) const {
  pinocchio::Data data(model_);
  QVec q;
  q.setZero();
  q.segment<4>(3) = Eigen::Quaterniond::Identity().coeffs();
  SetJointConfigSpace(leg_indx, joint_pos, q);
  pinocchio::forwardKinematics(model_, data, q);
  auto& frame_pos = pinocchio::updateFramePlacement(model_, data, feet_frame_idxs_[leg_indx]);
  T_BodyToFoot = frame_pos;
  foot_pos_body = frame_pos.translation();
}

Eigen::Vector3d QuadModelPino::CalcFootPositionInWorld(unsigned int foot_idx, const StateInterface& state) const {
  return CalcFootPositionInWorld(foot_idx,
                                 state.GetPositionInWorld(),
                                 state.GetOrientationInWorld(),
                                 Eigen::Map<const Eigen::Vector3d>(state.GetJointPositions()[foot_idx].data()));
}
Eigen::Vector3d QuadModelPino::CalcFootPositionInWorld(unsigned int foot_idx,
                                                       const Eigen::Vector3d& body_pos,
                                                       const Eigen::Quaterniond& body_orientation,
                                                       const Eigen::Vector3d& joint_positions) const {
  pinocchio::Data data(model_);
  QVec q;
  q.setZero();
  q.segment<3>(0) = body_pos;
  q.segment<4>(3) = body_orientation.coeffs();
  SetJointConfigSpace(foot_idx, joint_positions, q);
  pinocchio::forwardKinematics(model_, data, q);
  return pinocchio::updateFramePlacement(model_, data, feet_frame_idxs_[foot_idx]).translation();
}
Eigen::Vector3d QuadModelPino::CalcFootPositionInBodyFrame(unsigned int foot_idx,
                                                           const Eigen::Vector3d& joint_positions) const {
  return CalcFootPositionInWorld(foot_idx, Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), joint_positions);
}

Eigen::Matrix3d QuadModelPino::GetInertia() const { return data_.Ag.block<3, 3>(3, 3); }
Eigen::Matrix3d QuadModelPino::getBaseInertia() const { return model_.inertias[1].inertia().matrix(); }
void QuadModelPino::getLegInertia(
    const std::array<Eigen::Vector<double, ModelInterface::N_JOINTS_PER_LEG>, ModelInterface::N_LEGS>& joint_pos,
    Eigen::Ref<Eigen::Matrix3d> leg_inertia) const {
  pinocchio::Data data(model_);
  // set configuration
  QVec q;
  q.setZero();
  q.segment<4>(3) = Eigen::Quaterniond::Identity().coeffs();
  for (uint i = 0; i < ModelInterface::N_LEGS; i++) {
    SetJointConfigSpace(i, joint_pos[i], q);
  }
  auto model = model_;
  // set base_link inertia to zero
  Eigen::Matrix3d zero;
  zero.setZero();
  model.inertias[model.frames[base_link_idx_].parentJoint].inertia() = pinocchio::Symmetric3(zero);
  // compute inertias
  pinocchio::crba(model, data, q);
  // now that base_link inertia is zero Ycrb[1] should output total inertia in base frame
  leg_inertia = data.Ycrb[1].inertia().matrix();
}
double QuadModelPino::GetInertia(const int row, const int column) const { return GetInertia()(row, column); }
double QuadModelPino::GetMass() const { return data_.mass[0]; }
double QuadModelPino::GetLegMass(int foot_idx) const { return data_.mass[joint_indxs_[foot_idx][0]]; }
double QuadModelPino::getBaseMass() const { return model_.inertias[1].mass(); }
double QuadModelPino::GetG() const { return abs(model_.gravity.linear().z()); }

Eigen::Translation3d QuadModelPino::GetBodyToIMU() const {
  return Eigen::Translation3d(data_.oMf[imu_frame_id_].translation());
}
Eigen::Translation3d QuadModelPino::GetBodyToBellyBottom() const {
  return Eigen::Translation3d(data_.oMf[bellyb_frame_id_].translation());
}
Eigen::Translation3d QuadModelPino::GetBodyToBellyBottom(unsigned int leg) const {
  return Eigen::Translation3d(data_.oMf[joint_frame_indxs_[leg][0]].translation()
                              + data_.oMf[bellyb_frame_id_].translation());
}
Eigen::Translation3d QuadModelPino::GetBodyToCOM() const { return Eigen::Translation3d(data_.com[0]); }
Eigen::Vector3d QuadModelPino::getBaseCOM() const { return model_.inertias[1].lever(); }

Eigen::Translation3d QuadModelPino::GetBodyToLegCOM(int foot_idx) const {
  auto liMi = data_.liMi[joint_indxs_[foot_idx][0]];
  return Eigen::Translation3d(liMi.rotation() * data_.com[joint_indxs_[foot_idx][0]] + liMi.translation());
}

Eigen::Translation3d QuadModelPino::GetBodyToLegBase(int foot_idx) const {
  return Eigen::Translation3d(data_.liMi[joint_indxs_[foot_idx][0]].translation());
}
void QuadModelPino::CalcBaseHeight(const std::array<bool, ModelInterface::N_LEGS>& feet_contacts,
                                   const std::array<const Eigen::Vector3d, ModelInterface::N_LEGS>& joint_states,
                                   const Eigen::Quaterniond& body_orientation,
                                   double& base_height) const {
  pinocchio::Data data(model_);
  double sum_z = 0;
  int num_contact_feet = 0;
  Eigen::Vector3d foot_pos_body;
  for (int i = 0; i < ModelInterface::N_LEGS; i++) {
    if (feet_contacts[i]) {
      foot_pos_body = CalcFootPositionInBodyFrame(i, joint_states[i]);
      sum_z += -foot_pos_body(2);
      num_contact_feet += 1;
    }
  }
  base_height = sum_z / num_contact_feet;
}

bool QuadModelPino::IsLyingDown(const std::array<const Eigen::Vector3d, ModelInterface::N_LEGS>& joint_states,
                                const std::array<double, ModelInterface::N_LEGS>& distance_threshold) const {
  Eigen::Vector3d foot_pos_body;
  for (unsigned int i = 0; i < ModelInterface::N_LEGS; i++) {
    foot_pos_body = CalcFootPositionInBodyFrame(i, joint_states[i]);
    if ((foot_pos_body(2) - distance_threshold[i]) < GetBodyToBellyBottom().translation().z()) {
      return false;
    }
  }
  return true;
}

double QuadModelPino::ComputeKineticEnergy(int leg_index,
                                           const Eigen::Vector3d& joint_pos,
                                           const Eigen::Vector3d& joint_vel) const {
  return 0;
}
double QuadModelPino::ComputeEnergyDerivative(int leg_index,
                                              const Eigen::Vector3d& joint_vel,
                                              const Eigen::Vector3d& tau) const {
  return 0;
}

void QuadModelPino::ComputeMomentumSignal(
    const StateInterface& state, Eigen::Vector<double, ModelInterface::NUM_JOINTS + 6>& momentum_signal) const {
  pinocchio::Data data(model_);
  QVec q;
  VVec v;
  VVec a;
  VVec torque;
  q.setZero();
  v.setZero();
  a.setZero();
  torque.setZero();
  q.segment<3>(0) = state.GetPositionInWorld();
  q.segment<4>(3) = state.GetOrientationInWorld().coeffs();
  v.segment<3>(0) = state.GetLinearVelInWorld();
  v.segment<3>(3) = state.GetAngularVelInWorld();
  a.segment<3>(0) = state.GetLinearAccInWorld();
  a.segment<3>(3) = state.GetAngularAccInWorld();
  for (unsigned int leg_idx = 0; leg_idx < ModelInterface::N_LEGS; leg_idx++) {
    SetJointConfigSpace(
        leg_idx,
        Eigen::Vector<double, ModelInterface::N_JOINTS_PER_LEG>::Map(state.GetJointPositions()[leg_idx].data()),
        q);
    SetJointTangentConfigSpace(
        leg_idx,
        Eigen::Vector<double, ModelInterface::N_JOINTS_PER_LEG>::Map(state.GetJointVelocities()[leg_idx].data()),
        v);
    SetJointTangentConfigSpace(
        leg_idx,
        Eigen::Vector<double, ModelInterface::N_JOINTS_PER_LEG>::Map(state.GetJointAccelerations()[leg_idx].data()),
        a);
    SetJointTangentConfigSpace(
        leg_idx,
        Eigen::Vector<double, ModelInterface::N_JOINTS_PER_LEG>::Map(state.GetJointTorques()[leg_idx].data()),
        torque);
  }
  // Eigen::Vector<double, 18> tau_legDynamics = pinocchio::rnea(model_, data, q, v, a);
  // torque = torque - tau_legDynamics;
  pinocchio::computeGeneralizedGravity(model_, data, q);
  pinocchio::computeCoriolisMatrix(model_, data, q, v);
  momentum_signal = torque + data.C.transpose() * v - data.g;
}
void QuadModelPino::ComputeGeneralizedMomentum(
    const Eigen::Vector<double, ModelInterface::NUM_JOINTS + 7>& joint_pos,
    const Eigen::Vector<double, ModelInterface::NUM_JOINTS + 6>& joint_vel,
    Eigen::Vector<double, ModelInterface::NUM_JOINTS + 6>& generalized_momentum) const {
  // TODO: Change to joint pos, vel and acc in the arguments
  pinocchio::Data data(model_);
  QVec q;
  VVec v;
  q.setZero();
  v.setZero();
  SetJointConfigSpace(joint_pos, q);
  SetJointTangentConfigSpace(joint_vel, v);
  pinocchio::crba(model_, data, q);
  data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
  // pinocchio::computeGeneralizedGravity(model_, data, q);
  // pinocchio::computeAllTerms(model_, data, q, v);
  // pinocchio::computeCoriolisMatrix(model_, data, q, v);
  generalized_momentum = data.M * v;
}
void QuadModelPino::ComputeEstimatedForces(int leg_index,
                                           const Eigen::Vector<double, ModelInterface::N_JOINTS_PER_LEG>& tau_disturbed,
                                           const Eigen::Vector<double, ModelInterface::N_JOINTS_PER_LEG>& joint_pos,
                                           Eigen::Vector3d& estimated_forces) const {
  Eigen::Matrix3d jacobian;
  CalcJacobianLegBase(leg_index, joint_pos, jacobian);
  estimated_forces = jacobian.transpose().completeOrthogonalDecomposition().pseudoInverse() * tau_disturbed;
}

void QuadModelPino::ComputeRegressorMatrix(
    const StateInterface& state,
    Eigen::Matrix<double, ModelInterface::NUM_JOINTS + 6, (ModelInterface::NUM_JOINTS + 1) * 10>& regressor) const {
  pinocchio::Data data(model_);
  QVec q;
  VVec v;
  VVec a;
  q.setZero();
  q.segment<3>(0) = state.GetPositionInWorld();
  q.segment<4>(3) = state.GetOrientationInWorld().coeffs();
  SetJointConfigSpace(
      0, Eigen::Map<const Eigen::Vector<double, N_JOINTS_PER_LEG>>(state.GetJointPositions()[0].data()), q);
  SetJointConfigSpace(
      1, Eigen::Map<const Eigen::Vector<double, N_JOINTS_PER_LEG>>(state.GetJointPositions()[1].data()), q);
  SetJointConfigSpace(
      2, Eigen::Map<const Eigen::Vector<double, N_JOINTS_PER_LEG>>(state.GetJointPositions()[2].data()), q);
  SetJointConfigSpace(
      3, Eigen::Map<const Eigen::Vector<double, N_JOINTS_PER_LEG>>(state.GetJointPositions()[3].data()), q);
  v.setZero();
  v.segment<3>(0) = state.GetLinearVelInWorld();
  v.segment<3>(3) = state.GetAngularVelInWorld();
  SetJointTangentConfigSpace(
      0, Eigen::Map<const Eigen::Vector<double, N_JOINTS_PER_LEG>>(state.GetJointVelocities()[0].data()), v);
  SetJointTangentConfigSpace(
      1, Eigen::Map<const Eigen::Vector<double, N_JOINTS_PER_LEG>>(state.GetJointVelocities()[1].data()), v);
  SetJointTangentConfigSpace(
      2, Eigen::Map<const Eigen::Vector<double, N_JOINTS_PER_LEG>>(state.GetJointVelocities()[2].data()), v);
  SetJointTangentConfigSpace(
      3, Eigen::Map<const Eigen::Vector<double, N_JOINTS_PER_LEG>>(state.GetJointVelocities()[3].data()), v);
  a.setZero();
  a.segment<3>(0) = state.GetLinearAccInWorld();
  a.segment<3>(3) = state.GetAngularAccInWorld();
  SetJointTangentConfigSpace(
      0, Eigen::Map<const Eigen::Vector<double, N_JOINTS_PER_LEG>>(state.GetJointAccelerations()[0].data()), a);
  SetJointTangentConfigSpace(
      1, Eigen::Map<const Eigen::Vector<double, N_JOINTS_PER_LEG>>(state.GetJointAccelerations()[1].data()), a);
  SetJointTangentConfigSpace(
      2, Eigen::Map<const Eigen::Vector<double, N_JOINTS_PER_LEG>>(state.GetJointAccelerations()[2].data()), a);
  SetJointTangentConfigSpace(
      3, Eigen::Map<const Eigen::Vector<double, N_JOINTS_PER_LEG>>(state.GetJointAccelerations()[3].data()), a);
  pinocchio::computeJointTorqueRegressor(model_, data, q, v, a);
  assert(data.jointTorqueRegressor.rows() == regressor.rows());
  assert(data.jointTorqueRegressor.cols() == regressor.cols());

  // Rearrange rows from data matrix to our matric
  Eigen::Matrix<double, ModelInterface::NUM_JOINTS + 6, (ModelInterface::NUM_JOINTS + 1) * 10> regressor_lines_sorted;
  regressor_lines_sorted.topRows<6>() = data.jointTorqueRegressor.topRows<6>();

  for (int leg_idx = 0; leg_idx < ModelInterface::N_LEGS; leg_idx++) {
    for (int joint_idx = 0; joint_idx < ModelInterface::N_JOINTS_PER_LEG; joint_idx++) {
      regressor_lines_sorted.row(6 + leg_idx * ModelInterface::N_JOINTS_PER_LEG + joint_idx) =
          data.jointTorqueRegressor.row(model_.idx_vs[joint_indxs_[leg_idx][joint_idx]]);
    }
  }
  // Rearrange cols
  // First 10 should be correct
  regressor.leftCols<10>() =
      regressor_lines_sorted.middleCols<10>((model_.frames[base_link_idx_].parentJoint - 1) * 10);
  for (unsigned int leg_idx = 0; leg_idx < ModelInterface::N_LEGS; leg_idx++) {
    for (unsigned int joint_indx = 0; joint_indx < ModelInterface::N_JOINTS_PER_LEG; joint_indx++) {
      regressor.block<ModelInterface::NUM_JOINTS + 6, 10>(
          0, (1 + leg_idx * ModelInterface::N_JOINTS_PER_LEG + joint_indx) * 10) =
          regressor_lines_sorted.block<ModelInterface::NUM_JOINTS + 6, 10>(
              0, (model_.frames[joint_frame_indxs_[leg_idx][joint_indx]].parentJoint - 1) * 10);
    }
  }
}

void QuadModelPino::SetInertia(const Eigen::Matrix3d& inertia_tensor) {
  // Has to be symmetric!
  model_.inertias[model_.frames[base_link_idx_].parentJoint].inertia() = pinocchio::Symmetric3(inertia_tensor);
  UpdateModel();
}
void QuadModelPino::SetInertia(const int row, const int column, const double value) {
  // The following gi sreverse engineered from pinnochio
  unsigned char indx = 0;
  indx += row;
  indx += column;
  if (column > 1 || row > 1) {
    indx++;
  }
  model_.inertias[model_.frames[base_link_idx_].parentJoint].inertia().data()(indx) = value;
  UpdateModel();
}
void QuadModelPino::SetMass(double mass) {
  model_.inertias[model_.frames[base_link_idx_].parentJoint].mass() = mass;
  UpdateModel();
}
void QuadModelPino::SetCOM(const Eigen::Vector3d& com) {
  model_.inertias[model_.frames[base_link_idx_].parentJoint].lever() = com;
  UpdateModel();
}
void QuadModelPino::SetCOM(const int idx, const double val) {
  model_.inertias[model_.frames[base_link_idx_].parentJoint].lever()(idx) = val;
  UpdateModel();
}

const Eigen::Vector<double, (1 + ModelInterface::NUM_JOINTS) * 10>& QuadModelPino::GetAllDynamicParameters() {
  return dynamic_parameters_;
}

QuadModelPino& QuadModelPino::operator=(const ModelInterface& other) {
  return QuadModelPino::operator=(dynamic_cast<const QuadModelPino&>(other));
}
