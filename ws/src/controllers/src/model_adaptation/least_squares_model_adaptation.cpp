#include "model_adaptation/least_squares_model_adaptation.hpp"

#include "common/quaternion_operations.hpp"

LeastSquaresModelAdaptation::LeastSquaresModelAdaptation(
    std::unique_ptr<ModelInterface> quad_model,
    std::unique_ptr<StateInterface> initial_state,
    const Eigen::Ref<const Eigen::Vector<double, NUM_PARAMS>> CONV_THRESH,
    double lambda)
    : state_(std::move(initial_state)), conv_thresh_(CONV_THRESH), lambda_(lambda) {
  assert(NUM_PARAMS == 3);
  // initialize parameter vector
  // mass
  parameter_vector_(0) = quad_model->GetMass();
  // com (z is unobservable)
  auto com = quad_model->GetBodyToCOM();
  parameter_vector_(1) = com.x() * parameter_vector_(0);
  parameter_vector_(2) = com.y() * parameter_vector_(0);

  // initialize parameter covariance
  parameter_covariance_.setIdentity();
  // get leg total mass and com
  leg_total_mass_ = 0.0;
  leg_total_mcom_ = Eigen::Vector3d::Zero();
  for (int i = 0; i < 4; i++) {
    double leg_mass = quad_model->GetLegMass(i);
    Eigen::Translation3d leg_com = quad_model->GetBodyToLegCOM(i);
    leg_total_mass_ += leg_mass;
    leg_total_mcom_ += leg_mass * leg_com.translation();
  }
}

void LeastSquaresModelAdaptation::UpdateState(const StateInterface& state) { *state_ = state; }

void LeastSquaresModelAdaptation::UpdateGaitSequence(const GaitSequence& gs) { gait_sequence_ = gs; }

bool LeastSquaresModelAdaptation::DoModelAdaptation(ModelInterface& model) {
  // populate regressor matrix
  Eigen::Matrix<double, 18, 130> full_body_regressor;
  model.ComputeRegressorMatrix(*state_, full_body_regressor);
  // only the first 6 lines have non zero entries corresponding to the parameters of the base
  Eigen::Matrix<double, 6, NUM_PARAMS> regressor_matrix;
  regressor_matrix = full_body_regressor.block<6, NUM_PARAMS>(0, 0);
  // populate measurement vector
  std::array<bool, ModelInterface::N_LEGS> contact_state = state_->GetFeetContacts();
  std::array<Eigen::Vector3d, ModelInterface::N_LEGS> foot_positions;
  std::array<Eigen::Vector3d, ModelInterface::N_LEGS> contact_forces = state_->GetContactForces();
  for (int i = 0; i < 4; i++) {
    foot_positions[i] =
        model.CalcFootPositionInBodyFrame(i, Eigen::Map<const Eigen::Vector3d>(state_->GetJointPositions()[i].data()));
    contact_forces[i] = state_->GetOrientationInWorld().inverse() * contact_forces[i];
  }

  CalcMeasurementVector(
      contact_forces, foot_positions, contact_state, gait_sequence_.contact_sequence[0], measurement_vector_);
  // Recursive Least Squares step
  // define some helper variables
  Eigen::Matrix<double, 6, 6> I, A;
  I.setIdentity();
  A = lambda_ * I + regressor_matrix * parameter_covariance_ * regressor_matrix.transpose();
  parameter_covariance_ = (1.0 / lambda_ * parameter_covariance_
                           - 1.0 / lambda_ * parameter_covariance_ * regressor_matrix.transpose() * A.inverse()
                                 * regressor_matrix * parameter_covariance_)
                              .eval();
  innovation_.setZero();
  innovation_ = parameter_covariance_ * regressor_matrix.transpose()
                * (measurement_vector_ - regressor_matrix * parameter_vector_);
  parameter_vector_ = (parameter_vector_ + innovation_).eval();

  // check if mass is to be updated
  Eigen::Array<bool, NUM_PARAMS, 1> updated;
  updated.setConstant(false);
  if (parameter_covariance_(0, 0) < conv_thresh_(0)) {
    model.SetMass(parameter_vector_(0) - leg_total_mass_);
    updated(0) = true;
  }
  // check which com parameters are set
  // if one param is not selected and covariance is not low enough don't change
  Eigen::Vector3d set_com = model.GetBodyToCOM().translation() * model.GetMass();
  for (uint i = 1; i < 3; i++) {
    if (parameter_covariance_(i, i) < conv_thresh_(i)) {
      set_com(i - 1) = parameter_vector_(i);
      updated(i) = true;
    }
  }
  if ((updated.segment<2>(1) == true).any()) {
    Eigen::Vector3d com_trunk;
    com_trunk = (set_com - leg_total_mcom_) / parameter_vector_(0);
    model.SetCOM(com_trunk);
  }

  if ((updated == true).any()) {
    return true;
  } else {
    return false;
  }
  return false;
}

// attention in the force vector here the order is linear, rotational
void LeastSquaresModelAdaptation::CalcMeasurementVector(const std::array<Eigen::Vector3d, 4>& contact_forces,
                                                        const std::array<Eigen::Vector3d, 4>& foot_positions,
                                                        const std::array<bool, 4>& contact_state,
                                                        const std::array<bool, 4>& planned_contact,
                                                        Eigen::Ref<Eigen::Vector<double, 6>> measurement_vector) const {
  // sum of forces acting on torso
  Eigen::Vector3d forces = Eigen::Vector3d::Zero();
  Eigen::Vector3d torques = Eigen::Vector3d::Zero();
  for (int i = 0; i < 4; i++) {
    if (contact_state[i] && planned_contact[i]) {
      forces += contact_forces[i];
      torques += foot_positions[i].cross(contact_forces[i]);
    }
  }
  measurement_vector.head<3>() = forces;   // set first 3 elements to forces
  measurement_vector.tail<3>() = torques;  // and last 3 to torques
}

Eigen::Vector<double, ModelAdaptationInterface::NUM_PARAMS> LeastSquaresModelAdaptation::GetParameterVector() const {
  return parameter_vector_;
}
Eigen::Matrix<double, ModelAdaptationInterface::NUM_PARAMS, ModelAdaptationInterface::NUM_PARAMS>
LeastSquaresModelAdaptation::GetParameterCovariance() const {
  return parameter_covariance_;
}
Eigen::Vector<double, LeastSquaresModelAdaptation::NUM_PARAMS> LeastSquaresModelAdaptation::GetDelta() const {
  return innovation_;
}

Eigen::Vector<double, 6> LeastSquaresModelAdaptation::GetTotalForceTorque() const { return measurement_vector_; }

Eigen::Vector<double, LeastSquaresModelAdaptation::NUM_PARAMS> LeastSquaresModelAdaptation::GetSV() const {
  return Eigen::Vector<double, NUM_PARAMS>::Zero();
}
