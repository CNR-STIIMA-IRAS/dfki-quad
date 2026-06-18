#pragma once
#include "common/model_interface.hpp"
#include "common/state_interface.hpp"
#include "gait_controller/gait_sequence.hpp"

class ModelAdaptationInterface {
 public:
  static const int NUM_PARAMS = 3;
  /**
   * Updates the current state
   *
   * @param state the new state
   */
  virtual void UpdateState(const StateInterface& state) = 0;
  /**
   * Updates the model according to state data
   *
   * @param model the model to update
   * @return if the model was updated
   */
  virtual void UpdateGaitSequence(const GaitSequence& gs) = 0;
  virtual bool DoModelAdaptation(ModelInterface& model) = 0;
  virtual Eigen::Vector<double, NUM_PARAMS> GetParameterVector() const = 0;
  virtual Eigen::Matrix<double, NUM_PARAMS, NUM_PARAMS> GetParameterCovariance() const = 0;
  virtual Eigen::Vector<double, NUM_PARAMS> GetDelta() const = 0;
  virtual Eigen::Vector<double, 6> GetTotalForceTorque() const = 0;
  virtual Eigen::Vector<double, NUM_PARAMS> GetSV() const = 0;
};