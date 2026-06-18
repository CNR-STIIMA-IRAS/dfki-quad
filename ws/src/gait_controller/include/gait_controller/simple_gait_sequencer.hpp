#pragma once

#include <array>
#include <rclcpp/time.hpp>

#include "interfaces/msg/gait_state.hpp"
#include "common/target.hpp"

#include "gait_controller/gait.hpp"
#include "gait_controller/gait_sequencer_interface.hpp"

#include "gait_controller/raibert_foot_step_planner.hpp"

class SimpleGaitSequencer : public GaitSequencerInterface {
 public:
  SimpleGaitSequencer(const Gait& gait,
                      double k,
                      const std::array<const Eigen::Vector3d, N_LEGS>& shoulder_positions,
                      std::unique_ptr<StateInterface> quad_state,
                      std::unique_ptr<ModelInterface> quad_model,
                      unsigned int raibert_filtersize,
                      bool raibert_z_on_plane,
                      bool early_contact_detection);
  void GetGaitSequence(GaitSequence& gait_sequence) override;
  void UpdateState(const StateInterface& quad_state) override;
  void UpdateTarget(const Target& new_target) override;
  void GetGaitState(interfaces::msg::GaitState& state) override;
  void UpdateModel(const ModelInterface& quad_model) override;
  GS_Type GetType() const override;

 private:
  Gait gait_;
  //MPCTrajectoryPlanner trajectory_planner_;
  double phase_;
  StateInterface::TimePoint time_;
  bool time_initialized_;
  bool early_contact_detection_;
};