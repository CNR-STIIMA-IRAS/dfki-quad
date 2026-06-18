#pragma once

#include "common/model_interface.hpp"
#include "common/state_interface.hpp"
#include "common/target.hpp"

#include "interfaces/msg/gait_state.hpp"

#include "gait_sequence.hpp"
#include "gait_sequencer_types.hpp"
#include "gait_controller/raibert_foot_step_planner.hpp"

class GaitSequencerInterface {
 protected:
  GaitSequencerInterface() = default;  // protected, as there cant be any Object from an Interface
  std::unique_ptr<StateInterface> quad_state_;
  std::unique_ptr<ModelInterface> quad_model_;
  
  Target target_;
   RaibertFootStepPlanner foot_step_planner_;

  GaitSequencerInterface( double k,
                          const std::array<const Eigen::Vector3d, N_LEGS>& shoulder_positions,
                          std::unique_ptr<StateInterface> quad_state,
                          std::unique_ptr<ModelInterface> quad_model,
                          unsigned int raibert_filtersize,
                          bool raibert_z_on_plane) 
    : quad_state_(std::move(quad_state)),
      quad_model_(std::move(quad_model)), 
      target_({}),
      foot_step_planner_(shoulder_positions, *quad_state_, *quad_model_, raibert_filtersize, raibert_z_on_plane, k)
      {};  // protected, as there cant be any Object from an Interface
  
 public:
  virtual void GetGaitSequence(GaitSequence& gait_sequence) = 0;
  virtual void UpdateState(const StateInterface& quad_state) = 0;
  virtual void UpdateModel(const ModelInterface& quad_model) = 0;

  virtual void UpdateTarget(const Target& new_target) = 0;
  virtual void GetGaitState(interfaces::msg::GaitState& state) = 0;
  
  const StateInterface& GetState() const { return *quad_state_; }
  const ModelInterface& GetModel() const { return *quad_model_; }
  const Target& GetTarget() const { return target_; }

  virtual GS_Type GetType() const = 0;

  virtual ~GaitSequencerInterface() = default;


};
