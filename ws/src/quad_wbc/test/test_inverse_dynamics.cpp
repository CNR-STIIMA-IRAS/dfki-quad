#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <filesystem>
#include <memory>

#include "common/model_interface.hpp"
#include "quad_wbc/inverse_dynamics.hpp"
#include "quad_wbc/wbc_arc_opt.hpp"

namespace {

class MockState : public StateInterface {
 public:
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d linear_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
  Eigen::Vector3d angular_acceleration = Eigen::Vector3d::Zero();
  TimePoint time{};
  std::array<bool, NUM_FEET> feet_contacts{};
  std::array<Eigen::Vector3d, NUM_FEET> contact_forces{};
  std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET> joint_positions{};
  std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET> joint_velocities{};
  std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET> joint_accelerations{};
  std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET> joint_torques{};

  MockState() {
    for (auto& force : contact_forces) {
      force.setZero();
    }
  }

  const Eigen::Vector3d& GetPositionInWorld() const override { return position; }
  const Eigen::Quaterniond& GetOrientationInWorld() const override { return orientation; }
  const Eigen::Vector3d& GetLinearVelInWorld() const override { return linear_velocity; }
  const Eigen::Vector3d& GetAngularVelInWorld() const override { return angular_velocity; }
  const Eigen::Vector3d& GetLinearAccInWorld() const override { return linear_acceleration; }
  const Eigen::Vector3d& GetAngularAccInWorld() const override { return angular_acceleration; }
  const TimePoint& GetTime() const override { return time; }
  const std::array<bool, NUM_FEET>& GetFeetContacts() const override { return feet_contacts; }
  const std::array<Eigen::Vector3d, NUM_FEET>& GetContactForces() const override { return contact_forces; }
  const std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET>& GetJointPositions() const override {
    return joint_positions;
  }
  const std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET>& GetJointVelocities() const override {
    return joint_velocities;
  }
  const std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET>& GetJointAccelerations() const override {
    return joint_accelerations;
  }
  const std::array<std::array<double, NUM_JOINT_PER_FOOT>, NUM_FEET>& GetJointTorques() const override {
    return joint_torques;
  }

  void SetVelocitiesToZero() override {
    linear_velocity.setZero();
    angular_velocity.setZero();
    for (auto& velocity : joint_velocities) {
      velocity = {};
    }
  }

  void SetAccelerationsToZero() override {
    linear_acceleration.setZero();
    angular_acceleration.setZero();
    for (auto& acceleration : joint_accelerations) {
      acceleration = {};
    }
  }

  StateInterface& operator=(const StateInterface& other) override {
    position = other.GetPositionInWorld();
    orientation = other.GetOrientationInWorld();
    linear_velocity = other.GetLinearVelInWorld();
    angular_velocity = other.GetAngularVelInWorld();
    linear_acceleration = other.GetLinearAccInWorld();
    angular_acceleration = other.GetAngularAccInWorld();
    time = other.GetTime();
    feet_contacts = other.GetFeetContacts();
    contact_forces = other.GetContactForces();
    joint_positions = other.GetJointPositions();
    joint_velocities = other.GetJointVelocities();
    joint_accelerations = other.GetJointAccelerations();
    joint_torques = other.GetJointTorques();
    return *this;
  }
};

class MockModel : public ModelInterface {
 public:
  std::array<Eigen::Vector3d, N_LEGS> foot_positions{};
  Eigen::Matrix3d inertia = Eigen::Matrix3d::Identity();
  Eigen::Vector3d base_com = Eigen::Vector3d::Zero();
  Eigen::Translation3d identity_translation{Eigen::Vector3d::Zero()};
  Eigen::Vector<double, (1 + ModelInterface::NUM_JOINTS) * 10> dynamic_parameters =
      Eigen::Vector<double, (1 + ModelInterface::NUM_JOINTS) * 10>::Zero();

  MockModel() {
    for (auto& foot_position : foot_positions) {
      foot_position.setZero();
    }
  }

  void CalcFootForceVelocityInBodyFrame(int,
                                        const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>&,
                                        const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>&,
                                        const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>&,
                                        const Eigen::Ref<const Eigen::Matrix<double, N_JOINTS_PER_LEG, 1>>&,
                                        Eigen::Ref<Eigen::Vector3d> f_ee,
                                        Eigen::Ref<Eigen::Vector3d> v_ee) const override {
    f_ee.setZero();
    v_ee.setZero();
  }
  void CalcFootForceVelocityBodyFrame(int, const StateInterface&, Eigen::Vector3d& f_ee, Eigen::Vector3d& v_ee)
      const override {
    f_ee.setZero();
    v_ee.setZero();
  }
  void CalcLegInverseKinematicsInBody(int,
                                      const Eigen::Vector3d& p_ee_B,
                                      const Eigen::Vector3d&,
                                      Eigen::Vector3d& theta) const override {
    theta = p_ee_B;
  }
  void CalcLegDiffKinematicsBodyFrame(int,
                                      const StateInterface&,
                                      Eigen::Vector3d& f_ee_goal,
                                      Eigen::Vector3d& v_ee_goal,
                                      Eigen::Vector3d& tau_goal,
                                      Eigen::Vector3d& qd_goal) const override {
    tau_goal = f_ee_goal;
    qd_goal = v_ee_goal;
  }
  void CalcJacobianLegBase(int, Eigen::Vector3d, Eigen::Matrix3d& jac_leg_base_to_foot) const override {
    jac_leg_base_to_foot.setIdentity();
  }
  void CalcFwdKinLegBody(int leg_index,
                         const Eigen::Vector3d&,
                         Eigen::Matrix4d& body_to_foot,
                         Eigen::Vector3d& foot_pos_body) const override {
    body_to_foot.setIdentity();
    foot_pos_body = foot_positions[leg_index];
  }
  Eigen::Vector3d CalcFootPositionInWorld(unsigned int foot_idx, const StateInterface&) const override {
    return foot_positions[foot_idx];
  }
  Eigen::Vector3d CalcFootPositionInWorld(unsigned int foot_idx,
                                          const Eigen::Vector3d& body_pos,
                                          const Eigen::Quaterniond& body_orientation,
                                          const Eigen::Vector3d&) const override {
    return body_pos + body_orientation * foot_positions[foot_idx];
  }
  Eigen::Vector3d CalcFootPositionInBodyFrame(unsigned int foot_idx, const Eigen::Vector3d&) const override {
    return foot_positions[foot_idx];
  }
  Eigen::Matrix3d GetInertia() const override { return inertia; }
  double GetInertia(const int row, const int column) const override { return inertia(row, column); }
  Eigen::Matrix3d getBaseInertia() const override { return inertia; }
  void getLegInertia(const std::array<Eigen::Vector<double, ModelInterface::N_JOINTS_PER_LEG>, ModelInterface::N_LEGS>&,
                     Eigen::Ref<Eigen::Matrix3d> leg_inertia) const override {
    leg_inertia.setIdentity();
  }
  double GetMass() const override { return 1.0; }
  double GetLegMass(int) const override { return 0.25; }
  double getBaseMass() const override { return 1.0; }
  double GetG() const override { return 9.81; }
  void CalcBaseHeight(const std::array<bool, 4>&,
                      const std::array<const Eigen::Vector3d, 4>&,
                      const Eigen::Quaterniond&,
                      double& base_height) const override {
    base_height = 0.0;
  }
  bool IsLyingDown(const std::array<const Eigen::Vector3d, ModelInterface::N_LEGS>&,
                   const std::array<double, ModelInterface::N_LEGS>&) const override {
    return false;
  }
  double ComputeKineticEnergy(int, const Eigen::Vector3d&, const Eigen::Vector3d&) const override { return 0.0; }
  double ComputeEnergyDerivative(int, const Eigen::Vector3d&, const Eigen::Vector3d&) const override { return 0.0; }
  void ComputeMomentumSignal(const StateInterface&,
                             Eigen::Vector<double, ModelInterface::NUM_JOINTS + 6>& momentum_signal) const override {
    momentum_signal.setZero();
  }
  void ComputeGeneralizedMomentum(const Eigen::Vector<double, ModelInterface::NUM_JOINTS + 7>&,
                                  const Eigen::Vector<double, ModelInterface::NUM_JOINTS + 6>&,
                                  Eigen::Vector<double, ModelInterface::NUM_JOINTS + 6>& generalized_momentum)
      const override {
    generalized_momentum.setZero();
  }
  void ComputeEstimatedForces(int,
                              const Eigen::Vector<double, ModelInterface::N_JOINTS_PER_LEG>&,
                              const Eigen::Vector<double, ModelInterface::N_JOINTS_PER_LEG>&,
                              Eigen::Vector3d& estimated_forces) const override {
    estimated_forces.setZero();
  }
  void ComputeRegressorMatrix(
      const StateInterface&,
      Eigen::Matrix<double, ModelInterface::NUM_JOINTS + 6, (ModelInterface::NUM_JOINTS + 1) * 10>& regressor)
      const override {
    regressor.setZero();
  }
  void SetInertia(const Eigen::Matrix3d& inertia_tensor) override { inertia = inertia_tensor; }
  void SetInertia(const int row, const int column, const double value) override { inertia(row, column) = value; }
  void SetMass(double) override {}
  void SetCOM(const Eigen::Vector3d& com) override { base_com = com; }
  void SetCOM(const int idx, const double val) override { base_com(idx) = val; }
  const Eigen::Vector<double, (1 + ModelInterface::NUM_JOINTS) * 10>& GetAllDynamicParameters() override {
    return dynamic_parameters;
  }
  Eigen::Translation3d GetBodyToIMU() const override { return identity_translation; }
  Eigen::Translation3d GetBodyToBellyBottom() const override { return identity_translation; }
  Eigen::Translation3d GetBodyToBellyBottom(unsigned int) const override { return identity_translation; }
  Eigen::Translation3d GetBodyToCOM() const override { return Eigen::Translation3d(base_com); }
  Eigen::Translation3d GetBodyToLegCOM(int) const override { return identity_translation; }
  Eigen::Translation3d GetBodyToLegBase(int) const override { return identity_translation; }
  Eigen::Vector3d getBaseCOM() const override { return base_com; }

  ModelInterface& operator=(const ModelInterface& other) override {
    inertia = other.getBaseInertia();
    base_com = other.getBaseCOM();
    return *this;
  }
};

FeetTargets MakeFeetTargets() {
  FeetTargets targets;
  for (unsigned int leg = 0; leg < N_LEGS; ++leg) {
    targets.positions[leg] = Eigen::Vector3d::Zero();
    targets.velocities[leg] = Eigen::Vector3d::Zero();
    targets.accelerations[leg] = Eigen::Vector3d::Zero();
  }
  return targets;
}

InverseDynamics MakeController(MockState state, MockModel model, double target_velocity_blend = 0.0) {
  return InverseDynamics(std::make_unique<MockModel>(model),
                         std::make_unique<MockState>(state),
                         true,
                         true,
                         1,
                         target_velocity_blend);
}

std::array<std::string, ModelInterface::N_LEGS> Go2FeetNames() {
  return {"fl_contact", "fr_contact", "bl_contact", "br_contact"};
}

std::array<std::string, ModelInterface::NUM_JOINTS> Go2JointNames() {
  return {"fl_abad",
          "fl_shoulder",
          "fl_knee",
          "fr_abad",
          "fr_shoulder",
          "fr_knee",
          "bl_abad",
          "bl_shoulder",
          "bl_knee",
          "br_abad",
          "br_shoulder",
          "br_knee"};
}

std::string Go2UrdfPath() {
  return (std::filesystem::path(__FILE__).parent_path().parent_path().parent_path().parent_path()
          / "src/common/model/urdf/go2/urdf/go2_description.urdf")
      .string();
}

WBCArcOPT MakeArcOpt(std::array<std::string, ModelInterface::NUM_JOINTS> joint_names = Go2JointNames()) {
  return WBCArcOPT(std::make_unique<MockState>(),
                   "QPOasesSolver",
                   "AccelerationSceneReducedTSID",
                   Go2UrdfPath(),
                   Go2FeetNames(),
                   joint_names,
                   0.45,
                   Eigen::Matrix<double, 6, 1>::Constant(10.0),
                   Eigen::Matrix<double, 3, 1>::Constant(10.0),
                   Eigen::Matrix<double, 3, 1>::Constant(30.0),
                   Eigen::Matrix<double, 6, 1>::Constant(100.0),
                   Eigen::Matrix<double, 6, 1>::Constant(10.0),
                   Eigen::Matrix<double, 3, 1>::Constant(100.0),
                   Eigen::Matrix<double, 3, 1>::Constant(10.0),
                   Eigen::Matrix<double, 6, 1>::Constant(1000.0),
                   Eigen::Matrix<double, 3, 1>::Constant(1000.0),
                   -1.0);
}

}  // namespace

TEST(InverseDynamicsTest, StanceLegUsesTargetFrameAndWrench) {
  MockState state;
  state.position = Eigen::Vector3d(1.0, 2.0, 0.25);
  MockModel model;
  auto controller = MakeController(state, model);

  auto feet_targets = MakeFeetTargets();
  feet_targets.positions[0] = Eigen::Vector3d(2.0, 4.0, 3.0);
  WBCInterface<CartesianCommands>::Wrenches wrenches{};
  for (auto& wrench : wrenches) {
    wrench.setZero();
  }
  wrenches[0] = Eigen::Vector3d(3.0, -2.0, 1.0);
  WBCInterface<CartesianCommands>::FootContact contacts{};
  contacts.fill(false);
  contacts[0] = true;

  controller.UpdateFeetTarget(feet_targets);
  controller.UpdateWrenches(wrenches);
  controller.UpdateFootContact(contacts);
  controller.UpdateTarget(Eigen::Quaterniond::Identity(),
                          Eigen::Vector3d(0.0, 0.0, 1.0),
                          Eigen::Vector3d::Zero(),
                          Eigen::Vector3d::Zero());

  CartesianCommands command;
  const auto ret = controller.GetJointCommand(command);

  EXPECT_TRUE(ret.success);
  EXPECT_TRUE(command.position[0].isApprox(Eigen::Vector3d(1.0, 2.0, 2.0)));
  EXPECT_TRUE(command.velocity[0].isApprox(Eigen::Vector3d::Zero()));
  EXPECT_TRUE(command.force[0].isApprox(Eigen::Vector3d(-3.0, 2.0, -1.0)));
}

TEST(InverseDynamicsTest, SwingLegUsesCurrentBodyFrameAndFootVelocity) {
  MockState state;
  state.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  MockModel model;
  auto controller = MakeController(state, model);

  auto feet_targets = MakeFeetTargets();
  feet_targets.positions[1] = Eigen::Vector3d(2.0, 4.0, 6.0);
  feet_targets.velocities[1] = Eigen::Vector3d(0.4, -0.2, 0.1);
  WBCInterface<CartesianCommands>::Wrenches wrenches{};
  for (auto& wrench : wrenches) {
    wrench.setZero();
  }
  WBCInterface<CartesianCommands>::FootContact contacts{};
  contacts.fill(false);

  controller.UpdateFeetTarget(feet_targets);
  controller.UpdateWrenches(wrenches);
  controller.UpdateFootContact(contacts);
  controller.UpdateTarget(Eigen::Quaterniond::Identity(),
                          Eigen::Vector3d::Zero(),
                          Eigen::Vector3d::Zero(),
                          Eigen::Vector3d::Zero());

  CartesianCommands command;
  const auto ret = controller.GetJointCommand(command);

  EXPECT_TRUE(ret.success);
  EXPECT_TRUE(command.position[1].isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(command.velocity[1].isApprox(Eigen::Vector3d(0.4, -0.2, 0.1)));
  EXPECT_TRUE(command.force[1].isApprox(Eigen::Vector3d::Zero()));
}

TEST(InverseDynamicsTest, TargetVelocityBlendIsClamped) {
  MockState state;
  state.linear_velocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  MockModel model;
  auto feet_targets = MakeFeetTargets();
  WBCInterface<CartesianCommands>::Wrenches wrenches{};
  for (auto& wrench : wrenches) {
    wrench.setZero();
  }
  WBCInterface<CartesianCommands>::FootContact contacts{};
  contacts.fill(false);

  auto controller = MakeController(state, model, 10.0);
  controller.UpdateFeetTarget(feet_targets);
  controller.UpdateWrenches(wrenches);
  controller.UpdateFootContact(contacts);
  controller.UpdateTarget(Eigen::Quaterniond::Identity(),
                          Eigen::Vector3d::Zero(),
                          Eigen::Vector3d(0.0, 2.0, 0.0),
                          Eigen::Vector3d::Zero());

  CartesianCommands command;
  EXPECT_TRUE(controller.GetJointCommand(command).success);
  EXPECT_TRUE(command.velocity[2].isApprox(Eigen::Vector3d(0.0, -2.0, 0.0)));

  controller.setTargetVelocityBlend(-10.0);
  EXPECT_TRUE(controller.GetJointCommand(command).success);
  EXPECT_TRUE(command.velocity[2].isApprox(Eigen::Vector3d(-1.0, 0.0, 0.0)));
}

TEST(InverseDynamicsTest, ZeroTransformationFilterSizeIsClampedToOne) {
  MockState state;
  state.linear_velocity = Eigen::Vector3d(1.0, 0.0, 0.0);
  MockModel model;
  auto controller = MakeController(state, model, 0.0);
  controller.setTransformationFilterSize(0);

  auto feet_targets = MakeFeetTargets();
  WBCInterface<CartesianCommands>::Wrenches wrenches{};
  for (auto& wrench : wrenches) {
    wrench.setZero();
  }
  WBCInterface<CartesianCommands>::FootContact contacts{};
  contacts.fill(false);

  controller.UpdateFeetTarget(feet_targets);
  controller.UpdateWrenches(wrenches);
  controller.UpdateFootContact(contacts);
  controller.UpdateTarget(Eigen::Quaterniond::Identity(),
                          Eigen::Vector3d::Zero(),
                          Eigen::Vector3d::Zero(),
                          Eigen::Vector3d::Zero());

  CartesianCommands command;
  EXPECT_TRUE(controller.GetJointCommand(command).success);
  EXPECT_TRUE(command.velocity[0].isApprox(Eigen::Vector3d(-1.0, 0.0, 0.0)));
}

TEST(WBCArcOPTTest, ConstructsWithGo2Configuration) {
  EXPECT_NO_THROW({ auto controller = MakeArcOpt(); });
}

TEST(WBCArcOPTTest, RejectsJointNamesMissingFromUrdf) {
  auto joint_names = Go2JointNames();
  joint_names[0] = "missing_joint";

  EXPECT_THROW({ auto controller = MakeArcOpt(joint_names); }, std::runtime_error);
}
