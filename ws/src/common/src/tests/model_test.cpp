#include <drake/geometry/geometry_frame.h>
#include <drake/math/rigid_transform.h>

#include <Eigen/Dense>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <map>
#include <ostream>
#include <quad_model.hpp>
#include <quad_model_pino.hpp>
#include <quad_model_symbolic.hpp>
#include <string>

#include "drake/common/drake_assert.h"
#include "drake/common/find_resource.h"
#include "drake/geometry/scene_graph.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"

using drake::geometry::SceneGraph;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MultibodyPlant;
using drake::multibody::Parser;
using drake::systems::Context;
using drake::systems::DiagramBuilder;

const std::string URDF_PATH = "./src/go2_description/urdf/go2_description.urdf";

// void Gravity_vector(Eigen::Matrix<double, 3, 1>& gravity_vector, double q1, double q2, double q3) {
//   double sub0_14_0 = std::cos(q1);
//   double sub0_14_1 = std::sin(q2);
//   double sub0_14_2 = std::cos(q2);
//   double sub0_14_3 = q2 + q3;
//   double sub0_14_4 = 0.025086377858665368 * std::sin(sub0_14_3);
//   gravity_vector << 0.50869676571933908 * sub0_14_0
//                         + (-0.0022965054265986252 * sub0_14_1 + 0.26111583862911103 * sub0_14_2
//                            + 0.025086377858665368 * std::cos(sub0_14_3))
//                               * std::sin(q1),
//       sub0_14_0 * (0.26111583862911097 * sub0_14_1 + 0.0022965054265986252 * sub0_14_2 + sub0_14_4),
//       sub0_14_0 * sub0_14_4;
// }

// Eigen::Matrix<double, 3, 1> Gravity_vector(double q1, double q2, double q3) {
//   Eigen::Matrix<double, 3, 1> gravity_vector;
//   Gravity_vector(gravity_vector, q1, q2, q3);
//   return gravity_vector;
// }

// void Gravity_vector(Eigen::Matrix<double, 3, 1>& gravity_vector, const Eigen::Matrix<double, 3, 1>& q) {
//   Gravity_vector(gravity_vector, q[0], q[1], q[2]);
// }

// Eigen::Matrix<double, 3, 1> Gravity_vector(const Eigen::Matrix<double, 3, 1>& q) {
//   return Gravity_vector(q[0], q[1], q[2]);
// }

// void generalized_mass_inertia_matrix(Eigen::Matrix<double, 3, 3>& M, double q2, double q3) {
//   double sub0_12_0 = std::sin(q2);
//   double sub0_12_1 = std::cos(q2);
//   double sub0_12_2 = q2 + q3;
//   double sub0_12_3 = std::sin(sub0_12_2);
//   double sub0_12_4 = std::cos(sub0_12_2);
//   double sub0_12_5 = 0.00022708158550963149 * sub0_12_3;
//   double sub0_12_6 = 0.0020116722276856559 * sub0_12_0 + sub0_12_5;
//   double sub0_12_7 = std::cos(q3);
//   double sub0_12_8 = 0.00038358375930680992 * sub0_12_7;
//   double sub0_12_9 = sub0_12_8 + 0.00023310172625214202;
//   M << 0.0031314194551247398 * std::pow(sub0_12_0, 2.0) + 0.0037775204959576 * std::pow(sub0_12_1, 2.0)
//            + 0.14999999999999999 * sub0_12_1 * (0.014818664672012309 * sub0_12_1 + 0.0025572250620453994 * sub0_12_4)
//            + 0.00077901127434181828 * std::pow(sub0_12_3, 2.0) + 0.00077901127434181828 * std::pow(sub0_12_4, 2.0)
//            + sub0_12_4 * (0.00038358375930680992 * sub0_12_1 + 0.00021687107073270701 * sub0_12_4)
//            + 0.00037984234138564,
//       sub0_12_6, sub0_12_5, sub0_12_6,
//       -0.14999999999999999 * sub0_12_7 * (-0.014818664672012309 * sub0_12_7 - 0.0025572250620453994) + sub0_12_8
//           + 0.0022227997008018464 * std::pow(std::sin(q3), 2.0) + 0.0016799192446349022,
//       sub0_12_9, sub0_12_5, sub0_12_9, 0.00023310172625214199;
// }

// void generalized_mass_inertia_matrix(Eigen::Matrix<double, 3, 3>& M, const Eigen::Matrix<double, 3, 1>& q) {
//   generalized_mass_inertia_matrix(M, q[1], q[2]);
// }

// void Hybrid_jacobian_matrix_ee(Eigen::Matrix<double, 6, 3>& hybrid_jacobian_matrix_ee,
//                                double q1,
//                                double q2,
//                                double q3) {
//   double sub0_11_0 = std::cos(q1);
//   double sub0_11_1 = std::sin(q1);
//   double sub0_11_2 = 0.14999999999999999 * std::cos(q2);
//   double sub0_11_3 = q2 + q3;
//   double sub0_11_4 = 0.13 * std::cos(sub0_11_3);
//   double sub0_11_5 = -sub0_11_4;
//   double sub0_11_6 = sub0_11_2 + sub0_11_4;
//   double sub0_11_7 = 0.14999999999999999 * std::sin(q2);
//   double sub0_11_8 = 0.13 * std::sin(sub0_11_3);
//   hybrid_jacobian_matrix_ee << 1.0, 0.0, 0.0, 0.0, sub0_11_0, sub0_11_0, 0.0, sub0_11_1, sub0_11_1, 0.0,
//       -sub0_11_2 + sub0_11_5, sub0_11_5, sub0_11_0 * sub0_11_6 - 0.088800000000000004 * sub0_11_1,
//       sub0_11_1 * (-sub0_11_7 - sub0_11_8), -sub0_11_1 * sub0_11_8,
//       0.088800000000000004 * sub0_11_0 + sub0_11_1 * sub0_11_6, sub0_11_0 * (sub0_11_7 + sub0_11_8),
//       sub0_11_0 * sub0_11_8;
// }

// Eigen::Matrix<double, 6, 3> Hybrid_jacobian_matrix_ee(double q1, double q2, double q3) {
//   Eigen::Matrix<double, 6, 3> hybrid_jacobian_matrix_ee;
//   Hybrid_jacobian_matrix_ee(hybrid_jacobian_matrix_ee, q1, q2, q3);
//   return hybrid_jacobian_matrix_ee;
// }

// void Hybrid_jacobian_matrix_ee(Eigen::Matrix<double, 6, 3>& hybrid_jacobian_matrix_ee,
//                                const Eigen::Matrix<double, 3, 1>& q) {
//   Hybrid_jacobian_matrix_ee(hybrid_jacobian_matrix_ee, q[0], q[1], q[2]);
// }

// Eigen::Matrix<double, 6, 3> Hybrid_jacobian_matrix_ee(const Eigen::Matrix<double, 3, 1>& q) {
//   return Hybrid_jacobian_matrix_ee(q[0], q[1], q[2]);
// }
Eigen::Matrix3d skew(const Eigen::Ref<const Eigen::Vector3d> vec) {
  Eigen::Matrix3d skew;
  skew << 0.0, -vec(2), vec(1), vec(2), 0.0, -vec(0), -vec(1), vec(0), 0.0;
  return skew;
}

int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;
  // drake initializations
  DiagramBuilder<double> builder;
  MultibodyPlant<double>* quad;
  SceneGraph<double>* scene_graph;
  // add quad (multibodyplant) to scene_graph
  std::tie(quad, scene_graph) = AddMultibodyPlantSceneGraph(&builder, 0.0);
  // initialize URDF parser
  auto parser = Parser(quad, scene_graph);
  parser.package_map().PopulateFromRosPackagePath();
  // load from URDF
  auto quadmodel_instance_index = parser.AddModels(URDF_PATH)[0];
  quad->WeldFrames(quad->world_frame(), quad->GetFrameByName("base_link"));
  quad->Finalize();
  std::unique_ptr<drake::systems::Context<double>> context = quad->CreateDefaultContext();

  // drake quad dynamic quantities initialization
  Eigen::Matrix<double, 3, 12> J;   // jacobian
  Eigen::Matrix<double, 12, 12> M;  // mass matrix
  Eigen::Vector<double, 12> Cv;     // coriolis vector
  Eigen::Vector<double, 12> G;      // gravity forces
  Eigen::Vector3d eeposf;
  drake::math::RigidTransform<double> T_wb;  // transformation world to body

  // set zero configuration
  Eigen::Vector<double, 12> q_drake = Eigen::Vector<double, 12>::Zero();
  // q0(0) = 1.0;  // q starts with quaternion for orientation -> q0(0) == quaternion w
  Eigen::Vector<double, 12> qd_drake = Eigen::Vector<double, 12>::Zero();
  Eigen::Vector<double, 12> qdd_drake = Eigen::Vector<double, 12>::Zero();
  Eigen::Vector<double, 12> tau0 = Eigen::Vector<double, 12>::Zero();
  drake::multibody::MultibodyForces external_forces_drake(*quad);
  external_forces_drake.SetZero();
  for (uint i = 0; i < 12; i++) {
    q_drake(i) = 0.3;
    qd_drake(i) = 0.1;
    qdd_drake(i) = 0.05;
  }
  quad->SetPositions(context.get(), q_drake);
  quad->SetVelocities(context.get(), qd_drake);
  quad->CalcMassMatrix(*context, &M);
  quad->CalcBiasTerm(*context, &Cv);
  G = quad->CalcGravityGeneralizedForces(*context);
  quad->CalcBiasTerm(*context, &Cv);
  quad->CalcMassMatrix(*context, &M);

  // create quadmodel pino
  std::array<std::string, 4> feet_names = {
      "fl_contact_frame", "fr_contact_frame", "bl_contact_frame", "br_contact_frame"};
  std::string base_link_name = "base_link";
  std::array<std::string, 12> joint_names = {"fl_abad",
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
  std::array<double, ModelInterface::NUM_JOINTS> init_q;
  init_q.fill(0.3);
  QuadModelPino quad_pin(URDF_PATH, feet_names, base_link_name, joint_names, init_q);
  Eigen::Vector3d q_pin, qd_pin, qdd_pin, tau_pin;
  q_pin << 0.3, 0.3, 0.3;
  qd_pin << 0.1, 0.1, 0.1;
  qdd_pin << 0.05, 0.05, 0.05;
  tau_pin << 0.2, 0.2, 0.2;

  QuadModelSymbolic quad_sym(QuadModelSymbolic::GO2_QUAD);

  // ----------Start Tests-----------

  // Check Consistency of Parameters
  // ----------link lengths----------
  // std::cout << "---Geometric Quantities---" << std::endl;
  // auto T =
  //     quad->CalcRelativeTransform(*context, quad->GetFrameByName("fl_HAA_link"),
  //     quad->GetFrameByName("fl_KFE_link"));
  // auto leg_lengths_pin = quad_pin.getLegLinkLengths();

  // std::cout << "\nl0:" << std::endl;
  // std::cout << "Urdf:\t" << T.translation()[1] << std::endl;
  // std::cout << "pin:\t" << leg_lengths_pin[0] << std::endl;
  // T = quad->CalcRelativeTransform(*context, quad->GetFrameByName("fl_HFE_link"),
  // quad->GetFrameByName("fl_KFE_link")); std::cout << "\nl1:" << std::endl; std::cout << "Urdf:\t" <<
  // -T.translation()[2] << std::endl; std::cout << "pin:\t" << leg_lengths_pin[1] << std::endl;

  // T = quad->CalcRelativeTransform(*context, quad->GetFrameByName("fl_KFE_link"),
  // quad->GetFrameByName("fl_contact"));

  // std::cout << "\nl2:" << std::endl;
  // std::cout << "Urdf:\t" << -T.translation()[2] << std::endl;
  // std::cout << "pin:\t" << leg_lengths_pin[2] << std::endl;

  // ----------IMU offset----------
  drake::math::RigidTransform T_bimu =
      quad->CalcRelativeTransform(*context, quad->GetFrameByName("base_link"), quad->GetFrameByName("link_imu"));
  Eigen::Vector3d imu_offset = T_bimu.translation();

  std::cout << "\n---IMU offset:---" << std::endl;
  std::cout << "Urdf:\n" << imu_offset << std::endl;
  std::cout << "pin:\n" << quad_pin.GetBodyToIMU().vector().array() << std::endl;
  std::cout << "sym:\n" << quad_sym.GetBodyToIMU().vector().array() << std::endl;

  // ----------inertia of base + total mass----------

  std::cout << "\n---Total mass:---" << std::endl;
  std::cout << "Urdf:\t" << quad->CalcTotalMass(*context) << std::endl;
  std::cout << "pin:\t" << quad_pin.GetMass() << std::endl;
  std::cout << "sym:\t" << quad_sym.GetMass() << std::endl;

  std::cout << "\n---COM:---" << std::endl;
  std::cout << "Urdf:\t" << quad->CalcCenterOfMassPositionInWorld(*context) << std::endl;
  std::cout << "pin:\t" << quad_pin.GetBodyToCOM().translation().array() << std::endl;
  std::cout << "sym:\t" << quad_sym.GetBodyToCOM().translation().array() << std::endl;

  std::cout << "\n---BellyB:---" << std::endl;
  std::cout << "pin:\t" << quad_pin.GetBodyToBellyBottom().translation().array() << std::endl;
  std::cout << "sym:\t" << quad_sym.GetBodyToBellyBottom().translation().array() << std::endl;

  std::cout << "\n---Gravity:---" << std::endl;
  std::cout << "pin:\t" << quad_pin.GetG() << std::endl;
  std::cout << "sym:\t" << quad_sym.GetG() << std::endl;

  std::vector<drake::multibody::BodyIndex> body_indices = quad->GetBodyIndices(quadmodel_instance_index);
  const drake::multibody::SpatialInertia<double> I =
      quad->CalcSpatialInertia(*context, quad->GetFrameByName("base_link"), body_indices);
  auto base_rotational_inertia = I.CalcRotationalInertia();
  base_rotational_inertia.ShiftToCenterOfMassInPlace(I.get_mass(), I.get_com());
  auto base_inertia = base_rotational_inertia.CopyToFullMatrix3();

  std::cout << "\n---Total inerta---" << std::endl;
  std::cout << "Urdf:\n" << base_inertia << std::endl;
  std::cout << "pin:\n" << quad_pin.GetInertia() << std::endl;
  std::cout << "sym:\n" << quad_sym.GetInertia() << std::endl;

  // ----------dynamic quantities and kinematics----------
  std::array<std::string, 4> leg_names = {"fl", "fr", "bl", "br"};
  for (int i = 0; i < 4; i++) {
    std::cout << "\n ===== Leg " << i << " =====" << std::endl;
    // -------forward kinematics-------
    std::cout << "\n---Forward Kinematics---" << std::endl;
    auto T = quad->CalcRelativeTransform(
        *context, quad->GetFrameByName("base_link"), quad->GetFrameByName(leg_names[i] + "_contact"));
    Eigen::Vector3d foot_pos_body;
    foot_pos_body.setZero();
    foot_pos_body = quad_pin.CalcFootPositionInBodyFrame(i, q_pin);

    std::cout << "\n---BellyB:---" << std::endl;
    std::cout << "pin:\t" << quad_pin.GetBodyToBellyBottom(i).translation() << std::endl;
    std::cout << "sym:\t" << quad_sym.GetBodyToBellyBottom(i).translation() << std::endl;

    std::cout << "Fkin of Leg in body frame:" << i << std::endl;
    std::cout << "Drake:\n" << T << std::endl;
    std::cout << "pin:\n" << foot_pos_body << std::endl;

    // -----inverse kinematics-----
    // ---legframe---
    // Eigen::Vector3d foot_pos_leg;
    // Eigen::Matrix4d T_foot_leg;
    // foot_pos_leg.setZero();
    // quad_pin.CalcFwdKinLegBody(i, q_pin, T_foot_leg, foot_pos_leg);
    // Eigen::Vector3d q_inv_kin_leg;
    // // quad_pin.calcLegInverseKinematicsInLegFrame(i, foot_pos_leg, Eigen::Vector3d(0, 0, 0), q_inv_kin);
    // std::cout << "\n---Inverse Kinematics---" << std::endl;
    // std::cout << "Goal Joint State:\n" << q_pin << std::endl;
    // quad_pin.calcLegInverseKinematicsInLegFrame(i, foot_pos_leg, Eigen::Vector3d(-0.06, -0.18, 0.17), q_inv_kin_leg);
    // std::cout << "Joint State from Inv Kin leg " << i << " in legbase frame:\n" << q_inv_kin_leg << std::endl;
    // std::cout << "Remember the initial guess of the joint angles impacts the result!" << std::endl;
    // ---body frame---
    // foot_pos_body is already calculated in forward kinematics
    Eigen::Vector3d q_inv_kin_body;
    // quad_pin.calcLegInverseKinematicsInLegFrame(i, foot_pos_leg, Eigen::Vector3d(0, 0, 0), q_inv_kin);
    quad_pin.CalcLegInverseKinematicsInBody(i, foot_pos_body, Eigen::Vector3d(0.25, 0.25, 0.25), q_inv_kin_body);
    std::cout << "Joint State from Inv Kin leg " << i << " in body frame:\n" << q_inv_kin_body << std::endl;
    std::cout << "Remember the initial guess of the joint angles impacts the result!" << std::endl;
    // -----inverse dynamics-----
    // std::cout << "\n---Inverse Dynamics---" << std::endl;
    Eigen::Vector<double, 12> tau_invdyn_drake = quad->CalcInverseDynamics(*context, qdd_drake, external_forces_drake)
                                                 + quad->CalcGravityGeneralizedForces(*context);
    // std::cout << "Torque Vector from Inverse Dynamics" << std::endl;
    // std::cout << "Drake:\n" << tau_invdyn_drake.segment<3>(i * 3) << std::endl;
    // Eigen::Vector3d tau_invdyn_pin;
    // quad_pin.computeInverseDynamics(i, q_pin, qd_pin, qdd_pin, tau_invdyn_pin);
    // std::cout << "pin:\n" << tau_invdyn_pin << std::endl;
    //  -----leg jacobians-----
    std::cout << "\n---Leg Jacobians---" << std::endl;
    Eigen::Matrix<double, 3, 12> J_temp;
    quad->CalcJacobianTranslationalVelocity(*context,
                                            drake::multibody::JacobianWrtVariable::kQDot,
                                            quad->GetFrameByName(leg_names[i] + "_contact"),
                                            Eigen::Vector3d(0, 0, 0),
                                            quad->GetFrameByName("base_link"),
                                            quad->GetFrameByName("base_link"),
                                            &J_temp);
    // std::cout << "temp:\n" << J_temp << std::endl;
    Eigen::Matrix3d J_drake;
    J_drake << J_temp.block(0, 3 * i, 3, 3);
    Eigen::Matrix3d J_pin, J_sym;
    // Eigen::Matrix<double, 6, 3> hybrid_jacobian_matrix;
    quad_pin.CalcJacobianLegBase(i, q_pin, J_pin);
    quad_pin.CalcJacobianLegBase(i, q_pin, J_sym);
    // Hybrid_jacobian_matrix_ee(hybrid_jacobian_matrix, Eigen::Vector3d::Zero());

    std::cout << "\nBase to Foot Jacobian of Leg:" << i << std::endl;
    // std::cout << "temp:\n" << J_temp << std::endl;
    std::cout << "Drake:\n" << J_drake << std::endl;
    std::cout << "pin:\n" << J_pin << std::endl;
    std::cout << "sym:\n" << J_sym << std::endl;
    // std::cout << "Skiddy pin:\n" << hybrid_jacobian_matrix << std::endl;

    // foot force and velocities
    Eigen::Vector3d fee_drake;
    auto J_drake_trans_inv = J_drake.transpose().completeOrthogonalDecomposition().pseudoInverse();
    fee_drake = J_drake_trans_inv * (tau_pin - tau_invdyn_drake.segment<3>(i * 3));
    auto vee_drake = quad->EvalBodySpatialVelocityInWorld(*context, quad->GetBodyByName(leg_names[i] + "_contact"));
    Eigen::Vector3d fee_pin, vee_pin;
    quad_pin.CalcFootForceVelocityInBodyFrame(i, q_pin, qd_pin, qdd_pin, tau_pin, fee_pin, vee_pin);
    Eigen::Vector3d fee_sym, vee_sym;
    quad_sym.CalcFootForceVelocityInBodyFrame(i, q_pin, qd_pin, qdd_pin, tau_pin, fee_sym, vee_sym);
    std::cout << "\n-----Foot Forces-----" << std::endl;
    std::cout << "\n Foot " << i << " force:" << std::endl;
    std::cout << "Drake:\n" << fee_drake << std::endl;
    std::cout << "pin:\n" << fee_pin << std::endl;
    std::cout << "sym:\n" << fee_sym << std::endl;

    std::cout << "\n-----Foot Velocities-----" << std::endl;
    std::cout << "\n Foot " << i << " velocity:" << std::endl;
    std::cout << "Drake:\n" << vee_drake.translational() << std::endl;
    std::cout << "pin:\n" << vee_pin << std::endl;
    std::cout << "sym:\n" << vee_sym << std::endl;

    // std::cout << "\n---Dynamic Quantities---" << std::endl;
    // Eigen::Matrix3d M_drake = M.block(i * 3, i * 3, 3, 3);
    // Eigen::Matrix3d M_pin;
    // quad_pin.calcGeneralizedMassInetiaMatrix(i, q_pin, M_pin);
    // Eigen::Vector3d Cv_drake = Cv.block(i * 3, 0, 3, 1);
    // Eigen::Vector3d G_drake = G.block(i * 3, 0, 3, 1);
    // Eigen::Vector3d G_pin;
    // Eigen::Matrix3d Cv_pin;
    // quad_pin.calcGeneralizedCoriolisCentrifugalMatrix(i, q_pin, qd_pin, Cv_pin);
    // quad_pin.calcGravityVector(i, q_pin, G_pin);
    // std::cout << "\nMass Matrix Leg " << i << " Drake:\n" << M << std::endl;
    // std::cout << "\nMass Matrix Leg " << i << " pin:\n" << M_pin << std::endl;

    // std::cout << "\nCoriolis Term Leg " << i << " Drake:\n" << Cv_drake << std::endl;
    // std::cout << "\nCoriolis Term Leg " << i << " pin:\n" << Cv_pin * qd_pin << std::endl;

    // std::cout << "\ngravity Term Leg " << i << " Drake:\n" << G_drake << std::endl;
    // std::cout << "\ngravity Term Leg " << i << " pin:\n" << G_pin << std::endl;

    std::cout << "\n-----CalcLegDiffKinematicsBodyFrame-----" << std::endl;
    QuadState state;
    interfaces::msg::QuadState msg;
    msg.joint_state.position.fill(0.21);
    state.UpdateFromMsg(msg);
    Eigen::Vector3d f_goal(10, 10, 30);
    Eigen::Vector3d v_goal(1., 1., 0.5);

    Eigen::Vector3d tau_pino, tau_sym, qd_pino, qd_sym;
    quad_pin.CalcLegDiffKinematicsBodyFrame(i, state, f_goal, v_goal, tau_pin, qd_pin);
    quad_sym.CalcLegDiffKinematicsBodyFrame(i, state, f_goal, v_goal, tau_sym, qd_sym);

    std::cout << "\n Foot " << i << " tau:" << std::endl;
    std::cout << "pin:\n" << tau_pin << std::endl;
    std::cout << "sym:\n" << tau_sym << std::endl;

    std::cout << "\n Foot " << i << " qd:" << std::endl;
    std::cout << "pin:\n" << qd_pin << std::endl;
    std::cout << "sym:\n" << qd_sym << std::endl;

    std::cout << "\n-----CalcFwdKinLegBody-----" << std::endl;
    Eigen::Matrix4d unused;
    Eigen::Vector3d ee_pin, ee_sym;
    quad_pin.CalcFwdKinLegBody(i, q_pin, unused, ee_pin);
    quad_sym.CalcFwdKinLegBody(i, q_pin, unused, ee_sym);

    std::cout << "\n Foot " << i << " ee:" << std::endl;
    std::cout << "pin:\n" << ee_pin << std::endl;
    std::cout << "sym:\n" << ee_sym << std::endl;
  }

  std::cout << "Testing to set new mass (Changing base link to 10Kgs)" << std::endl;
  quad_pin.SetMass(10);
  std::cout << "New Mass " << quad_pin.GetMass() << std::endl;
  std::cout << "Testing to set new mass (Changing base link to 20Kgs)" << std::endl;
  quad_pin.SetMass(20);
  std::cout << "New Mass " << quad_pin.GetMass() << std::endl;

  // -----regressor matrix-----
  interfaces::msg::QuadState qs;
  qs.pose.pose.position.x = 0.0;
  qs.pose.pose.position.y = 0.0;
  qs.pose.pose.position.z = 0.0;
  qs.pose.pose.orientation.x = 0.0;
  qs.pose.pose.orientation.y = 0.0;
  qs.pose.pose.orientation.z = 0.0;
  qs.pose.pose.orientation.w = 1.0;
  qs.twist.twist.linear.x = 0.0;
  qs.twist.twist.linear.y = 0.0;
  qs.twist.twist.linear.z = 0.0;
  qs.twist.twist.angular.x = 0.0;
  qs.twist.twist.angular.y = 0.0;
  qs.twist.twist.angular.z = 0.0;
  qs.acceleration.linear.x = 0.0;
  qs.acceleration.linear.y = 0.0;
  qs.acceleration.linear.z = 0.0;
  qs.acceleration.angular.x = 0.0;
  qs.acceleration.angular.y = 0.0;
  qs.acceleration.angular.z = 0.0;
  qs.joint_state.position.fill(0.0);
  qs.joint_state.velocity.fill(0.0);
  qs.joint_state.acceleration.fill(0.000);
  Eigen::Matrix<double, 18, 130> regressor;
  quad_pin.ComputeRegressorMatrix(QuadState(qs), regressor);
  for (int i = 0; i < 13; i++) {
    std::cout << "Regressor Matrix Body " << i << ":\n"
              << regressor.block<regressor.rows(), 10>(0, 10 * i) << std::endl;
  }
  std::cout << "Regressor Full Dynamic config\n " << quad_pin.GetAllDynamicParameters() << std::endl;
}
