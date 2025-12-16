/******************************************************************************
Copyright (c) 2022, Halodi Robotics AS. All rights reserved.
 *
 * @package humanoid_centroidal_mpc
 *
 * @author Manuel Yves Galliker
 * Contact:  manuel.galliker@1x.tech
 *
 ******************************************************************************/

#include <pinocchio/fwd.hpp>

#include <chrono>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/joint/joint-composite.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ocs2_core/Types.h>
#include "humanoid_centroidal_mpc/common/CentroidalMpcRobotModel.h"
#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"
#include "humanoid_common_mpc/pinocchio_model/createPinocchioModel.h"

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace ocs2;
using namespace ocs2::humanoid;

void benchmarkInverseDynamics(PinocchioInterfaceTpl<scalar_t>& pinocchioInterface) {
  const int NUM_ITERATIONS = 10000;

  // Get dimensions from pinocchio interface
  const int nq = pinocchioInterface.getModel().nq;
  const int nv = pinocchioInterface.getModel().nv;
  const int njoints = nv - 6;  // Assuming floating base (6 DOF base + joint DOFs)

  // Random number generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<scalar_t> dis(0.0, 1.0);

  // Generate random test states
  std::vector<vector_t> q_states, qd_states, qdd_joints_states;
  std::vector<std::array<VECTOR6_T<scalar_t>, 2>> footWrenches_states;

  for (int i = 0; i < NUM_ITERATIONS; ++i) {
    // Random generalized coordinates
    vector_t q(nq);
    for (int j = 0; j < nq; ++j) {
      q[j] = dis(gen);
    }
    q_states.push_back(q);

    // Random generalized velocities
    vector_t qd(nv);
    for (int j = 0; j < nv; ++j) {
      qd[j] = dis(gen);
    }
    qd_states.push_back(qd);

    // Random joint accelerations
    vector_t qdd_joints(njoints);
    for (int j = 0; j < njoints; ++j) {
      qdd_joints[j] = dis(gen);
    }
    qdd_joints_states.push_back(qdd_joints);

    // Random foot wrenches
    std::array<VECTOR6_T<scalar_t>, 2> footWrenches;
    for (int foot = 0; foot < 2; ++foot) {
      for (int j = 0; j < 6; ++j) {
        footWrenches[foot][j] = dis(gen);
      }
    }
    footWrenches_states.push_back(footWrenches);
  }

  // Benchmark custom inverse dynamics
  double customAvg = 0.0;
  double rneaAvg = 0.0;

  for (int i = 0; i < NUM_ITERATIONS; i = i + 10) {
    auto start = std::chrono::high_resolution_clock::now();
    for (int j = 0; j < 10; j++) {
      auto result =
          computeJointTorques(q_states[i + j], qd_states[i + j], qdd_joints_states[i + j], footWrenches_states[i + j], pinocchioInterface);
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto customDuration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    customAvg += static_cast<double>(customDuration.count());

    // Benchmark RNEA inverse dynamics
    start = std::chrono::high_resolution_clock::now();

    for (int j = 0; j < 10; j++) {
      auto result = computeJointTorquesRNEA(q_states[i + j], qd_states[i + j], qdd_joints_states[i + j], footWrenches_states[i + j],
                                            pinocchioInterface);
    }

    end = std::chrono::high_resolution_clock::now();
    auto rneaDuration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    rneaAvg += static_cast<double>(rneaDuration.count());
  }

  customAvg = customAvg / NUM_ITERATIONS;
  rneaAvg = rneaAvg / NUM_ITERATIONS;

  // Print results
  std::cout << std::fixed << std::setprecision(2);
  std::cout << "Inverse Dynamics Benchmark Results (" << NUM_ITERATIONS << " iterations):\n";
  std::cout << "================================================\n";
  std::cout << "Custom Implementation:  " << customAvg << " μs average\n";
  std::cout << "RNEA Implementation:    " << rneaAvg << " μs average\n";
  std::cout << "Speed ratio (Custom/RNEA): " << customAvg / rneaAvg << "x\n";

  if (customAvg < rneaAvg) {
    std::cout << "Custom implementation is " << (rneaAvg / customAvg) << "x faster\n";
  } else {
    std::cout << "RNEA implementation is " << (customAvg / rneaAvg) << "x faster\n";
  }
}

void compareInverseDynamics(PinocchioInterfaceTpl<scalar_t>& pinocchioInterface) {
  const int NUM_ITERATIONS = 10;

  // Get dimensions from pinocchio interface
  const int nq = pinocchioInterface.getModel().nq;
  const int nv = pinocchioInterface.getModel().nv;
  const int njoints = nv - 6;  // Assuming floating base (6 DOF base + joint DOFs)

  // Random number generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<scalar_t> dis(0.0, 1.0);

  // Generate random test states
  std::vector<vector_t> q_states, qd_states, qdd_joints_states;
  std::vector<std::array<VECTOR6_T<scalar_t>, 2>> footWrenches_states;

  for (int i = 0; i < NUM_ITERATIONS; ++i) {
    // Random generalized coordinates
    vector_t q(nq);
    for (int j = 0; j < nq; ++j) {
      q[j] = dis(gen);
    }
    q_states.push_back(q);

    // Random generalized velocities
    vector_t qd(nv);
    for (int j = 0; j < nv; ++j) {
      qd[j] = dis(gen);
    }
    qd_states.push_back(qd);

    // Random joint accelerations
    vector_t qdd_joints(njoints);
    for (int j = 0; j < njoints; ++j) {
      qdd_joints[j] = dis(gen);
    }
    qdd_joints_states.push_back(qdd_joints);

    // Random foot wrenches
    std::array<VECTOR6_T<scalar_t>, 2> footWrenches;
    for (int foot = 0; foot < 2; ++foot) {
      for (int j = 0; j < 6; ++j) {
        footWrenches[foot][j] = dis(gen);
      }
    }
    footWrenches_states.push_back(footWrenches);
  }

  for (int i = 0; i < NUM_ITERATIONS; i++) {
    auto resultCustom = computeJointTorques(q_states[i], qd_states[i], qdd_joints_states[i], footWrenches_states[i], pinocchioInterface);

    auto resultRNEA = computeJointTorquesRNEA(q_states[i], qd_states[i], qdd_joints_states[i], footWrenches_states[i], pinocchioInterface);

    std::cout << "Result custom:" << resultCustom.transpose() << std::endl;
    std::cout << "Result rnea  :" << resultRNEA.transpose() << std::endl;
  }
}

/**
 * @brief This file contains Manu's personal pinocchio playground.
 */

void testOrientationErrorWrtPlane(const PinocchioInterface* pinocchioInterfacePtr, Eigen::VectorXd q) {
  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;

  pinocchio::Model model = pinocchioInterfacePtr->getModel();
  pinocchio::Data data = pinocchioInterfacePtr->getData();

  // Perform the forward kinematics over the kinematic tree
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  vector3_t error;
  vector3_t planeNormal(0.0, 0.0, 1.0);

  const size_t frameId = 69;
  vector3_t z_axis(0.0, 0.0, 1.0);

  // Rotation matrix local end effector frame to world frame
  matrix3_t R_w_l = data.oMf[frameId].rotation();

  // Passive rotation projecting from end effector frame to the closest frame in plane.
  // Computed through the shortest arc rotation  from the end effector z axis to the plane normal (both expressed in world frame).
  quaternion_t quaternion_correction = getQuaternionFromUnitVectors<scalar_t>(R_w_l * z_axis, planeNormal);

  std::cout << "quaternion_correction: " << quaternion_correction.coeffs() << std::endl;

  error = quaternionDistance(quaternion_correction, quaternion_t::Identity());
  std::cout << "error: " << error << std::endl;
}

void printModelDimensionality(PinocchioInterface pin_interface) {
  pinocchio::Model model = pin_interface.getModel();
  pinocchio::Data data = pin_interface.getData();

  std::cout << "model name: " << model.name << std::endl;
  std::cout << "n q: " << model.nq << std::endl;
  std::cout << "n v: " << model.nv << std::endl;
}

void printJointNames(PinocchioInterface pin_interface) {
  pinocchio::Model model = pin_interface.getModel();
  for (pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model.njoints; ++joint_id)
    std::cout << std::setw(24) << std::left << model.names[joint_id] << std::endl;
}

std::ostream& operator<<(std::ostream& os, const Eigen::Quaternion<double>& q) {
  os << "[" << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "]";
  return os;
}

void printFrameRotation(PinocchioInterface pin_interface, Eigen::VectorXd q, std::string& frameName) {
  pinocchio::Model model = pin_interface.getModel();
  pinocchio::Data data = pin_interface.getData();

  // Perform the forward kinematics over the kinematic tree
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  pinocchio::FrameIndex frameID = pin_interface.getModel().getFrameId(frameName);
  // Print out the placement of each joint of the kinematic tree
  matrix3_t R_w_l = data.oMf[frameID].rotation();
  auto q_w_l = matrixToQuaternion(R_w_l);
  auto translation = data.oMf[frameID].toHomogeneousMatrix_impl();  // translation from local into world frame
  std::cout << "Orientation of frame: R local to world " << frameName << ": " << std::endl;
  std::cout << q_w_l << std::endl;
  std::cout << R_w_l << std::endl;
  std::cout << "Translation from local to world frame " << frameName << ": " << std::endl;
  std::cout << translation << std::endl;
}

void computeForwardKinematics(PinocchioInterface pin_interface, Eigen::VectorXd q) {
  pinocchio::Model model = pin_interface.getModel();
  pinocchio::Data data = pin_interface.getData();

  // Perform the forward kinematics over the kinematic tree
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  // Print out the placement of each joint of the kinematic tree
  std::cout << "###########################################" << std::endl;
  std::cout << "############### Model Joints ##############" << std::endl;
  std::cout << "###########################################" << std::endl;
  for (pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model.njoints; ++joint_id)
    std::cout << std::setw(5) << std::left << "ID: " << joint_id << ", " << model.names[joint_id] << ": " << std::fixed
              << std::setprecision(5) << data.oMi[joint_id].translation().transpose() << std::endl;
  std::cout << "###########################################" << std::endl;
  std::cout << "############### Model Frames ##############" << std::endl;
  std::cout << "###########################################" << std::endl;
  for (pinocchio::FrameIndex frame_id = 0; frame_id < (pinocchio::FrameIndex)model.nframes; ++frame_id)
    std::cout << std::setw(10) << std::left << "ID: " << frame_id << ", name: " << model.frames[frame_id].name
              << " : Pos: " << std::setprecision(5) << data.oMf[frame_id].translation().transpose() << std::endl;
}

void computeInverseDyanmics(PinocchioInterface pin_interface, Eigen::VectorXd q, Eigen::VectorXd dq, Eigen::VectorXd ddq) {
  pinocchio::Model model = pin_interface.getModel();
  pinocchio::Data data = pin_interface.getData();

  const Eigen::VectorXd& tau = pinocchio::rnea(model, data, q, dq, ddq);
  std::cout << "tau = " << tau.transpose() << std::endl;
}

int main(int argc, char** argv) {
  const std::string path(__FILE__);
  const std::string dir = path.substr(0, path.find_last_of("/"));

  std::string urdfFile;
  try {
    urdfFile = ament_index_cpp::get_package_share_directory("g1_description") + "/urdf/g1_29dof.urdf";
  } catch (const std::exception& e) {
    throw std::runtime_error("Failed to get package share directory: g1_description. Error: " + std::string(e.what()));
  }

  const std::string taskFile = dir + "/../config/mpc/task.info";

  std::cout << "urdf filename: " << urdfFile << std::endl;

  /// Test default model

  PinocchioInterface pin_interface = createDefaultPinocchioInterface(urdfFile);

  std::cout << "Default PinocchioInterface initialized " << std::endl;

  printModelDimensionality(pin_interface);
  printJointNames(pin_interface);

  // Initialize states
  Eigen::VectorXd q = Eigen::VectorXd::Zero(35);
  q[2] = 0.8415;

  computeForwardKinematics(pin_interface, q);
  std::string leftFootFrameName("foot_l_contact");
  std::string rightFootFrameName("foot_r_contact");
  printFrameRotation(pin_interface, q, leftFootFrameName);

  /// Test custom model
  ModelSettings modelSettings(taskFile, urdfFile, "test_pinocchio", "true");

  pin_interface = createCustomPinocchioInterface(taskFile, urdfFile, modelSettings);

  std::cout << "Custom PinocchioInterface initialized " << std::endl;

  printModelDimensionality(pin_interface);
  printJointNames(pin_interface);

  // Initialize states
  q = Eigen::VectorXd::Zero(29);
  q[2] = 0.7925;
  // q[6] = 0.5;

  computeForwardKinematics(pin_interface, q);
  printFrameRotation(pin_interface, q, rightFootFrameName);
  printFrameRotation(pin_interface, q, leftFootFrameName);

  testOrientationErrorWrtPlane(&pin_interface, q);

  /// Test custom model with mass scaling

  pin_interface = createCustomPinocchioInterface(taskFile, urdfFile, modelSettings, true, 44.44);

  benchmarkInverseDynamics(pin_interface);

  compareInverseDynamics(pin_interface);

  return 0;
}
