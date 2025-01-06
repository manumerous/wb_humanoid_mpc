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

#include <iostream>

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
#include "humanoid_common_mpc/pinocchio_model/createPinocchioModel.h"

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_robotic_tools/common/SkewSymmetricMatrix.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace ocs2;
using namespace ocs2::humanoid;

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

  return 0;
}
