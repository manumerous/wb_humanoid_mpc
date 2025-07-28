/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
Copyright (c) 2024, 1X Technologies. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <iostream>
#include <string>

// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include "humanoid_wb_mpc/WBMpcInterface.h"

#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/Numerics.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include <humanoid_common_mpc/pinocchio_model/createPinocchioModel.h>
#include "humanoid_common_mpc/HumanoidCostConstraintFactory.h"
#include "humanoid_common_mpc/initialization/WeightCompInitializer.h"

#include "humanoid_wb_mpc/WBMpcPreComputation.h"
#include "humanoid_wb_mpc/constraint/JointMimicDynamicsConstraint.h"
#include "humanoid_wb_mpc/constraint/SwingLegVerticalConstraintCppAd.h"
#include "humanoid_wb_mpc/constraint/ZeroAccelerationConstraintCppAd.h"
#include "humanoid_wb_mpc/cost/EndEffectorDynamicsFootCost.h"
#include "humanoid_wb_mpc/cost/JointTorqueCostCppAd.h"
#include "humanoid_wb_mpc/dynamics/WBAccelDynamicsAD.h"
#include "humanoid_wb_mpc/end_effector/PinocchioEndEffectorDynamicsCppAd.h"

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
WBMpcInterface::WBMpcInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool setupOCP)
    : taskFile_(taskFile), urdfFile_(urdfFile), referenceFile_(referenceFile), modelSettings_(taskFile, urdfFile, "wb_mpc_", "true") {
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[WBMpcInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[WBMpcInterface] Task file not found: " + taskFilePath.string());
  }
  // check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile);
  if (boost::filesystem::exists(urdfFilePath)) {
    std::cerr << "[WBMpcInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  } else {
    throw std::invalid_argument("[WBMpcInterface] URDF file not found: " + urdfFilePath.string());
  }
  // check that targetCommand file exists
  boost::filesystem::path referenceFilePath(referenceFile);
  if (boost::filesystem::exists(referenceFilePath)) {
    std::cerr << "[WBMpcInterface] Loading target command settings from: " << referenceFilePath << std::endl;
  } else {
    throw std::invalid_argument("[WBMpcInterface] targetCommand file not found: " + referenceFilePath.string());
  }

  loadData::loadCppDataType(taskFile, "interface.verbose", verbose_);

  // load setting from loading file
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose_);
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose_);
  rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose_);
  sqpSettings_ = sqp::loadSettings(taskFile, "multiple_shooting", verbose_);

  // PinocchioInterface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(createCustomPinocchioInterface(taskFile, urdfFile, modelSettings_)));

  // Setup WB State Input Mapping
  mpcRobotModelPtr_.reset(new WBAccelMpcRobotModel<scalar_t>(modelSettings_));
  mpcRobotModelADPtr_.reset(new WBAccelMpcRobotModel<ad_scalar_t>(modelSettings_));

  // Swing trajectory planner
  std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPlanner(
      new SwingTrajectoryPlanner(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config", verbose_), N_CONTACTS));

  // Mode schedule manager
  referenceManagerPtr_ =
      std::make_shared<SwitchedModelReferenceManager>(GaitSchedule::loadGaitSchedule(referenceFile, modelSettings_, verbose_),
                                                      std::move(swingTrajectoryPlanner), *pinocchioInterfacePtr_, *mpcRobotModelPtr_);
  referenceManagerPtr_->setArmSwingReferenceActive(true);

  // initial state
  initialState_.setZero(mpcRobotModelPtr_->getStateDim());
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);

  if (setupOCP) {
    setupOptimalControlProblem();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void WBMpcInterface::setupOptimalControlProblem() {
  HumanoidCostConstraintFactory factory =
      HumanoidCostConstraintFactory(taskFile_, referenceFile_, *referenceManagerPtr_, *pinocchioInterfacePtr_, *mpcRobotModelPtr_,
                                    *mpcRobotModelADPtr_, modelSettings_, verbose_);

  // Optimal control problem
  problemPtr_.reset(new OptimalControlProblem);

  // Dynamics
  std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
  const std::string modelName = "dynamics";
  dynamicsPtr.reset(new WBAccelDynamicsAD(*pinocchioInterfacePtr_, *mpcRobotModelADPtr_, modelName, modelSettings_));

  problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

  // Cost terms
  problemPtr_->costPtr->add("stateInputQuadraticCost", factory.getStateInputQuadraticCost());
  // problemPtr_->costPtr->add("jointTorqueCost", getJointTorqueCost(taskFile_));
  problemPtr_->finalCostPtr->add("terminalCost", factory.getTerminalCost());

  // Constraints
  problemPtr_->stateSoftConstraintPtr->add("jointLimits", factory.getJointLimitsConstraint());
  problemPtr_->stateSoftConstraintPtr->add("FootCollisionSoftConstraint", factory.getFootCollisionConstraint());
  // Constraint terms

  EndEffectorDynamicsWeights footTrackingCostWeights =
      EndEffectorDynamicsWeights::getWeights(taskFile_, "task_space_foot_cost_weights.", verbose_);

  // check for mimic joints
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);
  bool hasMimicJoints = loadData::containsPtreeValueFind(pt, "mimicJoints");

  for (size_t i = 0; i < N_CONTACTS; i++) {
    const std::string& footName = modelSettings_.contactNames[i];

    std::unique_ptr<EndEffectorDynamics<scalar_t>> eeDynamicsPtr;
    eeDynamicsPtr.reset(new PinocchioEndEffectorDynamicsCppAd(*pinocchioInterfacePtr_, *mpcRobotModelADPtr_, {footName}, footName,
                                                              modelSettings_.modelFolderCppAd, modelSettings_.recompileLibrariesCppAd,
                                                              modelSettings_.verboseCppAd));

    problemPtr_->softConstraintPtr->add(footName + "_frictionForceCone", factory.getFrictionForceConeConstraint(i));
    problemPtr_->softConstraintPtr->add(footName + "_contactMomentXY",
                                        factory.getContactMomentXYConstraint(i, footName + "_contact_moment_XY_constraint"));
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroWrench", factory.getZeroWrenchConstraint(i));
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroVelocity", getStanceFootConstraint(*eeDynamicsPtr, i));
    problemPtr_->equalityConstraintPtr->add(footName + "_normalVelocity", getNormalVelocityConstraint(*eeDynamicsPtr, i));

    if (hasMimicJoints) {
      problemPtr_->equalityConstraintPtr->add(footName + "_kneeJointMimic", getJointMimicConstraint(i));
    }

    std::string footTrackingCostName = footName + "_TaskSpaceTrackingCost";

    problemPtr_->costPtr->add(footTrackingCostName, std::unique_ptr<StateInputCost>(new EndEffectorDynamicsFootCost(
                                                        *referenceManagerPtr_, footTrackingCostWeights, *pinocchioInterfacePtr_,
                                                        *eeDynamicsPtr, *mpcRobotModelADPtr_, i, footTrackingCostName, modelSettings_)));
  }

  // Pre-computation
  problemPtr_->preComputationPtr.reset(
      new WBMpcPreComputation(*pinocchioInterfacePtr_, *referenceManagerPtr_->getSwingTrajectoryPlanner(), *mpcRobotModelPtr_));

  // Rollout
  rolloutPtr_.reset(new TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings_));

  // Initialization
  initializerPtr_.reset(new WeightCompInitializer(*pinocchioInterfacePtr_, *referenceManagerPtr_, *mpcRobotModelPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputConstraint> WBMpcInterface::getStanceFootConstraint(const EndEffectorDynamics<scalar_t>& eeDynamics,
                                                                              size_t contactPointIndex) {
  const ModelSettings::FootConstraintConfig& footCfg = modelSettings_.footConstraintConfig;

  EndEffectorDynamicsAccelerationsConstraint::Config config;
  config.b.setZero(6);
  config.Ax.setZero(6, 6);
  config.Av.setIdentity(6, 6);
  config.Aa.setIdentity(6, 6);
  if (!numerics::almost_eq(footCfg.positionErrorGain_z, 0.0)) {
    config.Ax(2, 2) = footCfg.positionErrorGain_z;
  }
  if (!numerics::almost_eq(footCfg.orientationErrorGain, 0.0)) {
    config.Ax.block(3, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * footCfg.orientationErrorGain;
  }
  config.Av.block(0, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2) * footCfg.linearVelocityErrorGain_xy;
  config.Av(2, 2) = footCfg.linearVelocityErrorGain_z;
  config.Av.block(3, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * footCfg.angularVelocityErrorGain;
  config.Aa.block(0, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2) * footCfg.linearAccelerationErrorGain_xy;
  config.Aa(2, 2) = footCfg.linearAccelerationErrorGain_z;
  config.Aa.block(3, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * footCfg.angularAccelerationErrorGain;

  return std::unique_ptr<StateInputConstraint>(
      new ZeroAccelerationConstraintCppAd(*referenceManagerPtr_, eeDynamics, contactPointIndex, config));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> WBMpcInterface::getJointMimicConstraint(size_t mimicIndex) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);
  std::string prefix;
  if (mimicIndex == 0) {
    prefix = "mimicJoints.left_knee.";
  } else if (mimicIndex == 1) {
    prefix = "mimicJoints.right_knee.";
  } else {
    throw std::runtime_error("No mimic joint for index: " + std::to_string(mimicIndex));
  }

  std::string parentJointName;
  std::string childJointName;
  scalar_t multiplier;  // q_child = multiplier* q_parent
  scalar_t positionGain;
  scalar_t velocityGain;

  if (verbose_) {
    std::cerr << "\n #### Joint Mimic Kinematic Constraint Config: ";
    std::cerr << "\n #### "
                 "============================================================="
                 "================\n";
  }
  loadData::loadPtreeValue(pt, parentJointName, prefix + "parentJointName", verbose_);
  loadData::loadPtreeValue(pt, childJointName, prefix + "childJointName", verbose_);
  loadData::loadPtreeValue(pt, multiplier, prefix + "multiplier", verbose_);
  loadData::loadPtreeValue(pt, positionGain, prefix + "positionGain", verbose_);
  loadData::loadPtreeValue(pt, velocityGain, prefix + "velocityGain", verbose_);
  if (verbose_) {
    std::cerr << " #### "
                 "============================================================="
                 "================\n";
  }

  JointMimicDynamicsConstraint::Config config(*mpcRobotModelPtr_, parentJointName, childJointName, multiplier, positionGain, velocityGain);

  return std::unique_ptr<StateInputConstraint>(new JointMimicDynamicsConstraint(*mpcRobotModelPtr_, config));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> WBMpcInterface::getNormalVelocityConstraint(const EndEffectorDynamics<scalar_t>& eeDynamics,
                                                                                  size_t contactPointIndex) {
  return std::unique_ptr<StateInputConstraint>(new SwingLegVerticalConstraintCppAd(*referenceManagerPtr_, eeDynamics, contactPointIndex));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputCost> WBMpcInterface::getJointTorqueCost(const std::string& taskFile) {
  vector_t jointTorqueWeights(mpcRobotModelPtr_->getJointDim());
  loadData::loadEigenMatrix(taskFile, "joint_torque_weights", jointTorqueWeights);
  return std::unique_ptr<StateInputCost>(
      new JointTorqueCostCppAd(jointTorqueWeights, *pinocchioInterfacePtr_, *mpcRobotModelADPtr_, "jointTorqueCost", modelSettings_));
}

}  // namespace ocs2::humanoid
