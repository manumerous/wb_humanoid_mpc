/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.

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

#include "humanoid_centroidal_mpc/mrt/CentroidalMpcMrtJointController.h"

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include "ocs2_centroidal_model/ModelHelperFunctions.h"

#include "ocs2_centroidal_model/AccessHelperFunctions.h"

#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>
#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>
#include <humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h>
#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"

namespace ocs2::humanoid {

CentroidalMpcMrtJointController::CentroidalMpcMrtJointController(const ::robot::model::RobotDescription& robotDescription,
                                                                 const ModelSettings& modelSettings,
                                                                 const CentroidalMpcRobotModel<scalar_t>& mpcRobotModel,
                                                                 MPC_BASE& mpc,
                                                                 PinocchioInterface pinocchioInterface,
                                                                 scalar_t mpcDesiredFrequency,
                                                                 std::shared_ptr<DummyObserver> rVizVisualizerPtr)
    : mcpMrtInterface_(mpc),
      pinocchioInterface_(pinocchioInterface),
      mpcRobotModelPtr_(mpcRobotModel.clone()),
      mpcDeltaTMicroSeconds_(1000000 / mpcDesiredFrequency),
      realtime_(mpcDesiredFrequency <= 0),
      visualizerPtr_(rVizVisualizerPtr),
      inverse_dynamics_kp_(mpcRobotModel.getJointDim()),
      inverse_dynamics_kd_(mpcRobotModel.getJointDim()) {
  mpcJointIndices_ = robotDescription.getJointIndices(modelSettings.mpcModelJointNames);
  otherJointIndices_ = robotDescription.getJointIndices(modelSettings.fixedJointNames);
  currentMpcObservation_.state = vector_t::Zero(mpcRobotModelPtr_->getStateDim());
  currentMpcObservation_.input = vector_t::Zero(mpcRobotModelPtr_->getInputDim());

  // Currently set to 0. There is still a bug in the momentum computation of the inverse dynamics.
  inverse_dynamics_kp_.fill(0.0);
  inverse_dynamics_kd_.fill(0.0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

CentroidalMpcMrtJointController::~CentroidalMpcMrtJointController() {
  // Signal the solver thread to terminate
  terminateThread_.store(true);

  // Wait for the solver thread to finish if it's joinable
  if (solver_worker_.joinable()) {
    solver_worker_.join();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void CentroidalMpcMrtJointController::startMpcThread(const ::robot::model::RobotState& initRobotState) {
  updateMpcObservation(currentMpcObservation_, initRobotState);
  // Set observation to MPC
  mcpMrtInterface_.setCurrentObservation(currentMpcObservation_);
  solver_worker_ = std::jthread(&CentroidalMpcMrtJointController::solverWorker, this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void CentroidalMpcMrtJointController::updateMpcState(vector_t& mpcState, const ::robot::model::RobotState& robotState) {
  const auto& info = mpcRobotModelPtr_->getCentroidalModelInfo();

  const vector3_t euler_zyx = quaternionToEulerZYX(robotState.getRootRotationLocalToWorldFrame());

  vector_t qPinocchio(info.generalizedCoordinatesNum);
  qPinocchio.head<3>() = robotState.getRootPositionInWorldFrame();
  qPinocchio.segment<3>(3) = euler_zyx;
  qPinocchio.tail(mpcRobotModelPtr_->getJointDim()) = robotState.getJointPositions(mpcJointIndices_);

  vector_t vPinocchio(info.generalizedCoordinatesNum);
  vPinocchio.head<3>() = robotState.getRootRotationLocalToWorldFrame() * robotState.getRootLinearVelocityInLocalFrame();
  vPinocchio.segment<3>(3) =
      getEulerAnglesZyxDerivativesFromLocalAngularVelocity<scalar_t>(euler_zyx, robotState.getRootAngularVelocityInLocalFrame());
  vPinocchio.tail(mpcRobotModelPtr_->getJointDim()) = robotState.getJointVelocities(mpcJointIndices_);

  updateCentroidalDynamics(pinocchioInterface_, info, qPinocchio);
  const auto& A = getCentroidalMomentumMatrix(pinocchioInterface_);

  centroidal_model::getNormalizedMomentum(mpcState, info).noalias() = A * vPinocchio / info.robotMass;
  centroidal_model::getGeneralizedCoordinates(mpcState, info) = qPinocchio;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void CentroidalMpcMrtJointController::updateMpcObservation(ocs2::SystemObservation& mpcObservation,
                                                           const ::robot::model::RobotState& robotState) {
  updateMpcState(mpcObservation.state, robotState);
  mpcObservation.time = robotState.getTime();
  mpcObservation.input = vector_t::Zero(mpcRobotModelPtr_->getInputDim());  // Add contact forces later.
  std::vector<bool> configContacts = robotState.getContactFlags();
  assert(configContacts.size() == 2);
  contact_flag_t contactFlags;
  std::copy(configContacts.begin(), configContacts.end(), contactFlags.begin());
  mpcObservation.mode = stanceLeg2ModeNumber(contactFlags);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void CentroidalMpcMrtJointController::computeJointControlAction(scalar_t time,
                                                                const ::robot::model::RobotState& robotState,
                                                                ::robot::model::RobotJointAction& robotJointAction) {
  updateMpcObservation(currentMpcObservation_, robotState);
  // Set observation to MPC
  mcpMrtInterface_.setCurrentObservation(currentMpcObservation_);

  vector_t mpcPolicyState;
  vector_t mpcPolicyInput;
  size_t mpcPolicyMode;

  if (mcpMrtInterface_.initialPolicyReceived()) {
    // Evaluate policy with feedback if activated in config
    mcpMrtInterface_.evaluatePolicy(currentMpcObservation_.time + 0.005, currentMpcObservation_.state, mpcPolicyState, mpcPolicyInput,
                                    mpcPolicyMode);

    // TODO something seems wrong with the inverse dynamics. You should correct that.
    vector_t mpc_q_j_des = mpcRobotModelPtr_->getJointAngles(mpcPolicyState);
    vector_t mpc_qd_j_des = mpcRobotModelPtr_->getJointVelocities(mpcPolicyState, mpcPolicyInput);
    vector_t q_j = mpcRobotModelPtr_->getJointAngles(currentMpcObservation_.state);
    vector_t qd_j = mpcRobotModelPtr_->getJointVelocities(currentMpcObservation_.state, currentMpcObservation_.input);
    vector_t qdd_j_des = inverse_dynamics_kp_ * (mpc_q_j_des - q_j) + inverse_dynamics_kd_ * (mpc_qd_j_des - qd_j);

    std::array<vector6_t, 2> footWrenches{mpcRobotModelPtr_->getContactWrench(mpcPolicyInput, 0),
                                          mpcRobotModelPtr_->getContactWrench(mpcPolicyInput, 1)};

    vector_t q = mpcRobotModelPtr_->getGeneralizedCoordinates(currentMpcObservation_.state);
    vector_t qd = mpcRobotModelPtr_->getGeneralizedVelocities(currentMpcObservation_.state, currentMpcObservation_.input);
    // std::cerr << qd.head(6).transpose() << std::endl;

    // This did not help.
    // qd.head(6) = vector6_t::Zero();

    vector_t mpcJointTorques = computeJointTorques<scalar_t>(q, qd, qdd_j_des, footWrenches, pinocchioInterface_);

    // std::cout << "mpcJointTorques: " << mpcJointTorques.transpose() << std::endl;

    for (size_t i = 0; i < mpcJointIndices_.size(); i++) {
      size_t index = mpcJointIndices_[i];
      robot::model::JointAction& action = robotJointAction.at(index).value();

      action.q_des = mpc_q_j_des[i];
      action.qd_des = mpc_qd_j_des[i];
      action.kp = 1200.0;
      action.kd = 10.0;
      action.feed_forward_effort = mpcJointTorques[i];

      // std::cerr << "MPCtorque!: " << mpcJointTorques[i] << std::endl;
    };

    if (visualizerPtr_ != nullptr) {
      visualizerPtr_->update(currentMpcObservation_, mcpMrtInterface_.getPolicy(), mcpMrtInterface_.getCommand());
    }
  }

  else {
    std::cerr << "Apply weight compensating torque..." << std::endl;
    //   Apply weight compensated input around current state
    vector_t qdd_j_des = vector_t::Zero(mpcRobotModelPtr_->getJointDim());
    mpcPolicyInput = weightCompensatingInput(pinocchioInterface_, {true, true}, *mpcRobotModelPtr_);
    std::array<vector6_t, 2> footWrenches{mpcRobotModelPtr_->getContactWrench(mpcPolicyInput, 0),
                                          mpcRobotModelPtr_->getContactWrench(mpcPolicyInput, 1)};
    vector_t weightCompensatingTorques = computeJointTorques<scalar_t>(
        mpcRobotModelPtr_->getGeneralizedCoordinates(currentMpcObservation_.state),
        mpcRobotModelPtr_->getGeneralizedVelocities(currentMpcObservation_.state, currentMpcObservation_.input), qdd_j_des, footWrenches,
        pinocchioInterface_);

    for (size_t i = 0; i < mpcJointIndices_.size(); i++) {
      size_t index = mpcJointIndices_[i];
      robot::model::JointAction& action = robotJointAction.at(index).value();

      action.q_des = 0;
      action.qd_des = 0;
      action.kp = 1200;
      action.kd = 10;
      action.feed_forward_effort = weightCompensatingTorques[i];
    };
  }

  for (size_t i = 0; i < otherJointIndices_.size(); i++) {
    size_t index = otherJointIndices_[i];
    robot::model::JointAction& action = robotJointAction.at(index).value();

    action.q_des = 0;
    action.qd_des = 0;
    action.kp = 100;
    action.kd = 1.0;
    action.feed_forward_effort = 0.0;
  };
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void CentroidalMpcMrtJointController::solverWorker() {
  // while (!isInitialized_.load()) {
  //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // }

  mcpMrtInterface_.resetMpcNode(currentObservationToResetTrajectory(mcpMrtInterface_.getCurrentObservation()));
  std::cerr << "MPC is reset. NMPC solver started!" << std::endl;

  while (true) {
    auto targetTimeForNextIteration = std::chrono::steady_clock::now() + std::chrono::microseconds(mpcDeltaTMicroSeconds_);

    mcpMrtInterface_.advanceMpc();

    // Publish if Policy has been updated
    if (!mcpMrtInterface_.updatePolicy()) {
      std::cerr << "The solver has failed to update!!" << std::endl;
      return;
    }

    // std::cerr << "MPC policy computed!" << std::endl;

    if (!realtime_) {
      auto currentTime = std::chrono::steady_clock::now();
      if (currentTime > targetTimeForNextIteration) {
        auto delay = std::chrono::duration_cast<std::chrono::microseconds>(currentTime - targetTimeForNextIteration).count();

        std::cerr << "Warning: MPC loop running slow by " << delay << " microseconds." << std::endl;
      } else {
        // Sleep in case sim loop is faster than specified
        std::this_thread::sleep_until(targetTimeForNextIteration);
      }
    }
  }
  std::cerr << "Shutting down NMPC" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectories CentroidalMpcMrtJointController::currentObservationToResetTrajectory(const SystemObservation& currentObservation) {
  vector_t targetState = currentObservation.state;

  // zero out velocities
  targetState.tail(mpcRobotModelPtr_->getGenCoordinatesDim()) = vector_t::Zero(mpcRobotModelPtr_->getGenCoordinatesDim());

  // zero out pitch + roll angles
  targetState.segment<2>(4) = vector_t::Zero(2);

  const TargetTrajectories resetTargetTrajectories({currentObservation.time}, {targetState},
                                                   {vector_t::Zero(currentObservation.input.size())});

  std::cerr << "Resetting MPC to current state: \n" << targetState << std::endl;
  return resetTargetTrajectories;
}

}  // namespace ocs2::humanoid
