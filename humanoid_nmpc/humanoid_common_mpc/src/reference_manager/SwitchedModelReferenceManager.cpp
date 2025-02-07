/******************************************************************************
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

#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                             const PinocchioInterface& pinocchioInterface,
                                                             const MpcRobotModelBase<scalar_t>& mpcRobotModel)
    : ReferenceManager(TargetTrajectories(), ModeSchedule()),
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)),
      // The reference manager gets a copy of the pinocchio model to use for initializing the ground height
      pinocchioInterface_(pinocchioInterface),
      mpcRobotModelPtr_(&mpcRobotModel) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const {
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

scalar_t SwitchedModelReferenceManager::getPhaseVariable(scalar_t time) const {
  const auto it = std::upper_bound(modeSchedule_.eventTimes.begin(), modeSchedule_.eventTimes.end(), time);
  scalar_t nextEventTime = *it;
  scalar_t prevEventTime = *(it - 1);

  if (modeSchedule_.modeAtTime(time) == LF) {
    return (0.5 * (time - prevEventTime) / (nextEventTime - prevEventTime));
  } else if (modeSchedule_.modeAtTime(time) == RF) {
    return (0.5 + 0.5 * (time - prevEventTime) / (nextEventTime - prevEventTime));
  } else {
    if (modeSchedule_.modeAtTime(prevEventTime - 0.01) == LF) {
      return 0.5;
    } else {
      return 0;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwitchedModelReferenceManager::adaptToCurrentGroundHeight(TargetTrajectories& targetTrajectories,
                                                                   const vector_t& initState,
                                                                   size_t initMode) {
  scalar_t terrainHeight = computeGroundHeightEstimate(pinocchioInterface_, *mpcRobotModelPtr_,
                                                       mpcRobotModelPtr_->getGeneralizedCoordinates(initState), initMode);

  terrainHeight = 0.0;

  // adapt target Trajectories to current terrain height
  // Since they are published in the past the current observations ground height might have drifted.

  // Adapt the ground height difference for every state in the target Trajectories.
  // The height difference between last update and the current update is applied here
  // to prevent applying the same difference twice in case the trajectories have not been updated.
  for (size_t i = 0; i < targetTrajectories.stateTrajectory.size(); i++) {
    vector_t& targetState = targetTrajectories.stateTrajectory[i];
    scalar_t heightDifference = terrainHeight - previousGroundHeightEstimate_;
    mpcRobotModelPtr_->adaptBasePoseHeight(targetState, heightDifference);
  }
  previousGroundHeightEstimate_ = terrainHeight;
  return terrainHeight;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector_t SwitchedModelReferenceManager::getDesiredState(const TargetTrajectories& targetTrajectories,
                                                        const vector_t& state,
                                                        scalar_t time) const {
  vector_t xNominal = targetTrajectories.getDesiredState(time);

  if (armSwingReferenceActive_) {
    scalar_t phaseVariable = this->getPhaseVariable(time);
    vector_t desiredJointAngles = mpcRobotModelPtr_->getJointAngles(xNominal);

    vector3_t linVelCommand = mpcRobotModelPtr_->getBaseComLinearVelocity(xNominal);
    scalar_t currentEulerZ = mpcRobotModelPtr_->getBasePose(state)[3];

    const scalar_t localVelXCommand = (std::cos(currentEulerZ) * linVelCommand[0] + std::sin(currentEulerZ) * linVelCommand[1]);

    const ModelSettings& modelSettings = mpcRobotModelPtr_->modelSettings;

    scalar_t gaitCycleFactor = std::sin(2 * M_PI * (phaseVariable - 0.15)) * localVelXCommand;
    desiredJointAngles[modelSettings.j_l_shoulder_y_index] += -0.15 * gaitCycleFactor;
    desiredJointAngles[modelSettings.j_r_shoulder_y_index] += 0.15 * gaitCycleFactor;
    desiredJointAngles[modelSettings.j_l_elbow_y_index] += -0.15 * gaitCycleFactor;
    desiredJointAngles[modelSettings.j_r_elbow_y_index] += 0.15 * gaitCycleFactor;

    mpcRobotModelPtr_->setJointAngles(xNominal, desiredJointAngles);
  }
  return xNominal;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime,
                                                     scalar_t finalTime,
                                                     const vector_t& initState,
                                                     size_t initMode,
                                                     TargetTrajectories& targetTrajectories,
                                                     ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  scalar_t terrainHeight = adaptToCurrentGroundHeight(targetTrajectories, initState, initMode);

  swingTrajectoryPtr_->update(modeSchedule, terrainHeight);

  modeSchedule_ = modeSchedule;
}

}  // namespace ocs2::humanoid
