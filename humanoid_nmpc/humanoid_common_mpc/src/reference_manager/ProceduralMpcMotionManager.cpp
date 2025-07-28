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

#include "humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h"

#include <ocs2_core/misc/LoadData.h>

#include <cmath>
#include "humanoid_common_mpc/gait/GaitScheduleUpdater.h"
#include "humanoid_common_mpc/gait/ModeSequenceTemplate.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ProceduralMpcMotionManager::ProceduralMpcMotionManager(const std::string& gaitFile,
                                                       const std::string& referenceFile,
                                                       std::shared_ptr<SwitchedModelReferenceManager> switchedModelReferenceManagerPtr,
                                                       const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                                       VelocityTargetToTargetTrajectories velocityTargetToTargetTrajectories)
    : velocityTargetToTargetTrajectoriesFun_(std::move(velocityTargetToTargetTrajectories)),
      switchedModelReferenceManagerPtr_(switchedModelReferenceManagerPtr),
      gaitSchedulePtr_(switchedModelReferenceManagerPtr_->getGaitSchedule()),
      mpcRobotModelPtr_(&mpcRobotModel),
      velocityCommandFilter(5, vector4_t::Zero()) {
  loadData::loadCppDataType(referenceFile, "maxDisplacementVelocityX", maxDisplacementVelocityX_);
  loadData::loadCppDataType(referenceFile, "maxDisplacementVelocityY", maxDisplacementVelocityY_);
  loadData::loadCppDataType(referenceFile, "maxDeltaPelvisHeight", maxDeltaPelvisHeight_);
  loadData::loadCppDataType(referenceFile, "maxRotationVelocity", maxRotationVelocity_);

  gaitMap_ = getGaitMap(gaitFile);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void ProceduralMpcMotionManager::setAndScaleVelocityCommand(const WalkingVelocityCommand& rawVelocityCommand) {
  velocityCommand_ = scaleWalkingVelocityCommand(rawVelocityCommand);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

WalkingVelocityCommand ProceduralMpcMotionManager::scaleWalkingVelocityCommand(const WalkingVelocityCommand& rawVelocityCommand) const {
  WalkingVelocityCommand scaledCommand = rawVelocityCommand;
  scaledCommand.linear_velocity_x *= maxDisplacementVelocityX_;
  scaledCommand.linear_velocity_y *= maxDisplacementVelocityY_;
  scaledCommand.angular_velocity_z *= maxRotationVelocity_;
  return scaledCommand;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

bool ProceduralMpcMotionManager::transitionToFasterGait(const vector4_t& velCommandVec,
                                                        const vector6_t& baseVelocity,
                                                        const GaitModeStateConfig& cfg) {
  bool fasterGaitRequested = (std::abs(velCommandVec(0)) > cfg.maxLinVelCmd || std::abs(velCommandVec(1)) > cfg.maxLinVelCmd ||
                              std::abs(velCommandVec(3)) > cfg.maxAngVelCmd);

  bool withinMaxSpeedErrorThreshold = (std::abs(baseVelocity(0)) > cfg.maxLinVelCmd - cfg.linVelErrorThresh ||
                                       std::abs(baseVelocity(1)) > cfg.maxLinVelCmd - cfg.linVelErrorThresh ||
                                       std::abs(baseVelocity(3)) > cfg.maxAngVelCmd - cfg.angVelErrorThresh);
  return fasterGaitRequested && withinMaxSpeedErrorThreshold;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

bool ProceduralMpcMotionManager::transitionToSlowerGait(const vector4_t& velCommandVec,
                                                        const vector6_t& baseVelocity,
                                                        const GaitModeStateConfig& cfg) {
  bool slowerGaitRequested = (std::abs(velCommandVec(0)) < cfg.minLinVelCmd && std::abs(velCommandVec(1)) < cfg.minLinVelCmd &&
                              std::abs(velCommandVec(3)) < cfg.minAngVelCmd);

  bool baseSpeedSlowEnough = (std::abs(baseVelocity(0)) < cfg.minLinVelCmd + cfg.linVelErrorThresh &&
                              std::abs(baseVelocity(1)) < cfg.minLinVelCmd + cfg.linVelErrorThresh &&
                              std::abs(velCommandVec(3)) < cfg.minAngVelCmd + cfg.angVelErrorThresh);

  return slowerGaitRequested && baseSpeedSlowEnough;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void ProceduralMpcMotionManager::preSolverRun(scalar_t initTime,
                                              scalar_t finalTime,
                                              const vector_t& initState,
                                              const ReferenceManagerInterface& referenceManager) {
  WalkingVelocityCommand incommingVelCommand = getScaledWalkingVelocityCommand();
  vector4_t filteredVelCommand = velocityCommandFilter.getFilteredVector(incommingVelCommand.toVector());

  // Update TargetTrajectories
  TargetTrajectories targetTrajectories = velocityTargetToTargetTrajectoriesFun_(filteredVelCommand, initTime, finalTime, initState);
  switchedModelReferenceManagerPtr_->setTargetTrajectories(targetTrajectories);

  static GaitModeStateConfig currentCfg = gaitModeStates_[currentGaitMode_];
  vector6_t baseVelocity = mpcRobotModelPtr_->getBaseComVelocity(initState);

  // Do not change the gait pattern for at least 0.5s
  if (initTime > lastGaitChangeTime_ + 0.2) {
    if (transitionToFasterGait(filteredVelCommand, baseVelocity, currentCfg)) {
      std::cout << "filteredVelCommand: " << filteredVelCommand.transpose() << std::endl;
      std::cout << "Linear limits: " << currentCfg.minLinVelCmd << ", " << currentCfg.maxLinVelCmd << std::endl;
      currentGaitMode_++;
      currentCfg = gaitModeStates_[currentGaitMode_];
      currentGaitCommand_ = currentCfg.gaitCommand;
      std::cout << "ProceduralMpcMotionManager: Increasing to gait:" << currentCfg.gaitCommand << std::endl;
      lastGaitChangeTime_ = initTime;
    } else if (transitionToSlowerGait(filteredVelCommand, baseVelocity, currentCfg)) {
      std::cout << "filteredVelCommand: " << filteredVelCommand.transpose() << std::endl;
      std::cout << "Linear limits: " << currentCfg.minLinVelCmd << ", " << currentCfg.maxLinVelCmd << std::endl;
      currentGaitMode_--;
      currentCfg = gaitModeStates_[currentGaitMode_];
      currentGaitCommand_ = currentCfg.gaitCommand;
      std::cout << "ProceduralMpcMotionManager: Decreasing to gait:" << currentCfg.gaitCommand << std::endl;
      lastGaitChangeTime_ = initTime;
    }
  }

  if (currentGaitCommand_ != lastGaitCommand_) {
    ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(currentGaitCommand_);

    GaitScheduleUpdater::updateGaitSchedule(gaitSchedulePtr_, modeSequenceTemplate, initTime, finalTime);
    lastGaitCommand_ = currentGaitCommand_;
  }
}

}  // namespace ocs2::humanoid
