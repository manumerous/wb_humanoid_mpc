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

#include "humanoid_common_mpc/command/TargetTrajectoriesCalculatorBase.h"

#include <algorithm>  // For std::clamp

#include <ocs2_core/misc/LoadData.h>

#include <cmath>
#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"

namespace ocs2::humanoid {

TargetTrajectoriesCalculatorBase::TargetTrajectoriesCalculatorBase(const std::string& referenceFile,
                                                                   const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                                                   scalar_t mpcHorizon)
    : mpcRobotModelPtr_(mpcRobotModel.clone()), mpcHorizon_(mpcHorizon) {
  std::cerr << "Loading reference file: " << referenceFile << std::endl;
  targetJointState_.resize(mpcRobotModel.getJointDim());
  loadData::loadCppDataType(referenceFile, "defaultBaseHeight", defaultBaseHeight_);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", targetJointState_);
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity", targetRotationVelocity_);
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", targetDisplacementVelocity_);
  loadData::loadCppDataType(referenceFile, "maxDisplacementVelocityX", maxDisplacementVelocityX_);
  loadData::loadCppDataType(referenceFile, "maxDisplacementVelocityY", maxDisplacementVelocityY_);
  loadData::loadCppDataType(referenceFile, "maxDeltaPelvisHeight", maxDeltaPelvisHeight_);
  loadData::loadCppDataType(referenceFile, "maxRotationVelocity", maxRotationVelocity_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void TargetTrajectoriesCalculatorBase::setTargetJointState(const vector_t targetJointState) {
  assert(targetJointState.size() == mpcRobotModelPtr_->getJointDim());
  targetJointState_ = targetJointState;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector6_t TargetTrajectoriesCalculatorBase::getDeltaBaseTarget(const vector4_t& commadLinePoseTarget,
                                                               const vector6_t& currentPoseTarget) const {
  vector_t target(6);

  // X facing forward to the robot, Y to the left side in the baseFrame
  const scalar_t baseFrameDeltaX = commadLinePoseTarget(0);
  const scalar_t baseFrameDeltaY = commadLinePoseTarget(1);
  const scalar_t currentEulerZ = currentPoseTarget(3);

  const scalar_t globalFrameDeltaX = std::cos(currentEulerZ) * baseFrameDeltaX - std::sin(currentEulerZ) * baseFrameDeltaY;
  const scalar_t globalFrameDeltaY = std::sin(currentEulerZ) * baseFrameDeltaX + std::cos(currentEulerZ) * baseFrameDeltaY;

  // base p_x, p_y are relative to current state
  target(0) = currentPoseTarget(0) + globalFrameDeltaX;
  target(1) = currentPoseTarget(1) + globalFrameDeltaY;
  // base z relative to the default height
  scalar_t deltaPelvisHeight = std::clamp(commadLinePoseTarget(2), -maxDeltaPelvisHeight_, maxDeltaPelvisHeight_);
  target(2) = defaultBaseHeight_ + deltaPelvisHeight;
  // theta_z relative to current
  target(3) = currentPoseTarget(3) + commadLinePoseTarget(3) * M_PI / 180.0;
  target(4) = 0.0;
  target(5) = 0.0;

  return target;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector6_t TargetTrajectoriesCalculatorBase::getCurrentBasePoseTarget(const vector_t& state) const {
  vector_t currentPoseTarget = mpcRobotModelPtr_->getBasePose(state);
  // Zero out roll and pitch of the torso since target trajectories starts from current state
  currentPoseTarget(4) = 0.0;
  currentPoseTarget(5) = 0.0;

  return currentPoseTarget;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector4_t TargetTrajectoriesCalculatorBase::filterAndTransformVelCommandToLocal(const vector4_t& commandedVelLocal,
                                                                                const scalar_t& currentEulerZ,
                                                                                scalar_t filterAlpha) const {
  static vector4_t commVelFiltered = vector4_t::Zero();

  commVelFiltered = commVelFiltered * filterAlpha + commandedVelLocal * (1 - filterAlpha);

  vector4_t globalTargetVel = commVelFiltered;

  globalTargetVel(0) = std::cos(currentEulerZ) * commVelFiltered[0] - std::sin(currentEulerZ) * commVelFiltered[1];
  globalTargetVel(1) = std::sin(currentEulerZ) * commVelFiltered[0] + std::cos(currentEulerZ) * commVelFiltered[1];

  return globalTargetVel;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector6_t TargetTrajectoriesCalculatorBase::integrateTargetBasePose(const vector6_t& currentPose,
                                                                    const vector3_t& averageVel,
                                                                    scalar_t deltaPelvisHeight,
                                                                    scalar_t deltaT) const {
  vector6_t targetPose = currentPose;

  targetPose[0] += averageVel[0] * deltaT;
  targetPose[1] += averageVel[1] * deltaT;
  targetPose[2] = deltaPelvisHeight;
  targetPose[3] += averageVel[2] * deltaT;
  targetPose[4] = 0.0;
  targetPose[5] = 0.0;
  return targetPose;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

scalar_t TargetTrajectoriesCalculatorBase::estimateTimeToTarget(const vector_t& desiredBaseDisplacement) const {
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t rotationTime = std::abs(dyaw) / targetRotationVelocity_;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / targetDisplacementVelocity_;
  return std::max(rotationTime, displacementTime);
}

}  // namespace ocs2::humanoid
