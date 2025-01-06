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

#pragma once

#include <functional>

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>

#include <humanoid_common_mpc/common/ModelSettings.h>
#include <humanoid_common_mpc/common/Types.h>
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"

namespace ocs2::humanoid {

class TargetTrajectoriesCalculatorBase {
 public:
  TargetTrajectoriesCalculatorBase(const std::string& referenceFile,

                                   const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                   scalar_t mpcHorizon);

  TargetTrajectoriesCalculatorBase(const TargetTrajectoriesCalculatorBase& rhs) = delete;

  void setTargetDisplacementVelocity(scalar_t targetDisplacementVelocity) { targetDisplacementVelocity_ = targetDisplacementVelocity; }
  void setTargetRotationVelocity_(scalar_t targetRotationVelocity) { targetRotationVelocity = targetRotationVelocity; }
  void setTargetJointState(const vector_t targetJointState);

  /**
   * Converts command line to TargetTrajectories.
   * @param [in] commadLineTarget : [deltaX, deltaY, deltaZ, deltaYaw] defined in pelvis frame
   * @param [in] observation : the current observation
   */
  virtual TargetTrajectories commandedPositionToTargetTrajectories(const vector4_t& commandedVelocities,
                                                                   scalar_t initTime,
                                                                   const vector_t& initState) = 0;

  /**
   * Converts desired velocities to TargetTrajectories.
   * @param [in] commandedVelocities : [v_x, v_y, v_yaw] defined in pelvis frame
   * @param [in] observation : the current observation
   */
  virtual TargetTrajectories commandedVelocityToTargetTrajectories(const vector4_t& commandedVelocities,
                                                                   scalar_t initTime,
                                                                   const vector_t& initState) = 0;

 protected:
  scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) const;

  virtual vector6_t getCurrentBasePoseTarget(const vector_t& state) const;

  vector6_t getDeltaBaseTarget(const vector4_t& commadLinePoseTarget, const vector6_t& currentPoseTarget) const;

  vector4_t filterAndTransformVelCommandToLocal(const vector4_t& commandedVelLocal,
                                                const scalar_t& currentEulerZ,
                                                scalar_t filterAlpha) const;

  vector6_t integrateTargetBasePose(const vector6_t& currentPose,
                                    const vector3_t& averageVel,
                                    scalar_t deltaPelvisHeight,
                                    scalar_t deltaT) const;

  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;

  // For pose control mode
  scalar_t targetDisplacementVelocity_;
  scalar_t targetRotationVelocity_;

  // For velocity control mode
  scalar_t maxDisplacementVelocityX_ = 0.6;
  scalar_t maxDisplacementVelocityY_ = 0.3;
  scalar_t maxDeltaPelvisHeight_ = 0.3;
  scalar_t maxRotationVelocity_ = 0.6;

  scalar_t defaultBaseHeight_;
  vector_t targetJointState_;
  scalar_t mpcHorizon_;
};

}  // namespace ocs2::humanoid