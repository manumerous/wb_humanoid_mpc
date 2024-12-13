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
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include "ocs2_centroidal_model/CentroidalModelInfo.h"

#include <humanoid_common_mpc/command/TargetTrajectoriesCalculatorBase.h>
#include <humanoid_common_mpc/common/ModelSettings.h>
#include <humanoid_common_mpc/common/MpcRobotModelBase.h>
#include <humanoid_common_mpc/common/Types.h>

namespace ocs2::humanoid {

class CentroidalMpcTargetTrajectoriesCalculator : public TargetTrajectoriesCalculatorBase {
 public:
  CentroidalMpcTargetTrajectoriesCalculator(const std::string& referenceFile,
                                            const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                            PinocchioInterface pinocchioInterface,
                                            const CentroidalModelInfo& info,
                                            scalar_t mpcHorizon);

  CentroidalMpcTargetTrajectoriesCalculator(const CentroidalMpcTargetTrajectoriesCalculator& rhs) = delete;

  /**
   * Converts command line to TargetTrajectories.
   * @param [in] commadLineTarget : [deltaX, deltaY, deltaZ, deltaYaw] defined in pelvis frame
   * @param [in] observation : the current observation
   */
  TargetTrajectories commandedPositionToTargetTrajectories(const vector4_t& commadLineTarget,
                                                           scalar_t initTime,
                                                           const vector_t& initState) override;

  /**
   * Converts desired velocities to TargetTrajectories.
   * @param [in] commandedVelocities : [v_x, v_y, v_yaw] defined in pelvis frame
   * @param [in] observation : the current observation
   */
  TargetTrajectories commandedVelocityToTargetTrajectories(const vector4_t& commandedVelocities,
                                                           scalar_t initTime,
                                                           const vector_t& initState) override;

 private:
  PinocchioInterface pinocchioInterface_;
  const CentroidalModelInfo& info_;
  const scalar_t mass_;
};

}  // namespace ocs2::humanoid