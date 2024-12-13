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

#include "humanoid_common_mpc/cost/StateInputQuadraticCost.h"

#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"

#include <cmath>
#include <numbers>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateInputQuadraticCost::StateInputQuadraticCost(matrix_t Q,
                                                 matrix_t R,
                                                 const SwitchedModelReferenceManager& referenceManager,
                                                 const PinocchioInterface& pinocchioInterface,
                                                 const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                                 bool armSwingReferenceActivated,
                                                 scalar_t maxDisplacementVelocityX)
    : QuadraticStateInputCost(std::move(Q), std::move(R)),
      referenceManagerPtr_(&referenceManager),
      pinInterface_(pinocchioInterface),
      mpcRobotModelPtr_(&mpcRobotModel),
      // j_l_shoulder_y_index_(mpcRobotModel.getJointIndex("j_l_shoulder_y")),
      // j_r_shoulder_y_index_(mpcRobotModel.getJointIndex("j_r_shoulder_y")),
      // j_l_elbow_y_index_(mpcRobotModel.getJointIndex("j_l_elbow_y")),
      // j_r_elbow_y_index_(mpcRobotModel.getJointIndex("j_r_elbow_y")),
      // j_spine_y_index_(mpcRobotModel.getJointIndex("j_spine_y")),
      // j_spine_z_index_(mpcRobotModel.getJointIndex("j_spine_z")),
      armSwingReferenceActivated_(armSwingReferenceActivated),
      maxDisplacementVelocityX_(maxDisplacementVelocityX) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

StateInputQuadraticCost::StateInputQuadraticCost(const StateInputQuadraticCost& rhs)
    : QuadraticStateInputCost(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      pinInterface_(rhs.pinInterface_),
      mpcRobotModelPtr_(rhs.mpcRobotModelPtr_),
      // j_l_shoulder_y_index_(rhs.j_l_shoulder_y_index_),
      // j_r_shoulder_y_index_(rhs.j_r_shoulder_y_index_),
      // j_l_elbow_y_index_(rhs.j_l_elbow_y_index_),
      // j_r_elbow_y_index_(rhs.j_r_elbow_y_index_),
      // j_spine_y_index_(rhs.j_spine_y_index_),
      // j_spine_z_index_(rhs.j_spine_z_index_),
      armSwingReferenceActivated_(rhs.armSwingReferenceActivated_),
      maxDisplacementVelocityX_(rhs.maxDisplacementVelocityX_) {
  std::cout << "Loaded max displacement velocity of: " << maxDisplacementVelocityX_ << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::pair<vector_t, vector_t> StateInputQuadraticCost::getStateInputDeviation(scalar_t time,
                                                                              const vector_t& state,
                                                                              const vector_t& input,
                                                                              const TargetTrajectories& targetTrajectories) const {
  const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
  vector_t xNominal = targetTrajectories.getDesiredState(time);

  // All reference stuff should eventually be moved out of here.
  const vector_t uNominal = weightCompensatingInput(pinInterface_, contactFlags, *mpcRobotModelPtr_);

  // if (armSwingReferenceActivated_) {
  //   scalar_t phaseVariable = referenceManagerPtr_->getPhaseVariable(time);
  //   vector_t desiredJointAngles = mpcRobotModelPtr_->getJointAngles(xNominal);

  //   vector3_t linVelCommand = mpcRobotModelPtr_->getBaseComLinearVelocity(xNominal);
  //   scalar_t currentEulerZ = mpcRobotModelPtr_->getBasePose(state)[3];

  //   const scalar_t localVelXCommand = (std::cos(currentEulerZ) * linVelCommand[0] + std::sin(currentEulerZ) * linVelCommand[1]);

  //   scalar_t gaitCycleFactor = std::sin(2 * M_PI * (phaseVariable - 0.15)) * localVelXCommand;
  //   desiredJointAngles[j_l_shoulder_y_index_] += -0.3 * gaitCycleFactor;
  //   desiredJointAngles[j_r_shoulder_y_index_] += 0.3 * gaitCycleFactor;
  //   desiredJointAngles[j_l_elbow_y_index_] += -0.3 * gaitCycleFactor;
  //   desiredJointAngles[j_r_elbow_y_index_] += 0.3 * gaitCycleFactor;
  //   desiredJointAngles[j_spine_y_index_] += 0.1 * std::max(localVelXCommand, 1.0);
  //   desiredJointAngles[j_spine_z_index_] += 0.1 * gaitCycleFactor;

  //   mpcRobotModelPtr_->setJointAngles(xNominal, desiredJointAngles);
  // }

  return {state - xNominal, input - uNominal};
}

}  // namespace ocs2::humanoid
