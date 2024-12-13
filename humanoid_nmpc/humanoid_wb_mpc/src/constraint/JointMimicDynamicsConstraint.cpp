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

#include "humanoid_wb_mpc/constraint/JointMimicDynamicsConstraint.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

JointMimicDynamicsConstraint::Config::Config(const WBAccelMpcRobotModel<scalar_t>& mpcRobotModel,
                                             std::string parentJointNameParam,
                                             std::string childJointNameParam,
                                             scalar_t multiplierParam,
                                             scalar_t positionGainParam,
                                             scalar_t velocityGainParam)
    : parentJointName(parentJointNameParam),
      childJointName(childJointNameParam),
      parentJointIndex(mpcRobotModel.getJointIndex(parentJointNameParam)),
      childJointIndex(mpcRobotModel.getJointIndex(childJointNameParam)),
      multiplier(multiplierParam),
      positionGain(positionGainParam),
      velocityGain(velocityGainParam) {
  assert(positionGain > 0.0);
  assert(positionGain > velocityGainParam);
  assert(velocityGainParam > 0.0);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

JointMimicDynamicsConstraint::JointMimicDynamicsConstraint(const WBAccelMpcRobotModel<scalar_t>& wbAccelMpcRobotModel, Config config)
    : StateInputConstraint(ConstraintOrder::Linear), wbAccelMpcRobotModelPtr_(&wbAccelMpcRobotModel), config_(config) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

JointMimicDynamicsConstraint::JointMimicDynamicsConstraint(const JointMimicDynamicsConstraint& rhs)
    : StateInputConstraint(rhs), wbAccelMpcRobotModelPtr_(rhs.wbAccelMpcRobotModelPtr_), config_(rhs.config_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

bool JointMimicDynamicsConstraint::isActive(scalar_t time) const {
  return isActive_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t JointMimicDynamicsConstraint::getValue(scalar_t time,
                                                const vector_t& state,
                                                const vector_t& input,
                                                const PreComputation& preComp) const {
  vector_t jointAngles = wbAccelMpcRobotModelPtr_->getJointAngles(state);
  vector_t jointVelocities = wbAccelMpcRobotModelPtr_->getJointVelocities(state, input);
  vector_t jointAccelerations = wbAccelMpcRobotModelPtr_->getJointAccelerations(input);
  scalar_t posError = config_.multiplier * jointAngles[config_.parentJointIndex] - jointAngles[config_.childJointIndex];
  scalar_t velError = config_.multiplier * jointVelocities[config_.parentJointIndex] - jointVelocities[config_.childJointIndex];
  scalar_t accError = config_.multiplier * jointAccelerations[config_.parentJointIndex] - jointAccelerations[config_.childJointIndex];

  vector_t value(1);
  value << (config_.positionGain * posError + config_.velocityGain * velError + accError);

  return value;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation JointMimicDynamicsConstraint::getLinearApproximation(scalar_t time,
                                                                                       const vector_t& state,
                                                                                       const vector_t& input,
                                                                                       const PreComputation& preComp) const {
  VectorFunctionLinearApproximation linearApproximation =
      VectorFunctionLinearApproximation::Zero(getNumConstraints(time), state.size(), input.size());

  linearApproximation.f = getValue(time, state, input, preComp);

  linearApproximation.dfdx(0, wbAccelMpcRobotModelPtr_->getJointStartindex() + config_.parentJointIndex) =
      config_.positionGain * config_.multiplier;
  linearApproximation.dfdx(0, wbAccelMpcRobotModelPtr_->getJointStartindex() + config_.childJointIndex) = -config_.positionGain;

  linearApproximation.dfdx(0, wbAccelMpcRobotModelPtr_->getJointVelocitiesStartindex() + config_.parentJointIndex) =
      config_.velocityGain * config_.multiplier;
  linearApproximation.dfdx(0, wbAccelMpcRobotModelPtr_->getJointVelocitiesStartindex() + config_.childJointIndex) = -config_.velocityGain;

  linearApproximation.dfdu(0, wbAccelMpcRobotModelPtr_->getJointAccelerationsStartindex() + config_.parentJointIndex) = config_.multiplier;
  linearApproximation.dfdu(0, wbAccelMpcRobotModelPtr_->getJointAccelerationsStartindex() + config_.childJointIndex) = -1;

  return linearApproximation;
}

}  // namespace ocs2::humanoid
