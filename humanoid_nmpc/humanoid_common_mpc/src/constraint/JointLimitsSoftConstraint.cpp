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

#include "humanoid_common_mpc/constraint/JointLimitsSoftConstraint.h"

#include "humanoid_common_mpc/common/ModelSettings.h"

#include <iostream>

namespace ocs2::humanoid {

JointLimitsSoftConstraint::JointLimitsSoftConstraint(std::pair<vector_t, vector_t> positionlimits,
                                                     ocs2::PieceWisePolynomialBarrierPenalty::Config barrierSettings,
                                                     const MpcRobotModelBase<scalar_t>& mpcRobotModel)
    : jointPositionPenaltyPtr_(new ocs2::PieceWisePolynomialBarrierPenalty(barrierSettings)),
      positionLimits_(positionlimits),
      mpcRobotModelPtr_(&mpcRobotModel),
      offset_(0.0) {
  // Obtain the offset at the middle joint angles. Just to compensate high negative costs when being far away from an infinite joint limit
  // offset_ = -getValue(0.5 * (positionLimits_.first + positionLimits_.second));
  std::cout << "joint limit offset: " << offset_ << std::endl;
}

JointLimitsSoftConstraint::JointLimitsSoftConstraint(const JointLimitsSoftConstraint& rhs)
    : jointPositionPenaltyPtr_(rhs.jointPositionPenaltyPtr_->clone()),
      positionLimits_(rhs.positionLimits_),
      mpcRobotModelPtr_(rhs.mpcRobotModelPtr_),
      offset_(rhs.offset_) {}

scalar_t JointLimitsSoftConstraint::getValue(scalar_t time,
                                             const vector_t& state,
                                             const ocs2::TargetTrajectories& targetTrajectories,
                                             const ocs2::PreComputation& preComp) const {
  return getValue(mpcRobotModelPtr_->getJointAngles(state));
}

ScalarFunctionQuadraticApproximation JointLimitsSoftConstraint::getQuadraticApproximation(
    scalar_t time, const vector_t& state, const ocs2::TargetTrajectories& targetTrajectories, const ocs2::PreComputation& preComp) const {
  return getQuadraticApproximation(mpcRobotModelPtr_->getJointAngles(state));
}

scalar_t JointLimitsSoftConstraint::getValue(const vector_t& jointPositions) const {
  const vector_t upperBoundPositionOffset = positionLimits_.second - jointPositions;
  const vector_t lowerBoundPositionOffset = jointPositions - positionLimits_.first;

  return upperBoundPositionOffset.unaryExpr([&](scalar_t hi) { return jointPositionPenaltyPtr_->getValue(0.0, hi); }).sum() +
         lowerBoundPositionOffset.unaryExpr([&](scalar_t hi) { return jointPositionPenaltyPtr_->getValue(0.0, hi); }).sum() + offset_;
}

ScalarFunctionQuadraticApproximation JointLimitsSoftConstraint::getQuadraticApproximation(const vector_t& jointPositions) const {
  const vector_t upperBoundPositionOffset = positionLimits_.second - jointPositions;
  const vector_t lowerBoundPositionOffset = jointPositions - positionLimits_.first;

  const size_t stateDim = mpcRobotModelPtr_->getStateDim();
  const size_t jointDim = mpcRobotModelPtr_->getJointDim();
  const size_t jointStartIndex = mpcRobotModelPtr_->getJointStartindex();

  ScalarFunctionQuadraticApproximation cost;
  cost.f = upperBoundPositionOffset.unaryExpr([&](scalar_t hi) { return jointPositionPenaltyPtr_->getValue(0.0, hi); }).sum() +
           lowerBoundPositionOffset.unaryExpr([&](scalar_t hi) { return jointPositionPenaltyPtr_->getValue(0.0, hi); }).sum() + offset_;

  cost.dfdx = vector_t::Zero(stateDim);
  cost.dfdx.segment(jointStartIndex, jointDim) = lowerBoundPositionOffset.unaryExpr([&](scalar_t hi) {
    return jointPositionPenaltyPtr_->getDerivative(0.0, hi);
  }) - upperBoundPositionOffset.unaryExpr([&](scalar_t hi) { return jointPositionPenaltyPtr_->getDerivative(0.0, hi); });

  cost.dfdxx = matrix_t::Zero(stateDim, stateDim);
  cost.dfdxx.block(jointStartIndex, jointStartIndex, jointDim, jointDim).diagonal() = lowerBoundPositionOffset.unaryExpr([&](scalar_t hi) {
    return jointPositionPenaltyPtr_->getSecondDerivative(0.0, hi);
  }) + upperBoundPositionOffset.unaryExpr([&](scalar_t hi) { return jointPositionPenaltyPtr_->getSecondDerivative(0.0, hi); });

  return cost;
}

void JointLimitsSoftConstraint::setGains(const scalar_t& mu, const scalar_t& delta) {
  jointPositionPenaltyPtr_->setConfig(ocs2::PieceWisePolynomialBarrierPenalty::Config(mu, delta));
}

void JointLimitsSoftConstraint::getGains(scalar_t& mu, scalar_t& delta) const {
  ocs2::PieceWisePolynomialBarrierPenalty::Config config;
  jointPositionPenaltyPtr_->getConfig(config);
  mu = config.mu;
  delta = config.delta;
}

}  // namespace ocs2::humanoid