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

#include <memory>

#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/penalties/penalties/PieceWisePolynomialBarrierPenalty.h>

#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

class JointLimitsSoftConstraint final : public StateCost {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   *
   * @param positionlimits : {lower bounds, upper bounds} joint position limits.
   * @param barrierSettings : Settings for the position barrier penalty function.
   */
  JointLimitsSoftConstraint(std::pair<vector_t, vector_t> positionlimits,
                            ocs2::PieceWisePolynomialBarrierPenalty::Config barrierSettings,
                            const MpcRobotModelBase<scalar_t>& mpcRobotModel);

  JointLimitsSoftConstraint* clone() const override { return new JointLimitsSoftConstraint(*this); }

  scalar_t getValue(scalar_t time,
                    const vector_t& state,
                    const ocs2::TargetTrajectories& targetTrajectories,
                    const ocs2::PreComputation& preComp) const override;

  ScalarFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time,
                                                                 const vector_t& state,
                                                                 const ocs2::TargetTrajectories& targetTrajectories,
                                                                 const ocs2::PreComputation& preComp) const override;

  scalar_t getValue(const vector_t& jointPositions) const;
  ScalarFunctionQuadraticApproximation getQuadraticApproximation(const vector_t& jointPositions) const;

  void setGains(const scalar_t& mu, const scalar_t& delta);
  void getGains(scalar_t& mu, scalar_t& delta) const;

  bool isActive(scalar_t time) const override { return isActive_; }
  void setActive(bool isActive) { isActive_ = isActive; }
  bool getActive() const { return isActive_; }

 private:
  JointLimitsSoftConstraint(const JointLimitsSoftConstraint& rhs);

  std::unique_ptr<PieceWisePolynomialBarrierPenalty> jointPositionPenaltyPtr_;
  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;
  std::pair<vector_t, vector_t> positionLimits_;
  scalar_t offset_;
  bool isActive_ = true;
};

}  // namespace ocs2::humanoid