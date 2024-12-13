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

#include "humanoid_wb_mpc/constraint/SwingLegVerticalConstraintCppAd.h"
#include "humanoid_wb_mpc/WBMpcPreComputation.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

SwingLegVerticalConstraintCppAd::SwingLegVerticalConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                                                 const EndEffectorDynamics<scalar_t>& endEffectorDynamics,
                                                                 size_t contactPointIndex)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      eeLinearConstraintPtr_(new EndEffectorDynamicsLinearAccConstraint(endEffectorDynamics, 1)),
      contactPointIndex_(contactPointIndex) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

SwingLegVerticalConstraintCppAd::SwingLegVerticalConstraintCppAd(const SwingLegVerticalConstraintCppAd& rhs)
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      eeLinearConstraintPtr_(rhs.eeLinearConstraintPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool SwingLegVerticalConstraintCppAd::isActive(scalar_t time) const {
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t SwingLegVerticalConstraintCppAd::getValue(scalar_t time,
                                                   const vector_t& state,
                                                   const vector_t& input,
                                                   const PreComputation& preComp) const {
  const auto& humanoidPreComp = cast<WBMpcPreComputation>(preComp);
  eeLinearConstraintPtr_->configure(humanoidPreComp.getEeNormalAccelerationConstraintConfigs()[contactPointIndex_]);

  return eeLinearConstraintPtr_->getValue(time, state, input, preComp);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation SwingLegVerticalConstraintCppAd::getLinearApproximation(scalar_t time,
                                                                                          const vector_t& state,
                                                                                          const vector_t& input,
                                                                                          const PreComputation& preComp) const {
  const auto& humanoidPreComp = cast<WBMpcPreComputation>(preComp);

  auto config = humanoidPreComp.getEeNormalAccelerationConstraintConfigs()[contactPointIndex_];

  eeLinearConstraintPtr_->configure(config);

  return eeLinearConstraintPtr_->getLinearApproximation(time, state, input, preComp);
}

}  // namespace ocs2::humanoid
