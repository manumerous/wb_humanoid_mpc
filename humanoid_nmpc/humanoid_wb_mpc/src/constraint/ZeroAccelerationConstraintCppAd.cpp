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

#include "humanoid_wb_mpc/constraint/ZeroAccelerationConstraintCppAd.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ZeroAccelerationConstraintCppAd::ZeroAccelerationConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                                                 const EndEffectorDynamics<scalar_t>& endEffectorDynamics,
                                                                 size_t contactPointIndex,
                                                                 EndEffectorDynamicsAccelerationsConstraint::Config config)
    : StateInputConstraint(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      eeAccelConstraintPtr_(new EndEffectorDynamicsAccelerationsConstraint(endEffectorDynamics, 6, std::move(config))),
      contactPointIndex_(contactPointIndex) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ZeroAccelerationConstraintCppAd::ZeroAccelerationConstraintCppAd(const ZeroAccelerationConstraintCppAd& rhs)
    : StateInputConstraint(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      eeAccelConstraintPtr_(rhs.eeAccelConstraintPtr_->clone()),
      contactPointIndex_(rhs.contactPointIndex_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool ZeroAccelerationConstraintCppAd::isActive(scalar_t time) const {
  return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ZeroAccelerationConstraintCppAd::getValue(scalar_t time,
                                                   const vector_t& state,
                                                   const vector_t& input,
                                                   const PreComputation& preComp) const {
  return eeAccelConstraintPtr_->getValue(time, state, input, preComp);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ZeroAccelerationConstraintCppAd::getLinearApproximation(scalar_t time,
                                                                                          const vector_t& state,
                                                                                          const vector_t& input,
                                                                                          const PreComputation& preComp) const {
  return eeAccelConstraintPtr_->getLinearApproximation(time, state, input, preComp);
}

}  // namespace ocs2::humanoid
