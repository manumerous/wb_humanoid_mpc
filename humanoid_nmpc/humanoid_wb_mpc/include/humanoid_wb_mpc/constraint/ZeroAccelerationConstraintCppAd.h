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

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

#include "humanoid_wb_mpc/constraint/EndEffectorDynamicsAccelerationsConstraint.h"

namespace ocs2::humanoid {

/**
 * Specializes the CppAd version of zero velocity constraint on an end-effector position and linear velocity.
 * Constructs the member EndEffectorKinematicsLinearVelConstraint object with number of constraints of 3.
 *
 * See also EndEffectorKinematicsLinearVelConstraint for the underlying computation.
 */
class ZeroAccelerationConstraintCppAd final : public StateInputConstraint {
 public:
  /**
   * Constructor
   * @param [in] referenceManager : Switched model ReferenceManager
   * @param [in] endEffectorDynamics: The dynamics interface to the target end-effector.
   * @param [in] contactPointIndex : The 3 DoF contact index.
   * @param [in] config: The constraint coefficients
   */
  ZeroAccelerationConstraintCppAd(
      const SwitchedModelReferenceManager& referenceManager,
      const EndEffectorDynamics<scalar_t>& endEffectorDynamics,
      size_t contactPointIndex,
      EndEffectorDynamicsAccelerationsConstraint::Config config = EndEffectorDynamicsAccelerationsConstraint::Config());

  ~ZeroAccelerationConstraintCppAd() override = default;
  ZeroAccelerationConstraintCppAd* clone() const override { return new ZeroAccelerationConstraintCppAd(*this); }

  bool isActive(scalar_t time) const override;
  size_t getNumConstraints(scalar_t time) const override { return 6; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time,
                                                           const vector_t& state,
                                                           const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  ZeroAccelerationConstraintCppAd(const ZeroAccelerationConstraintCppAd& rhs);

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  std::unique_ptr<EndEffectorDynamicsAccelerationsConstraint> eeAccelConstraintPtr_;
  const size_t contactPointIndex_;
};

}  // namespace ocs2::humanoid
