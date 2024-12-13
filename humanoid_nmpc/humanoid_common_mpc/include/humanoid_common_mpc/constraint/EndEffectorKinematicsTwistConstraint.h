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

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

/**
 * Defines a linear constraint on an end-effector position (xee) and linear velocity (vee).
 * g(xee, vee) = Ax * xee + Av * vee + b
 * - For defining constraint of type g(xee), set Av to matrix_t(0, 0)
 * - For defining constraint of type g(vee), set Ax to matrix_t(0, 0)
 */
class EndEffectorKinematicsTwistConstraint final : public StateInputConstraint {
 public:
  /**
   * Coefficients of the linear constraints of the form:
   * g(xee, vee) = Ax * xee + Av * vee + b = 0
   */
  struct Config {
    vector_t b;
    matrix_t Ax;
    matrix_t Av;
  };

  /**
   * Constructor
   * @param [in] endEffectorKinematics: The kinematic interface to the target end-effector.
   * @param [in] numConstraints: The number of constraints {1, 2, 3, 4, 5, 6}
   * @param [in] config: The constraint coefficients, g(xee, vee) = Ax * xee + Av * vee + b
   */
  EndEffectorKinematicsTwistConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                       size_t numConstraints,
                                       Config config = Config());

  ~EndEffectorKinematicsTwistConstraint() override = default;
  EndEffectorKinematicsTwistConstraint* clone() const override { return new EndEffectorKinematicsTwistConstraint(*this); }

  /** Sets a new constraint coefficients. */
  void configure(Config&& config);
  /** Sets a new constraint coefficients. */
  void configure(const Config& config) { this->configure(Config(config)); }
  /** Gets a reference of the config to allow to modify it. */
  Config& getConfig() { return config_; }

  /** Gets the underlying end-effector kinematics interface. */
  EndEffectorKinematics<scalar_t>& getEndEffectorKinematics() { return *endEffectorKinematicsPtr_; }

  size_t getNumConstraints(scalar_t time) const override { return numConstraints_; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time,
                                                           const vector_t& state,
                                                           const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  EndEffectorKinematicsTwistConstraint(const EndEffectorKinematicsTwistConstraint& rhs);
  vector3_t ground_plane_normal_;
  std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  const size_t numConstraints_;
  Config config_;
};

}  // namespace ocs2::humanoid
