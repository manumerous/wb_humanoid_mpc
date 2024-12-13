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

#include <stdexcept>

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include "humanoid_wb_mpc/common/WBAccelMpcRobotModel.h"

namespace ocs2::humanoid {
/**
 * Mapping between OCS2 and pinocchio state and input
 */
template <typename SCALAR_T>
class WBAccelPinocchioStateInputMapping final : public PinocchioStateInputMapping<SCALAR_T> {
 public:
  WBAccelPinocchioStateInputMapping() = default;
  ~WBAccelPinocchioStateInputMapping() override = default;
  WBAccelPinocchioStateInputMapping<SCALAR_T>* clone() const override { return new WBAccelPinocchioStateInputMapping(*this); };

  /** Get the pinocchio joint configuration from OCS2 state and input vectors. */
  vector_t getPinocchioJointPosition(const vector_t& state) const override { return mapping.getGeneralizedCoordinates(state); };

  /** Get the pinocchio joint velocity from OCS2 state and input vectors. */
  vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override {
    return mapping.getGeneralizedVelocities(state, input);
  };

  /** Mapps pinocchio jacobians dfdq, dfdv to OCS2 jacobians dfdx, dfdu. */
  std::pair<matrix_t, matrix_t> getOcs2Jacobian(const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override {
    throw std::runtime_error("Not implemented");
    return {Jq, Jv};
  };

 private:
  WBAccelPinocchioStateInputMapping(const WBAccelPinocchioStateInputMapping<SCALAR_T>& rhs) = default;

  WBAccelMpcRobotModel<SCALAR_T> mapping;
};

}  // namespace ocs2::humanoid
