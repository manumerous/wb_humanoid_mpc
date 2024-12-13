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

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_common_mpc/common/Types.h"

#include "humanoid_wb_mpc/common/WBAccelMpcRobotModel.h"

namespace ocs2::humanoid {

template <typename SCALAR_T>
VECTOR6_T<SCALAR_T> computeBaseAcceleration(const VECTOR_T<SCALAR_T>& state,
                                            const VECTOR_T<SCALAR_T>& input,
                                            const PinocchioInterfaceTpl<SCALAR_T>& pinInterface,
                                            WBAccelMpcRobotModel<SCALAR_T>& mpcRobotModel);

template <typename SCALAR_T>
VECTOR_T<SCALAR_T> computeGeneralizedAccelerations(const VECTOR_T<SCALAR_T>& state,
                                                   const VECTOR_T<SCALAR_T>& input,
                                                   const PinocchioInterfaceTpl<SCALAR_T>& pinInterface,
                                                   WBAccelMpcRobotModel<SCALAR_T>& mpcRobotModel);

template <typename SCALAR_T>
VECTOR_T<SCALAR_T> computeStateDerivative(const VECTOR_T<SCALAR_T>& state,
                                          const VECTOR_T<SCALAR_T>& input,
                                          const PinocchioInterfaceTpl<SCALAR_T>& pinInterface,
                                          WBAccelMpcRobotModel<SCALAR_T>& mpcRobotModel);

template <typename SCALAR_T>
VECTOR_T<SCALAR_T> computeJointTorques(const VECTOR_T<SCALAR_T>& state,
                                       const VECTOR_T<SCALAR_T>& input,
                                       const PinocchioInterfaceTpl<SCALAR_T>& pinInterface,
                                       WBAccelMpcRobotModel<SCALAR_T>& mpcRobotModel);

}  // namespace ocs2::humanoid