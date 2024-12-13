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

// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>

#include "humanoid_common_mpc/common/ModelSettings.h"

#include <ocs2_pinocchio_interface/urdf.h>
#include <pinocchio/multibody/model.hpp>

namespace ocs2::humanoid {

///
/// \brief Checks that the joint names in the pinocchio interface are in the same order as defined in ModelSettings.cpp
///
/// \param[in] pinocchioInterface: A pinocchio Interface
///

void checkPinocchioJointNaming(const PinocchioInterface& pinocchioInterface, const ModelSettings& modelSettings, bool verbose = false);

///
/// \brief Creates a standard pinocchio model from the urdf
///
/// \param[in] urdfFilePath: The absolute path to the URDF file for the robot.
///
/// \param[out] jointLimits A std pair of joint limits consisting of {lower_bounds, upper_bounds}

std::pair<vector_t, vector_t> readPinocchioJointLimits(const PinocchioInterface& pinocchioInterface,
                                                       const ModelSettings& modelSettings,
                                                       bool verbose = true);

void scalePinocchioModelInertia(pinocchio::ModelTpl<scalar_t>& model, scalar_t targetRobotMass, bool verbose = true);

}  // namespace ocs2::humanoid