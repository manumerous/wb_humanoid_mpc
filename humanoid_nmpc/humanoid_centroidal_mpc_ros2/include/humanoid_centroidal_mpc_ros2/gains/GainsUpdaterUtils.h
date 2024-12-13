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

#include <humanoid_centroidal_mpc/CentroidalMpcInterface.h>
#include <humanoid_centroidal_mpc_ros2/gains/GainsUpdaterInterface.h>
#include <humanoid_common_mpc/common/ModelSettings.h>
#include <memory>
#include <string>
#include <unordered_map>

namespace ocs2::humanoid::utils {

/**
 * Returns entire list of state descriptions in the same order as they are stored in the state vector.
 * @return
 */
std::vector<std::string> getStateDescriptions(const ocs2::humanoid::ModelSettings& modelSettings);

/**
 * Returns entire list of input descriptions in the same order as they are stored in the input vector.
 * @return
 */
std::vector<std::string> getInputDescriptions(const ocs2::humanoid::ModelSettings& modelSettings);

/**
 * Fill in all relevant gain updaters
 * @return
 */
std::unordered_map<std::string, std::shared_ptr<GainsUpdaterInterface>> getGainsUpdaters(OptimalControlProblem& optimalControlProblem,
                                                                                         const CentroidalMpcInterface& centroidalInterface,
                                                                                         std::shared_ptr<GenericGuiInterface> gui);

}  // namespace ocs2::humanoid::utils