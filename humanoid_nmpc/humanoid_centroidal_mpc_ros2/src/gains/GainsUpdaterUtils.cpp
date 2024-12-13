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

#include <humanoid_centroidal_mpc_ros2/gains/GainsUpdaterUtils.h>
#include <iostream>

#include <humanoid_centroidal_mpc_ros2/gains/EndEffectorFootGainsUpdater.h>
#include <humanoid_centroidal_mpc_ros2/gains/EndEffectorKinematicsGainsUpdater.h>
#include <humanoid_centroidal_mpc_ros2/gains/FootCollisionGainsUpdater.h>
#include <humanoid_centroidal_mpc_ros2/gains/JointLimitsGainsUpdater.h>
#include <humanoid_centroidal_mpc_ros2/gains/QuadraticStateCostWeightsUpdater.h>
#include <humanoid_centroidal_mpc_ros2/gains/QuadraticStateInputGainsUpdater.h>
#include <humanoid_centroidal_mpc_ros2/gains/StateInputConstraintGainsUpdater.h>
#include <humanoid_centroidal_mpc_ros2/gains/StateInputSoftConstraintGainsUpdater.h>

namespace ocs2::humanoid::utils {

std::vector<std::string> getStateDescriptions(const ocs2::humanoid::ModelSettings& modelSettings) {
  std::vector<std::string> stateDescriptions = {"vcom_x",   "vcom_y",   "vcom_z",   "L_x / mass",   "L_y / mass",   "L_z / mass",
                                                "p_base_x", "p_base_y", "p_base_z", "theta_base_z", "theta_base_y", "theta_base_x"};
  stateDescriptions.insert(stateDescriptions.end(), modelSettings.mpcModelJointNames.begin(), modelSettings.mpcModelJointNames.end());
  if (stateDescriptions.size() != ocs2::humanoid::STATE_DIM) {
    std::cout << stateDescriptions.size() << " VS " << ocs2::humanoid::STATE_DIM << std::endl;
    throw std::runtime_error("[getStateDescriptions] Dimension mismatch!");
  }
  return stateDescriptions;
}

std::vector<std::string> getInputDescriptions(const ocs2::humanoid::ModelSettings& modelSettings) {
  std::vector<std::string> inputDescriptions = {"W_l_x", "W_l_y", "W_l_z", "W_l_a", "W_l_b", "W_l_c",
                                                "W_r_x", "W_r_y", "W_r_z", "W_r_a", "W_r_b", "W_r_c"};
  for (const auto& jointName : modelSettings.mpcModelJointNames) {
    inputDescriptions.emplace_back("vel_" + jointName);
  }
  if (inputDescriptions.size() != ocs2::humanoid::INPUT_DIM) {
    std::cout << inputDescriptions.size() << " VS " << ocs2::humanoid::INPUT_DIM << std::endl;
    throw std::runtime_error("[getInputDescriptions] Dimension mismatch!");
  }
  return inputDescriptions;
}

std::unordered_map<std::string, std::shared_ptr<GainsUpdaterInterface>> getGainsUpdaters(OptimalControlProblem& optimalControlProblem,
                                                                                         const CentroidalMpcInterface& centroidalInterface,
                                                                                         std::shared_ptr<GenericGuiInterface> gui) {
  // Initialize vector
  std::unordered_map<std::string, std::shared_ptr<GainsUpdaterInterface>> gainsUpdaters;

  // Find all relevant descriptions
  const auto costNames = centroidalInterface.getCostNames();
  const auto terminalCostNames = centroidalInterface.getTerminalCostNames();
  const auto stateSoftConstraintNames = centroidalInterface.getStateSoftConstraintNames();
  const auto softConstraintNames = centroidalInterface.getSoftConstraintNames();
  const auto equalityConstraintNames = centroidalInterface.getEqualityConstraintNames();

  const size_t totalSize = costNames.size() + terminalCostNames.size() + stateSoftConstraintNames.size() + softConstraintNames.size() +
                           equalityConstraintNames.size();
  std::vector<std::string> descriptions;
  descriptions.reserve(totalSize);

  descriptions.insert(descriptions.end(), costNames.begin(), costNames.end());
  descriptions.insert(descriptions.end(), terminalCostNames.begin(), terminalCostNames.end());
  descriptions.insert(descriptions.end(), stateSoftConstraintNames.begin(), stateSoftConstraintNames.end());
  descriptions.insert(descriptions.end(), softConstraintNames.begin(), softConstraintNames.end());
  descriptions.insert(descriptions.end(), equalityConstraintNames.begin(), equalityConstraintNames.end());

  // Initialize updaters
  for (const auto& description : descriptions) {
    bool found = false;

    auto checkAndAddCandidate = [&](std::shared_ptr<GainsUpdaterInterface> candidate) {
      if (candidate->initialize(optimalControlProblem, description)) {
        gainsUpdaters.insert({description, candidate});
        found = true;
      } else {
        candidate.reset();
      }
    };
    checkAndAddCandidate(std::make_shared<QuadraticStateInputGainsUpdater>(centroidalInterface.getMpcRobotModel(),
                                                                           centroidalInterface.modelSettings(), gui));
    checkAndAddCandidate(std::make_shared<QuadraticStateCostWeightsUpdater>(centroidalInterface.getMpcRobotModel(),
                                                                            centroidalInterface.modelSettings(), gui));
    checkAndAddCandidate(std::make_shared<EndEffectorFootGainsUpdater>(gui));
    checkAndAddCandidate(std::make_shared<JointLimitsGainsUpdater>(gui));
    checkAndAddCandidate(std::make_shared<EndEffectorKinematicsGainsUpdater>(gui));
    checkAndAddCandidate(std::make_shared<FootCollisionGainsUpdater>(gui));
    checkAndAddCandidate(std::make_shared<StateInputConstraintGainsUpdater>(gui));
    checkAndAddCandidate(std::make_shared<StateInputSoftConstraintGainsUpdater>(gui));

    if (!found) std::cout << "[getGainsUpdaters] Could not find updater for `" << description << "`" << std::endl;
  }

  return gainsUpdaters;
}

}  // namespace ocs2::humanoid::utils