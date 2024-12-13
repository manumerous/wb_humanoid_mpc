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

#include <humanoid_centroidal_mpc_ros2/gains/GainsReceiver.h>
#include <humanoid_centroidal_mpc_ros2/gains/GainsUpdaterUtils.h>
#include <iostream>

namespace ocs2::humanoid {

GainsReceiver::GainsReceiver(rclcpp::Node::SharedPtr node,
                             const CentroidalMpcInterface& centroidalInterface,
                             std::vector<OptimalControlProblem>& ocpDefinitions)
    : node_(node) {
  // Setup subscriber
  rclcpp::QoS qos(1);
  qos.best_effort();
  gainsSubscription_ = node_->create_subscription<ocs2_ros2_msgs::msg::Gains>("/humanoid/mpc_gains", qos,
                                                                              [&](const ocs2_ros2_msgs::msg::Gains& msg) -> void {
                                                                                std::lock_guard<std::mutex> lock(gainsMutex_);
                                                                                currentGains_ = msg;
                                                                              });

  // Setup gains updaters
  // IMPORTANT NOTE: This will only work reliably as long as ocpDefinitions is not modified after this point!!!
  // Yeah, it's actually not great...
  gainsUpdaters_.clear();
  for (OptimalControlProblem& ocpDefinition : ocpDefinitions)
    gainsUpdaters_.push_back(utils::getGainsUpdaters(ocpDefinition, centroidalInterface, nullptr));
}

void GainsReceiver::preSolverRun(scalar_t initTime,
                                 scalar_t finalTime,
                                 const vector_t& currentState,
                                 const ReferenceManagerInterface& referenceManager) {
  // Lock the current gains
  std::lock_guard<std::mutex> lock(gainsMutex_);

  // If there are no gains, return
  if (!currentGains_.has_value()) return;

  // Update all gains
  for (const auto& individualGains : currentGains_->value) {
    bool found = false;
    for (auto& gainsUpdater : gainsUpdaters_) {
      if (gainsUpdater.find(individualGains.name) != gainsUpdater.end()) {
        gainsUpdater.at(individualGains.name)->setFromMessage(individualGains);
        found = true;
      }
    }
    // Print debug info
    if (!found) {
      RCLCPP_ERROR(node_->get_logger(), "[GainsReceiver::preSolverRun] Cost term `%s` does not seem to exist",
                   individualGains.name.c_str());
    } else if (printDebugInfo_) {
      RCLCPP_INFO(node_->get_logger(), "[GainsReceiver::preSolverRun] Updated gains for cost term `%s`", individualGains.name.c_str());
    }
  }

  // Reset the gains
  currentGains_.reset();
}

}  // namespace ocs2::humanoid