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
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <memory>
#include <mutex>
#include <ocs2_ros2_msgs/msg/gains.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ocs2::humanoid {

class GainsReceiver : public SolverSynchronizedModule {
 public:
  GainsReceiver(rclcpp::Node::SharedPtr node,
                const CentroidalMpcInterface& centroidalInterface,
                std::vector<OptimalControlProblem>& ocpDefinitions);
  ~GainsReceiver() override = default;

  void preSolverRun(scalar_t initTime,
                    scalar_t finalTime,
                    const vector_t& currentState,
                    const ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const PrimalSolution& primalSolution) override {}

 public:
  bool printDebugInfo_ = true;

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<ocs2_ros2_msgs::msg::Gains>::SharedPtr gainsSubscription_;
  std::mutex gainsMutex_;
  std::optional<ocs2_ros2_msgs::msg::Gains> currentGains_ = std::nullopt;
  std::vector<std::unordered_map<std::string, std::shared_ptr<GainsUpdaterInterface>>> gainsUpdaters_;
};

}  // namespace ocs2::humanoid
