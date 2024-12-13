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

// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>

#include "humanoid_common_mpc_ros2/visualization/HumanoidVisualizerRos2Interface.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
HumanoidVisualizerRos2Interface::HumanoidVisualizerRos2Interface(const std::string& taskFile,
                                                                 PinocchioInterface pinocchioInterface,
                                                                 const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                                                 rclcpp::Node::SharedPtr nodeHandle,
                                                                 scalar_t maxUpdateFrequency)
    : HumanoidVisualizer(taskFile, pinocchioInterface, mpcRobotModel, nodeHandle, maxUpdateFrequency), mRTPolicySubscriper_("humanoid") {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidVisualizerRos2Interface::launchSubscribers() {
  auto qos = rclcpp::QoS(10);
  qos.best_effort();
  mRTPolicySubscriper_.launchNodes(node_handle_, qos);

  rclcpp::spin_some(node_handle_);

  observationSubscriberPtr_ = node_handle_->create_subscription<ocs2_ros2_msgs::msg::MpcObservation>(
      "/humanoid/mpc_observation", qos, std::bind(&HumanoidVisualizerRos2Interface::mpcObservationCallback, this, std::placeholders::_1));

  std::cout << "observationSubscriberPtr_ initialized" << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidVisualizerRos2Interface::mpcObservationCallback(const ocs2_ros2_msgs::msg::MpcObservation::SharedPtr msg) {
  auto currentObservation = ros_msg_conversions::readObservationMsg(*msg);

  if (mRTPolicySubscriper_.initialPolicyReceived()) {
    mRTPolicySubscriper_.updatePolicy();

    update(currentObservation, mRTPolicySubscriper_.getPolicy(), mRTPolicySubscriper_.getCommand());

  } else {
    update(currentObservation, PrimalSolution(), CommandData());
  }
}

}  // namespace ocs2::humanoid
