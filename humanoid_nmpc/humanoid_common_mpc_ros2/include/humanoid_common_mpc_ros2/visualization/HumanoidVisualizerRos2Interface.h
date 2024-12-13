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

#include <ocs2_ros2_interfaces/mrt/MRT_ROS_Interface.h>
#include <ocs2_ros2_msgs/msg/mpc_flattened_controller.hpp>

#include "humanoid_common_mpc_ros2/ros_comm/MRTPolicySubscriber.h"
#include "humanoid_common_mpc_ros2/visualization/HumanoidVisualizer.h"

namespace ocs2::humanoid {

class HumanoidVisualizerRos2Interface : public HumanoidVisualizer {
 public:
  HumanoidVisualizerRos2Interface(const std::string& taskFile,
                                  PinocchioInterface pinocchioInterface,
                                  const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                  rclcpp::Node::SharedPtr nodeHandle,
                                  scalar_t maxUpdateFrequency = 100.0);

  HumanoidVisualizerRos2Interface(const HumanoidVisualizerRos2Interface&) = delete;

  ~HumanoidVisualizerRos2Interface() override = default;

  void launchSubscribers() override;

 private:
  void mpcObservationCallback(const ocs2_ros2_msgs::msg::MpcObservation::SharedPtr msg) override;

  MRTPolicySubscriber mRTPolicySubscriper_;
};

}  // namespace ocs2::humanoid
