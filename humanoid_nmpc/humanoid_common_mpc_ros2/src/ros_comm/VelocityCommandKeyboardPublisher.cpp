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

#include "humanoid_common_mpc_ros2/ros_comm/VelocityCommandKeyboardPublisher.h"

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/Display.h>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VelocityCommandKeyboardPublisher::VelocityCommandKeyboardPublisher(rclcpp::Node::SharedPtr nodeHandle,
                                                                   const std::string& topicPrefix,
                                                                   const scalar_array_t& targetCommandLimits,
                                                                   scalar_t defaultBaseHeight)
    : targetCommandLimits_(Eigen::Map<const vector_t>(targetCommandLimits.data(), targetCommandLimits.size())),
      defaultBaseHeight_(defaultBaseHeight),
      node_(nodeHandle) {
  assert(targetCommandLimits_.size() == 4);
  // Target publisher
  commandPublisherPtr_ =
      node_->create_publisher<humanoid_mpc_msgs::msg::WalkingVelocityCommand>(topicPrefix + "/walking_velocity_command", 1);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void VelocityCommandKeyboardPublisher::publishKeyboardCommand(const std::string& commadMsg) {
  while (rclcpp::ok()) {
    // get command line
    std::cout << commadMsg << ": ";
    const vector4_t commandLineInput = getCommandLine().cwiseMin(targetCommandLimits_).cwiseMax(-targetCommandLimits_);

    // display
    std::cout << "The following command is published: [" << toDelimitedString(commandLineInput) << "]\n\n";

    humanoid_mpc_msgs::msg::WalkingVelocityCommand msg;
    msg.linear_velocity_x = commandLineInput[0] / targetCommandLimits_[0];
    msg.linear_velocity_y = commandLineInput[1] / targetCommandLimits_[1];
    msg.desired_pelvis_height = defaultBaseHeight_ + commandLineInput[2];
    msg.angular_velocity_z = commandLineInput[3] / targetCommandLimits_[3];

    // publish TargetTrajectories
    commandPublisherPtr_->publish(msg);
  }  // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector4_t VelocityCommandKeyboardPublisher::getCommandLine() {
  // get command line as one long string
  auto shouldTerminate = []() { return !rclcpp::ok(); };
  const std::string line = getCommandLineString(shouldTerminate);

  // a line to words
  const std::vector<std::string> words = stringToWords(line);

  const size_t targetCommandSize = targetCommandLimits_.size();
  vector4_t targetCommand = vector_t::Zero(targetCommandSize);
  for (size_t i = 0; i < std::min(words.size(), targetCommandSize); i++) {
    targetCommand(i) = static_cast<scalar_t>(stof(words[i]));
  }

  return targetCommand;
}

}  // namespace ocs2::humanoid
