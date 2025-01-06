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

#include <humanoid_common_mpc/common/Types.h>
#include <rclcpp/rclcpp.hpp>
#include "humanoid_mpc_msgs/msg/walking_velocity_command.hpp"

namespace ocs2::humanoid {

/**
 * This class lets the user to publish a velocity command form command line.
 */
class VelocityCommandKeyboardPublisher final {
 public:
  /**
   * Constructor
   *
   * @param [in] nodeHandle: ROS node handle.
   * @param [in] topicPrefix: The TargetTrajectories will be published on "topicPrefix_mpc_target" topic. Moreover, the latest
   * observation is be expected on "topicPrefix_mpc_observation" topic.
   * @param [in] targetCommandLimits: The limits of the loaded command from command-line (for safety purposes).
   */
  VelocityCommandKeyboardPublisher(rclcpp::Node::SharedPtr nodeHandle,
                                   const std::string& topicPrefix,
                                   const scalar_array_t& targetCommandLimits,
                                   scalar_t defaultBaseHeight);

  // VelocityCommandKeyboardPublisher(const VelocityCommandKeyboardPublisher&) = delete;

  /**
   * Publishes command line input. If the input command is shorter than the expected command
   * size (targetCommandSize), the method will set the rest of the command to zero.
   *
   * @param [in] commadMsg: Message to be displayed on screen.
   */
  void publishKeyboardCommand(const std::string& commadMsg = "Enter command, separated by space");

 private:
  /** Gets the target from command line. */
  vector4_t getCommandLine();

  const vector4_t targetCommandLimits_;
  const scalar_t defaultBaseHeight_;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<humanoid_mpc_msgs::msg::WalkingVelocityCommand>::SharedPtr commandPublisherPtr_{};
};

}  // namespace ocs2::humanoid
