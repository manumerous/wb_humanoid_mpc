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

#include <ocs2_mpc/MRT_BASE.h>
#include <ocs2_ros2_msgs/msg/mpc_flattened_controller.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ocs2::humanoid {

// This class implements a Ros subscriber to the policy using the MRT base interface .

class MRTPolicySubscriber : public MRT_BASE {
 public:
  explicit MRTPolicySubscriber(std::string topicPrefix = "anonymousRobot");

  /**
   * Destructor
   */
  ~MRTPolicySubscriber() override;

  void resetMpcNode(const TargetTrajectories& initTargetTrajectories) override{};

  void setCurrentObservation(const SystemObservation& currentObservation) override{};

  /**
   * Shut down the ROS nodes.
   */
  void shutdownNodes();

  /**
   * Launches the ROS publishers and subscribers to communicate with the MPC node.
   * @param [in] nodeHandle
   * @param [in] qos quality of service setting for the ROS publishers and subscribers.
   */
  void launchNodes(rclcpp::Node::SharedPtr node, const rclcpp::QoS& qos);

 private:
  /**
   * Callback method to receive the MPC policy as well as the mode sequence.
   * It only updates the policy variables with suffix (*Buffer_) variables.
   *
   * @param [in] msg: A constant pointer to the message
   */
  void mpcPolicyCallback(const ocs2_ros2_msgs::msg::MpcFlattenedController::SharedPtr msg);

 public:
  /**
   * Helper function to read a MPC policy message.
   *
   * @param [in] msg: A constant pointer to the message
   * @param [out] commandData: The MPC command data
   * @param [out] primalSolution: The MPC policy data
   * @param [out] performanceIndices: The MPC performance indices data
   */
  static void readPolicyMsg(const ocs2_ros2_msgs::msg::MpcFlattenedController& msg,
                            CommandData& commandData,
                            PrimalSolution& primalSolution,
                            PerformanceIndex& performanceIndices);

 private:
  rclcpp::Subscription<ocs2_ros2_msgs::msg::MpcFlattenedController>::SharedPtr mpcPolicySubscriber_;
  rclcpp::Node::SharedPtr node_;
  std::string topicPrefix_;
};

}  // namespace ocs2::humanoid
