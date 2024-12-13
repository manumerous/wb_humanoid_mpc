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

#include <Eigen/Dense>
#include <memory>
#include <string>
#include <unordered_map>

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace ocs2::humanoid {

/**
 * @class COMVisualizer
 * @brief A ROS2 node for visualizing the Center of Mass (COM) of a robot
 *
 * This class loads a URDF model, subscribes to joint states, calculates the
 * robot's COM, and publishes visualization markers for RViz.
 */
class COMVisualizer : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for the COMVisualizer
   *
   * Initializes the node, loads the URDF, sets up subscribers and publishers,
   * and starts the COM calculation and visualization timer.
   */
  COMVisualizer();

 private:
  /**
   * @brief Loads the robot model from a URDF file
   * @param urdf_file Path to the URDF file
   * @return true if the model was loaded successfully, false otherwise
   */
  bool loadRobotModel(const std::string& urdf_file);

  /**
   * @brief Prints information about the joints in the loaded model
   */
  void printJointInfo();

  /**
   * @brief Initializes the joint name to index map
   */
  void initializeJointMap();

  /**
   * @brief Publishes the COM and related visualization markers
   */
  void publishCOM();

  /**
   * @brief Calculates the Center of Mass of the robot
   * @return Eigen::Vector3d representing the COM position
   */
  Eigen::Vector3d calculateCOM();

  /**
   * @brief Callback function for joint state messages
   * @param msg Shared pointer to the received JointState message
   */
  void updateJointPositions(const sensor_msgs::msg::JointState::SharedPtr msg);

  // ROS publishers and subscribers
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr com_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // Pinocchio model and data
  pinocchio::Model pinocchio_model_;
  pinocchio::Data pinocchio_data_;

  // Joint positions and mapping
  Eigen::VectorXd joint_positions_;
  std::unordered_map<std::string, int> joint_name_to_index_;
};
}  // namespace ocs2::humanoid