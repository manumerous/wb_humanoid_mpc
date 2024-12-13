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

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "Eigen/Dense"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "rclcpp/rclcpp.hpp"
#include "urdf/model.h"

#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "humanoid_common_mpc_ros2/visualization/COMVisualizer.hpp"

namespace ocs2::humanoid {

COMVisualizer::COMVisualizer() : Node("COMVisualizer") {
  // Load URDF
  std::string urdf_file = this->declare_parameter<std::string>("urdf_file", "");
  if (!loadRobotModel(urdf_file)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load URDF. Exiting.");
    return;
  }

  // Print joint information
  printJointInfo();

  // Initialize joint name to index map
  initializeJointMap();

  // Resize joint positions array to match number of joints
  joint_positions_ = Eigen::VectorXd::Zero(pinocchio_model_.nq);

  // Subscribe to joint states
  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, std::bind(&COMVisualizer::updateJointPositions, this, std::placeholders::_1));

  // Create publisher for COM marker
  com_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("humanoid/com_markers", 10);

  // Timer to publish COM periodically
  timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&COMVisualizer::publishCOM, this));

  RCLCPP_INFO(this->get_logger(), "Humanoid COM Visualizer initialized.");
}

bool COMVisualizer::loadRobotModel(const std::string& urdf_file) {
  try {
    pinocchio::urdf::buildModel(urdf_file, pinocchio_model_);
    pinocchio_data_ = pinocchio::Data(pinocchio_model_);
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load URDF: %s", e.what());
    return false;
  }
}

void COMVisualizer::printJointInfo() {
  RCLCPP_INFO(this->get_logger(), "Total number of joints (including universe joint): %d", static_cast<int>(pinocchio_model_.njoints));
  RCLCPP_INFO(this->get_logger(), "Number of degrees of freedom: %d", static_cast<int>(pinocchio_model_.nq));

  for (int i = 0; i < pinocchio_model_.njoints; i++) {
    const auto& joint = pinocchio_model_.joints[i];
    RCLCPP_INFO(this->get_logger(), "Joint %d: %s (nq: %d, nv: %d)", i, pinocchio_model_.names[i].c_str(), static_cast<int>(joint.nq()),
                static_cast<int>(joint.nv()));
  }
}

void COMVisualizer::initializeJointMap() {
  for (int i = 1; i < pinocchio_model_.njoints; i++) {  // Start from 1 to skip the universe joint!!!
    joint_name_to_index_[pinocchio_model_.names[i]] = i;
    RCLCPP_INFO(this->get_logger(), "Mapped joint '%s' to index %d", pinocchio_model_.names[i].c_str(), i);
  }
}

void COMVisualizer::publishCOM() {
  Eigen::Vector3d com = calculateCOM();

  visualization_msgs::msg::MarkerArray marker_array;

  // COM Marker
  visualization_msgs::msg::Marker com_marker;
  com_marker.header.frame_id = "link_pelvis";
  com_marker.header.stamp = this->now();
  com_marker.ns = "humanoid_com";
  com_marker.id = 0;
  com_marker.type = visualization_msgs::msg::Marker::SPHERE;
  com_marker.action = visualization_msgs::msg::Marker::ADD;
  com_marker.pose.position.x = com.x();
  com_marker.pose.position.y = com.y();
  com_marker.pose.position.z = com.z();
  com_marker.scale.x = 0.1;
  com_marker.scale.y = 0.1;
  com_marker.scale.z = 0.1;
  com_marker.color.r = 0.0;
  com_marker.color.g = 0.0;
  com_marker.color.b = 1.0;
  com_marker.color.a = 1.0;

  // Ground Projection Marker
  visualization_msgs::msg::Marker projection_marker;
  projection_marker.header = com_marker.header;
  projection_marker.ns = "humanoid_com_projection";
  projection_marker.id = 1;
  projection_marker.type = visualization_msgs::msg::Marker::CYLINDER;
  projection_marker.action = visualization_msgs::msg::Marker::ADD;
  projection_marker.pose.position.x = com.x();
  projection_marker.pose.position.y = com.y();
  projection_marker.pose.position.z = -0.85;  // Slightly above the ground
  projection_marker.scale.x = 0.1;
  projection_marker.scale.y = 0.1;
  projection_marker.scale.z = 0.02;
  projection_marker.color.r = 1.0;
  projection_marker.color.g = 0.0;
  projection_marker.color.b = 0.0;
  projection_marker.color.a = 0.7;

  // Line connecting COM to its projection
  visualization_msgs::msg::Marker line_marker;
  line_marker.header = com_marker.header;
  line_marker.ns = "humanoid_com_line";
  line_marker.id = 2;
  line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_marker.action = visualization_msgs::msg::Marker::ADD;
  line_marker.scale.x = 0.01;  // Line width
  line_marker.color.r = 0.0;
  line_marker.color.g = 1.0;
  line_marker.color.b = 0.0;
  line_marker.color.a = 0.5;

  geometry_msgs::msg::Point p1, p2;
  p1.x = com.x();
  p1.y = com.y();
  p1.z = com.z();
  p2.x = com.x();
  p2.y = com.y();
  p2.z = -0.85;
  line_marker.points.push_back(p1);
  line_marker.points.push_back(p2);

  marker_array.markers.push_back(com_marker);
  marker_array.markers.push_back(projection_marker);
  marker_array.markers.push_back(line_marker);

  com_pub_->publish(marker_array);
}

Eigen::Vector3d COMVisualizer::calculateCOM() {
  pinocchio::forwardKinematics(pinocchio_model_, pinocchio_data_, joint_positions_);
  pinocchio::updateGlobalPlacements(pinocchio_model_, pinocchio_data_);
  return pinocchio::centerOfMass(pinocchio_model_, pinocchio_data_);
}

void COMVisualizer::updateJointPositions(const sensor_msgs::msg::JointState::SharedPtr msg) {
  for (size_t i = 0; i < msg->name.size(); i++) {
    auto it = joint_name_to_index_.find(msg->name[i]);
    if (it != joint_name_to_index_.end()) {
      int idx = it->second;
      if (idx > 0 && idx < joint_positions_.size()) {
        joint_positions_(pinocchio_model_.joints[idx].idx_q()) = msg->position[i];
      } else {
        // RCLCPP_WARN(this->get_logger(), "Joint index out of range for joint %s: %d", msg->name[i].c_str(), idx);
      }
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Joint %s not found in the model", msg->name[i].c_str());
    }
  }
}
}  // namespace ocs2::humanoid
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ocs2::humanoid::COMVisualizer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
