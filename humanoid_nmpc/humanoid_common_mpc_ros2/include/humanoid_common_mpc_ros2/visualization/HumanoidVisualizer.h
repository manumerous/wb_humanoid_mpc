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

#include <tf2_ros/transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <utility>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros2_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros2_interfaces/mrt/DummyObserver.h>
#include <ocs2_ros2_interfaces/visualization/VisualizationColors.h>
#include <ocs2_ros2_msgs/msg/mpc_observation.hpp>

#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/constraint/FootCollisionConstraint.h"
#include "humanoid_common_mpc_ros2/visualization/EquivalentContactCornerForcesVisualizer.h"

namespace ocs2::humanoid {

class HumanoidVisualizer : public DummyObserver {
 public:
  /** Visualization settings (publicly available) */
  std::string frameId_ = "odom";              // Frame name all messages are published in
  scalar_t footMarkerDiameter_ = 0.03;        // Size of the spheres at the feet
  scalar_t footAlphaWhenLifted_ = 0.3;        // Alpha value when a foot is lifted.
  scalar_t forceScale_ = 200.0;               // Vector scale in N/m
  scalar_t velScale_ = 5.0;                   // Vector scale in m/s
  scalar_t copMarkerDiameter_ = 0.03;         // Size of the sphere at the center of pressure
  scalar_t supportPolygonLineWidth_ = 0.005;  // LineThickness for the support polygon
  scalar_t trajectoryLineWidth_ = 0.01;       // LineThickness for trajectories
  std::vector<Color> contactColorMap_ = {Color::purple, Color::orange, Color::blue, Color::green,
                                         Color::yellow};  // Colors for the contact points

  /**
   *
   * @param pinocchioInterface
   * @param n
   * @param maxUpdateFrequency : maximum publish frequency measured in MPC time.
   */
  HumanoidVisualizer(const std::string& taskFile,
                     PinocchioInterface pinocchioInterface,
                     const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                     rclcpp::Node::SharedPtr nodeHandle,
                     scalar_t maxUpdateFrequency = 60.0);

  HumanoidVisualizer(const HumanoidVisualizer&) = delete;

  virtual ~HumanoidVisualizer() override = default;

  void update(const SystemObservation& observation, const PrimalSolution& policy, const CommandData& command) override;

  virtual void launchSubscribers();

 protected:
  rclcpp::Node::SharedPtr node_handle_;
  rclcpp::Subscription<ocs2_ros2_msgs::msg::MpcObservation>::SharedPtr observationSubscriberPtr_{};

 private:
  void createVisualizationPublishers();

  virtual void mpcObservationCallback(const ocs2_ros2_msgs::msg::MpcObservation::SharedPtr msg);

  void publishObservation(const SystemObservation& observation) const;

  void publishBaseTransform(const vector6_t& basePose, const std::string prefix = "") const;

  void publishJointTransforms(const vector_t& jointAngles,
                              rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointPublisherPtr) const;

  void publishOptimizedStateTrajectory(const scalar_array_t& mpcTimeTrajectory,
                                       const vector_array_t& mpcStateTrajectory,
                                       const ModeSchedule& modeSchedule,
                                       const TargetTrajectories& targetTrajectories,
                                       scalar_t observationGroundHeight) const;

  void updatePinocchioFrames(const vector_t& state);

  void publishCartesianMarkers(const contact_flag_t& contactFlags, const vector_t& state, const vector_t& input) const;

  void publishSelfCollisionMarkers(const contact_flag_t& contactFlags, const vector_t& state) const;

  scalar_t lastTime_ = 0.0;
  scalar_t minPublishTimeDifference_;

  PinocchioInterface pinocchioInterface_;
  const EquivalentContactCornerForcesVisualizer contactVisualizer_;
  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointPublisherPtr_{};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr terminalJointPublisherPtr_{};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr terminalJointTargetPublisherPtr_{};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcasterPtr_{};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markerPublisherPtr_{};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr collsisionMarkerPublisherPtr_{};
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr stateOptimizedPublisherPtr_{};

  scalar_t prevPolicyTime = 0.0;
  vector_t prevPolicyState;
  vector_t prevPolicyInput;

  FootCollisionConstraint::Config collisionConfig_;

  const std::string base_link_name_;
};

}  // namespace ocs2::humanoid
