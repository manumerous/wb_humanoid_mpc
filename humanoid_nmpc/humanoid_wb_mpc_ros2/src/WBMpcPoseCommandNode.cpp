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

#include <string>

#include <rclcpp/rclcpp.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_ros2_interfaces/command/TargetTrajectoriesKeyboardPublisher.h>

#include <humanoid_common_mpc/common/ModelSettings.h>
#include <humanoid_common_mpc/common/Types.h>

#include <humanoid_wb_mpc/common/WBAccelMpcRobotModel.h>

using namespace ocs2;

namespace {
scalar_t targetDisplacementVelocity;
scalar_t targetRotationVelocity;
scalar_t defaultBaseHeight;
vector_t defaultJointState;  // ToDo preallocate the correct size.
}  // namespace

scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) {
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t rotationTime = std::abs(dyaw) / targetRotationVelocity;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / targetDisplacementVelocity;
  return std::max(rotationTime, displacementTime);
}

/**
 * Converts command line to TargetTrajectories.
 * @param [in] commadLineTarget : [deltaX, deltaY, deltaZ, deltaYaw]
 * @param [in] observation : the current observation
 */
TargetTrajectories commandLineToTargetTrajectories(const vector_t& commadLineTarget, const SystemObservation& observation) {
  const vector_t currentPose = observation.state.head(6);
  const vector_t targetPose = [&]() {
    vector_t target(6);
    // base p_x, p_y are relative to current state
    target(0) = currentPose(0) + commadLineTarget(0);
    target(1) = currentPose(1) + commadLineTarget(1);
    // base z relative to the default height
    target(2) = defaultBaseHeight + commadLineTarget(2);
    // theta_z relative to current
    target(3) = currentPose(3) + commadLineTarget(3) * M_PI / 180.0;
    // theta_y, theta_x
    // target(4) = currentPose(4);
    // target(5) = currentPose(5);
    target(4) = 0.0;
    target(5) = 0.0;
    return target;
  }();

  // target reaching duration
  const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);

  // desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << currentPose, defaultJointState, vector_t::Zero(observation.state.size() / 2);
  stateTrajectory[1] << targetPose, defaultJointState, vector_t::Zero(observation.state.size() / 2);

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

int main(int argc, char* argv[]) {
  std::vector<std::string> programArgs;
  programArgs = rclcpp::remove_ros_arguments(argc, argv);
  if (programArgs.size() < 3) {
    throw std::runtime_error("No robot name, config folder, target command file, or description name specified. Aborting.");
  }

  rclcpp::init(argc, argv);

  const std::string robotName(argv[1]);
  const std::string taskFile(argv[2]);
  const std::string referenceFile(argv[3]);
  const std::string urdfFile(argv[4]);

  std::cerr << "Loading reference file: " << referenceFile << std::endl;

  loadData::loadCppDataType(referenceFile, "defaultBaseHeight", defaultBaseHeight);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", defaultJointState);
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity", targetRotationVelocity);
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", targetDisplacementVelocity);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(robotName + "_target");

  // goalPose: [deltaX, deltaY, deltaZ, deltaYaw]
  const scalar_array_t relativeBaseLimit{10.0, 10.0, 0.5, 360.0};
  TargetTrajectoriesKeyboardPublisher targetPoseCommand(node, robotName, relativeBaseLimit, &commandLineToTargetTrajectories);

  const std::string commandMsg = "Enter XYZ and Yaw (deg) displacements for the PELVIS, separated by spaces";
  targetPoseCommand.publishKeyboardCommand(commandMsg);

  // Successful exit
  return 0;
}
