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
#include "humanoid_common_mpc_ros2/ros_comm/VelocityCommandKeyboardPublisher.h"

using namespace ocs2;
using namespace ocs2::humanoid;

namespace ocs2 {}  // namespace ocs2

int main(int argc, char* argv[]) {
  std::vector<std::string> programArgs;
  programArgs = rclcpp::remove_ros_arguments(argc, argv);
  if (programArgs.size() < 5) {
    throw std::runtime_error("No robot name, config folder, target command file, or description name specified. Aborting.");
  }

  const std::string robotName(argv[1]);
  const std::string taskFile(argv[2]);
  const std::string referenceFile(argv[3]);
  const std::string urdfFile(argv[4]);
  const std::string gaitFile(argv[5]);

  rclcpp::init(argc, argv);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>(robotName + "_keyboard_velocity_publisher");

  // goalPose: [deltaX, deltaY, deltaZ, deltaYaw]
  scalar_array_t relativeBaseLimit{0.5, 0.3, 0.4, 0.5};
  scalar_t defaultBaseHeight = 0.7;

  loadData::loadCppDataType(referenceFile, "maxDisplacementVelocityX", relativeBaseLimit[0]);
  loadData::loadCppDataType(referenceFile, "maxDisplacementVelocityY", relativeBaseLimit[1]);
  loadData::loadCppDataType(referenceFile, "maxDeltaPelvisHeight", relativeBaseLimit[2]);
  loadData::loadCppDataType(referenceFile, "maxRotationVelocity", relativeBaseLimit[3]);
  loadData::loadCppDataType(referenceFile, "defaultBaseHeight", defaultBaseHeight);

  VelocityCommandKeyboardPublisher targetVelCommand(node, "humanoid", relativeBaseLimit, defaultBaseHeight);

  const std::string commandMsg = "Enter v_x [m/s], v_y [m/s], delta_height [m], ang_vel_z [rad/s] of the PELVIS, separated by spaces";
  targetVelCommand.publishKeyboardCommand(commandMsg);

  // Successful exit
  return 0;
}
