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

#include <rclcpp/rclcpp.hpp>

#include "humanoid_common_mpc_ros2/gait/GaitKeyboardPublisher.h"

using namespace ocs2;
using namespace ocs2::humanoid;

int main(int argc, char* argv[]) {
  std::vector<std::string> programArgs;
  programArgs = rclcpp::remove_ros_arguments(argc, argv);
  if (programArgs.size() < 3) {
    throw std::runtime_error("No robot name, config folder, target command file, or description name specified. Aborting.");
  }

  const std::string robotName(argv[1]);
  const std::string gaitCommandFile(argv[2]);
  std::cerr << "Loading gait file: " << gaitCommandFile << std::endl;

  rclcpp::init(argc, argv);
  auto nodeHandle = std::make_shared<rclcpp::Node>(robotName + "_mpc_mode_schedule");

  GaitKeyboardPublisher gaitCommand(nodeHandle, gaitCommandFile, robotName, true);

  while (rclcpp::ok()) {
    gaitCommand.getKeyboardCommand();
  }

  // Successful exit
  return 0;
}
