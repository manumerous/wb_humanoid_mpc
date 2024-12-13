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

#include <humanoid_centroidal_mpc/CentroidalMpcInterface.h>
#include <humanoid_common_mpc_ros2/visualization/HumanoidVisualizer.h>

#include "ocs2_centroidal_model/CentroidalModelPinocchioMapping.h"

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

using namespace ocs2;
using namespace ocs2::humanoid;

int main(int argc, char** argv) {
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
  auto node = std::make_shared<rclcpp::Node>("humanoid_visualizer");

  CentroidalMpcInterface interface(taskFile, urdfFile, referenceFile, gaitFile);

  HumanoidVisualizer visualization(taskFile, interface.getPinocchioInterface(), interface.getMpcRobotModel(), node);

  SystemObservation observation;

  observation.state = vector_t::Zero(interface.getMpcRobotModel().getStateDim());
  observation.state[8] = 0.9;
  observation.input = vector_t::Zero(interface.getMpcRobotModel().getInputDim());

  PrimalSolution dummySolution;
  CommandData commandData;

  rclcpp::Rate loop_rate(10);

  while (rclcpp::ok()) {
    // observation.state += 0.1*vector_t::Random(14);
    observation.state[13] += 0.1;
    visualization.update(observation, dummySolution, commandData);
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  return 0;
}
