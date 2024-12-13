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

#include <humanoid_centroidal_mpc_ros2/gains/GenericGuiInterface.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_ros2_msgs/msg/gains.hpp>
#include <optional>
#include <string>

namespace ocs2::humanoid {

class GainsUpdaterInterface {
 public:
  GainsUpdaterInterface(std::shared_ptr<GenericGuiInterface> gui) : gui_(gui) {}
  virtual ~GainsUpdaterInterface() = default;

  virtual bool initialize(ocs2::OptimalControlProblem& optimalControlProblem, const std::string& description) = 0;
  virtual bool drawGui() = 0;
  virtual void addToMessage(ocs2_ros2_msgs::msg::Gains& gains) = 0;
  virtual void setFromMessage(const ocs2_ros2_msgs::msg::IndividualGains& gains) = 0;

 protected:
  std::shared_ptr<GenericGuiInterface> gui_;
  std::string name_;
};

}  // namespace ocs2::humanoid