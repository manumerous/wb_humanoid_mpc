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

#include <humanoid_mpc_msgs/msg/walking_velocity_command.hpp>
#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

struct WalkingVelocityCommand {
 public:
  WalkingVelocityCommand() = default;
  WalkingVelocityCommand(scalar_t v_x, scalar_t v_y, scalar_t desired_pelvis_h, scalar_t v_yaw)
      : linear_velocity_x(v_x), linear_velocity_y(v_y), desired_pelvis_height(desired_pelvis_h), angular_velocity_z(v_yaw){};
  WalkingVelocityCommand(const vector4_t& velCommand)
      : linear_velocity_x(velCommand(0)),
        linear_velocity_y(velCommand(1)),
        desired_pelvis_height(velCommand(2)),
        angular_velocity_z(velCommand(3)){};
  scalar_t linear_velocity_x = 0.0;
  scalar_t linear_velocity_y = 0.0;
  scalar_t desired_pelvis_height = 0.8;  // Above ground
  scalar_t angular_velocity_z = 0.0;

  void setToDefaultCommand() {
    linear_velocity_x = 0.0;
    linear_velocity_y = 0.0;
    desired_pelvis_height = 0.8;  // Above ground
    angular_velocity_z = 0.0;
  }

  vector4_t toVector() { return vector4_t(linear_velocity_x, linear_velocity_y, desired_pelvis_height, angular_velocity_z); };
};

inline WalkingVelocityCommand getWalkingVelocityCommandFromMsg(const humanoid_mpc_msgs::msg::WalkingVelocityCommand& msg) {
  WalkingVelocityCommand cmd;
  cmd.linear_velocity_x = std::clamp(msg.linear_velocity_x, -1.0, 1.0);
  cmd.linear_velocity_y = std::clamp(msg.linear_velocity_y, -1.0, 1.0);
  cmd.desired_pelvis_height = std::clamp(msg.desired_pelvis_height, 0.2, 1.0);
  cmd.angular_velocity_z = std::clamp(msg.angular_velocity_z, -1.0, 1.0);
  return cmd;
}
}  // namespace ocs2::humanoid
