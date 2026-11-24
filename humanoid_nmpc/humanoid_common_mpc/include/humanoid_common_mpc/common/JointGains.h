/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
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

#include <string>
#include <vector>

#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

// Gains of one joint's low-level controller:
//   tau = kp (q_des - q) + kd (v_des - v) + tau_ff + dampingCompensation v_des
// dampingCompensation cancels the passive joint damping of a simulator (Nm s/rad) along the desired
// motion; set it to that damping, or 0 on hardware.
struct JointGains {
  scalar_t kp;
  scalar_t kd;
  scalar_t dampingCompensation{0.0};
};

/**
 * Loads the gains for each joint in jointNames from the "jointGains" block of an .info file:
 *
 *   jointGains
 *   {
 *     kp
 *     {
 *       hip_pitch   400.0   ; every joint whose name contains "hip_pitch"
 *       default      25.0   ; joints that no other entry matches
 *     }
 *     kd
 *     {
 *       ...
 *     }
 *     dampingCompensation   ; optional, 0 for all joints when absent
 *     {
 *       default      10.0
 *     }
 *   }
 *
 * Entries are matched against joint names by substring; the first match in file order wins.
 * Throws if a joint matches no entry and its table has no "default".
 *
 * @return Gains in the order of jointNames.
 */
std::vector<JointGains> loadJointGains(const std::string& file, const std::vector<std::string>& jointNames);

}  // namespace ocs2::humanoid
