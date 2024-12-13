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

#include <pinocchio/multibody/fwd.hpp>

#include <visualization_msgs/msg/marker_array.hpp>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <humanoid_common_mpc/common/ModelSettings.h>
#include <humanoid_common_mpc/common/MpcRobotModelBase.h>
#include <humanoid_common_mpc/contact/ContactWrenchMapper.h>

namespace ocs2::humanoid {

#define N_CONTACT_POLYGON_POINTS 4

class EquivalentContactCornerForcesVisualizer {
 public:
  EquivalentContactCornerForcesVisualizer(const std::string& taskFile,
                                          const PinocchioInterface& pinocchioInterface,
                                          const MpcRobotModelBase<scalar_t>& mpcRobotModel);

  visualization_msgs::msg::MarkerArray generateContactVisualizationForceMarkers(const vector_t& input,
                                                                                const contact_flag_t& contactFlags,
                                                                                const scalar_t& forceScale) const;

 private:
  /// \brief Rotates a 6D contact wrench vector from world into contact frame
  ///
  /// \param[in] contactWrench contact wrench [forces, torques] expressed in world frame
  /// \param[in] R_ContactToWorld Rotation Matrix from contact to world frame
  ///
  vector6_t rotateWrenchToContactFrame(const vector6_t& contactWrench, const matrix3_t R_ContactToWorld) const {
    matrix3_t R_WorldToContact = R_ContactToWorld.inverse();
    matrix6_t R_mat = matrix6_t::Zero();
    R_mat.block(0, 0, 3, 3) = R_WorldToContact;
    R_mat.block(3, 3, 3, 3) = R_WorldToContact;
    return (R_mat * contactWrench);
  };

  /// \brief Rotates a 3D force vector from local contact frame to world frame
  ///
  /// \param[in] contactForce contact force expressed in local contact frame
  /// \param[in] R_ContactToWorld Rotation Matrix from contact to world frame
  ///
  vector3_t rotateForceToWorldFrame(const vector3_t& contactForce, const matrix3_t R_ContactToWorld) const {
    return (R_ContactToWorld * contactForce);
  };
  const std::vector<pinocchio::FrameIndex> contactFrameIndizes_;
  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;
  const PinocchioInterface* pinocchioInterfacePtr_;
  std::vector<ContactWrenchMapper<N_CONTACT_POLYGON_POINTS>> contactMappers;
  std::vector<std::array<pinocchio::FrameIndex, N_CONTACT_POLYGON_POINTS>> polygonPointFrameIndizes_;
};

}  // namespace ocs2::humanoid
