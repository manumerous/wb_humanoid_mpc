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

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_common_mpc/constraint/EndEffectorKinematicsLinearVelConstraint.h"

#include "humanoid_common_mpc/common/MpcRobotModelBase.h"

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/swing_foot_planner/SwingTrajectoryPlanner.h"

namespace ocs2::humanoid {

/** Callback for caching and reference update */
class HumanoidPreComputation : public PreComputation {
 public:
  HumanoidPreComputation(PinocchioInterface pinocchioInterface,
                         const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                         const MpcRobotModelBase<scalar_t>& mpcRobotModel);
  virtual ~HumanoidPreComputation() override = default;

  virtual HumanoidPreComputation* clone() const override;

  virtual void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;
  const matrix3_t& getRWorldToContacts(size_t contactIndex) const { return R_world_to_contacts_[contactIndex]; }

  const std::vector<EndEffectorKinematicsLinearVelConstraint::Config>& getEeNormalVelocityConstraintConfigs() const {
    return eeNormalVelConConfigs_;
  }
  scalar_t getFootReferenceHeight(size_t contactIndex) const { return footHeightReferences_[contactIndex]; }

  PinocchioInterface& getPinocchioInterface() { return pinocchioInterface_; }
  const PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

 protected:
  HumanoidPreComputation(const HumanoidPreComputation& rhs);

  void updatePinocchioModelKinematics(const vector_t& generalizedCoordinates);

  PinocchioInterface pinocchioInterface_;
  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;

  std::vector<matrix3_t> R_world_to_contacts_;

  std::vector<EndEffectorKinematicsLinearVelConstraint::Config> eeNormalVelConConfigs_;
  std::vector<scalar_t> footHeightReferences_;
};

}  // namespace ocs2::humanoid
