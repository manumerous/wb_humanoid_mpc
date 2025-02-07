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

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_core/misc/Numerics.h>

#include <humanoid_common_mpc/HumanoidPreComputation.h>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
HumanoidPreComputation::HumanoidPreComputation(PinocchioInterface pinocchioInterface,
                                               const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                                               const MpcRobotModelBase<scalar_t>& mpcRobotModel)
    : pinocchioInterface_(std::move(pinocchioInterface)),
      swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner),
      mpcRobotModelPtr_(&mpcRobotModel) {
  eeNormalVelConConfigs_.resize(N_CONTACTS);
  R_world_to_contacts_.resize(N_CONTACTS);
  footHeightReferences_.resize(N_CONTACTS);
  for (size_t i = 0; i < N_CONTACTS; i++) {
    R_world_to_contacts_[i] = matrix3_t::Identity();
    footHeightReferences_[i] = 0.0;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

HumanoidPreComputation::HumanoidPreComputation(const HumanoidPreComputation& rhs)
    : pinocchioInterface_(rhs.pinocchioInterface_),
      swingTrajectoryPlannerPtr_(rhs.swingTrajectoryPlannerPtr_),
      mpcRobotModelPtr_(rhs.mpcRobotModelPtr_),
      R_world_to_contacts_(rhs.R_world_to_contacts_),
      eeNormalVelConConfigs_(rhs.eeNormalVelConConfigs_),
      footHeightReferences_(rhs.footHeightReferences_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
HumanoidPreComputation* HumanoidPreComputation::clone() const {
  return new HumanoidPreComputation(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidPreComputation::updatePinocchioModelKinematics(const vector_t& q) {
  const pinocchio::Model& model = pinocchioInterface_.getModel();
  pinocchio::Data& data = pinocchioInterface_.getData();

  // Perform the forward kinematics over the kinematic tree
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void HumanoidPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }

  updatePinocchioModelKinematics(mpcRobotModelPtr_->getGeneralizedCoordinates(x));

  // lambda to set config for normal velocity constraints
  auto eeNormalVelConConfig = [&](size_t footIndex) {
    EndEffectorKinematicsLinearVelConstraint::Config config;
    config.b = (vector_t(1) << -swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t)).finished();
    config.Av = (matrix_t(1, 3) << 0.0, 0.0, 1.0).finished();
    const ModelSettings::FootConstraintConfig& footConstraintCfg = mpcRobotModelPtr_->modelSettings.footConstraintConfig;
    if (!numerics::almost_eq(footConstraintCfg.positionErrorGain_z, 0.0)) {
      config.b(0) -= footConstraintCfg.positionErrorGain_z * swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t);
      config.Ax = (matrix_t(1, 3) << 0.0, 0.0, footConstraintCfg.positionErrorGain_z).finished();
    }
    return config;
  };

  if (request.contains(Request::Constraint)) {
    for (size_t i = 0; i < N_CONTACTS; i++) {
      eeNormalVelConConfigs_[i] = eeNormalVelConConfig(i);
      pinocchio::FrameIndex frameID = pinocchioInterface_.getModel().getFrameId(mpcRobotModelPtr_->modelSettings.contactNames6DoF[i]);
      R_world_to_contacts_[i] = pinocchioInterface_.getData().oMf[frameID].rotation().inverse();
      footHeightReferences_[i] = swingTrajectoryPlannerPtr_->getZpositionConstraint(i, t);
    }
  }
}

}  // namespace ocs2::humanoid
