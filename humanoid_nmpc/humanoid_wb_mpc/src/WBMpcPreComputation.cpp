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

#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_core/misc/Numerics.h>

#include <humanoid_wb_mpc/WBMpcPreComputation.h>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
WBMpcPreComputation::WBMpcPreComputation(PinocchioInterface pinocchioInterface,
                                         const SwingTrajectoryPlanner& swingTrajectoryPlanner,
                                         const MpcRobotModelBase<scalar_t>& mpcRobotModel)
    : HumanoidPreComputation(pinocchioInterface, swingTrajectoryPlanner, mpcRobotModel) {
  eeNormalAccConConfigs_.resize(N_CONTACTS);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

WBMpcPreComputation::WBMpcPreComputation(const WBMpcPreComputation& rhs)
    : HumanoidPreComputation(rhs), eeNormalAccConConfigs_(rhs.eeNormalAccConConfigs_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
WBMpcPreComputation* WBMpcPreComputation::clone() const {
  return new WBMpcPreComputation(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void WBMpcPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {
  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }

  const ModelSettings::FootConstraintConfig& footConstraintCfg = mpcRobotModelPtr_->modelSettings.footConstraintConfig;
  updatePinocchioModelKinematics(mpcRobotModelPtr_->getGeneralizedCoordinates(x));

  // lambda to set config for normal velocity constraints
  auto eeNormalVelConConfig = [&](size_t footIndex) {
    EndEffectorKinematicsLinearVelConstraint::Config config;
    config.b =
        (vector_t(1) << -footConstraintCfg.linearVelocityErrorGain_z * swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t))
            .finished();
    config.Av = (matrix_t(1, 3) << 0.0, 0.0, footConstraintCfg.linearVelocityErrorGain_z).finished();
    if (!numerics::almost_eq(footConstraintCfg.positionErrorGain_z, 0.0)) {
      config.b(0) -= footConstraintCfg.positionErrorGain_z * swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t);
      config.Ax = (matrix_t(1, 3) << 0.0, 0.0, footConstraintCfg.positionErrorGain_z).finished();
    }
    return config;
  };

  // lambda to set config for normal velocity constraints
  auto eeNormalAccConConfig = [&](size_t footIndex) {
    EndEffectorDynamicsLinearAccConstraint::Config config;
    config.b =
        (vector_t(1) << -footConstraintCfg.linearVelocityErrorGain_z * swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t))
            .finished();
    config.Av = (matrix_t(1, 3) << 0.0, 0.0, footConstraintCfg.linearVelocityErrorGain_z).finished();
    config.b(0) -= footConstraintCfg.linearAccelerationErrorGain_z * swingTrajectoryPlannerPtr_->getZaccelerationConstraint(footIndex, t);
    config.Aa = (matrix_t(1, 3) << 0.0, 0.0, footConstraintCfg.linearAccelerationErrorGain_z).finished();
    if (!numerics::almost_eq(footConstraintCfg.positionErrorGain_z, 0.0)) {
      config.b(0) -= footConstraintCfg.positionErrorGain_z * swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t);
      config.Ax = (matrix_t(1, 3) << 0.0, 0.0, footConstraintCfg.positionErrorGain_z).finished();
    }
    return config;
  };

  if (request.contains(Request::Constraint)) {
    for (size_t i = 0; i < N_CONTACTS; i++) {
      eeNormalAccConConfigs_[i] = eeNormalAccConConfig(i);
      pinocchio::FrameIndex frameID = pinocchioInterface_.getModel().getFrameId(mpcRobotModelPtr_->modelSettings.contactNames6DoF[i]);
      R_world_to_contacts_[i] = pinocchioInterface_.getData().oMf[frameID].rotation().inverse();
    }
  }
}

}  // namespace ocs2::humanoid
