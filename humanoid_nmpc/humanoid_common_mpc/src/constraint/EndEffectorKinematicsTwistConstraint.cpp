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

#include "humanoid_common_mpc/constraint/EndEffectorKinematicsTwistConstraint.h"
#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

EndEffectorKinematicsTwistConstraint::EndEffectorKinematicsTwistConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                                           size_t numConstraints,
                                                                           Config config)
    : StateInputConstraint(ConstraintOrder::Linear),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      numConstraints_(numConstraints),
      ground_plane_normal_(0.0, 0.0, 1.0),
      config_(std::move(config)) {
  if (endEffectorKinematicsPtr_->getIds().size() != 1) {
    throw std::runtime_error("[EndEffectorKinematicsTwistConstraint] this class only accepts a single end-effector!");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

EndEffectorKinematicsTwistConstraint::EndEffectorKinematicsTwistConstraint(const EndEffectorKinematicsTwistConstraint& rhs)
    : StateInputConstraint(rhs),
      endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
      numConstraints_(rhs.numConstraints_),
      ground_plane_normal_(rhs.ground_plane_normal_),
      config_(rhs.config_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void EndEffectorKinematicsTwistConstraint::configure(Config&& config) {
  assert(config.b.rows() == numConstraints_);
  assert(config.Ax.size() > 0 || config.Av.size() > 0);
  assert((config.Ax.size() > 0 && config.Ax.rows() == numConstraints_) || config.Ax.size() == 0);
  assert((config.Ax.size() > 0 && config.Ax.cols() == 6) || config.Ax.size() == 0);
  assert((config.Av.size() > 0 && config.Av.rows() == numConstraints_) || config.Av.size() == 0);
  assert((config.Av.size() > 0 && config.Av.cols() == 6) || config.Av.size() == 0);
  config_ = std::move(config);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector_t EndEffectorKinematicsTwistConstraint::getValue(scalar_t time,
                                                        const vector_t& state,
                                                        const vector_t& input,
                                                        const PreComputation& preComp) const {
  vector_t f = config_.b;
  if (config_.Ax.size() > 0) {
    // foot pose is a 6D vector containing the foot position and orientation error wrt. to the ground normal
    vector6_t footPose;
    footPose << endEffectorKinematicsPtr_->getPosition(state).front(),
        endEffectorKinematicsPtr_->getOrientationErrorWrtPlane(state, {ground_plane_normal_}).front();
    f.noalias() += config_.Ax * footPose;
  }
  if (config_.Av.size() > 0) {
    f.noalias() += config_.Av * endEffectorKinematicsPtr_->getTwist(state, input).front();
  }
  return f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

VectorFunctionLinearApproximation EndEffectorKinematicsTwistConstraint::getLinearApproximation(scalar_t time,
                                                                                               const vector_t& state,
                                                                                               const vector_t& input,
                                                                                               const PreComputation& preComp) const {
  VectorFunctionLinearApproximation linearApproximation =
      VectorFunctionLinearApproximation::Zero(getNumConstraints(time), state.size(), input.size());

  linearApproximation.f = config_.b;

  // Orientation error gains are ignored for now
  // This is equal with assuming that the bottom 3 rows of Ax are zero.
  if (config_.Ax.size() > 0) {
    const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
    const auto orientationApprox =
        endEffectorKinematicsPtr_->getOrientationErrorWrtPlaneLinearApproximation(state, {ground_plane_normal_}).front();

    linearApproximation.f.head(3).noalias() += config_.Ax.topLeftCorner(3, 3) * positionApprox.f;
    linearApproximation.f.tail(3).noalias() += config_.Ax.bottomRightCorner(3, 3) * orientationApprox.f;
    linearApproximation.dfdx.topRows(3).noalias() += config_.Ax.topLeftCorner(3, 3) * positionApprox.dfdx;
    linearApproximation.dfdx.bottomRows(3).noalias() += config_.Ax.bottomRightCorner(3, 3) * orientationApprox.dfdx;
  }

  if (config_.Av.size() > 0) {
    const auto velocityApprox = endEffectorKinematicsPtr_->getTwistLinearApproximation(state, input).front();
    linearApproximation.f.noalias() += config_.Av * velocityApprox.f;
    linearApproximation.dfdx.noalias() += config_.Av * velocityApprox.dfdx;
    linearApproximation.dfdu.noalias() += config_.Av * velocityApprox.dfdu;
  }

  return linearApproximation;
}

}  // namespace ocs2::humanoid
