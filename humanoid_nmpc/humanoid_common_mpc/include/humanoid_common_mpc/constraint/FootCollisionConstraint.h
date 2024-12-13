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

#include <pinocchio/fwd.hpp>

#include <ocs2_core/constraint/StateConstraintCppAd.h>
#include <ocs2_core/cost/StateCost.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <pinocchio/algorithm/frames.hpp>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid {

/**
 * Implements the constraint h(t,x,u) >= 0 to prevent collisions of the feet.
 */

class FootCollisionConstraint final : public StateConstraintCppAd {
 public:
  struct Config {
    // Foot and ankle
    std::string leftAnkleFrame;
    std::string rightAnkleFrame;

    std::string leftFootCenterFrame{"foot_l_contact"};
    std::string rightFootCenterFrame{"foot_r_contact"};

    std::string leftFootFrame1{"foot_l_contact_collision_p_1"};
    std::string rightFootFrame1{"foot_r_contact_collision_p_1"};

    std::string leftFootFrame2{"foot_l_contact_collision_p_2"};
    std::string rightFootFrame2{"foot_r_contact_collision_p_2"};

    scalar_t footCollisionSphereRadius;

    // Knee
    std::string leftKneeFrame;
    std::string rightKneeFrame;
    scalar_t kneeCollisionSphereRadius;
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  FootCollisionConstraint(const SwitchedModelReferenceManager& referenceManager,
                          const PinocchioInterface& pinocchioInterface,
                          const MpcRobotModelBase<ad_scalar_t>& mpcRobotModel,
                          const Config& config,
                          std::string costName,
                          const ModelSettings& modelSettings);

  ~FootCollisionConstraint() override = default;
  FootCollisionConstraint* clone() const override { return new FootCollisionConstraint(*this); }

  bool isActive(scalar_t time) const override;
  bool getActive() const { return isActive_; }
  void setActive(bool active) { isActive_ = active; }

  size_t getNumConstraints(scalar_t time) const override { return numConstraints_; };

  vector_t getParameters(scalar_t time, const PreComputation& preComputation) const override {
    vector_t parameters(2);
    parameters << cfg_.footCollisionSphereRadius, cfg_.kneeCollisionSphereRadius;
    return parameters;
  };

  void setSphereRadii(scalar_t footCollisionSphereRadius, scalar_t kneeCollisionSphereRadius) {
    cfg_.footCollisionSphereRadius = footCollisionSphereRadius;
    cfg_.kneeCollisionSphereRadius = kneeCollisionSphereRadius;
  }

  void getSphereRadii(scalar_t& footCollisionSphereRadius, scalar_t& kneeCollisionSphereRadius) const {
    footCollisionSphereRadius = cfg_.footCollisionSphereRadius;
    kneeCollisionSphereRadius = cfg_.kneeCollisionSphereRadius;
  }

  static Config loadFootCollisionConstraintConfig(const std::string taskFile, bool verbose = false);

 private:
  ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const override;

  FootCollisionConstraint(const FootCollisionConstraint& other);

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  PinocchioInterfaceCppAd pinocchioInterfaceCppAd_;
  const MpcRobotModelBase<ad_scalar_t>* const mpcRobotModelPtr_;
  Config cfg_;

  const size_t numConstraints_ = 16;
  bool isActive_ = true;
};

}  // namespace ocs2::humanoid