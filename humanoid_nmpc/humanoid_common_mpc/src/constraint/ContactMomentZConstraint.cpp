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

#include "humanoid_common_mpc/constraint/ContactMomentZConstraint.h"

#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ContactMomentZConstraint::ContactMomentZConstraint(const SwitchedModelReferenceManager& referenceManager,
                                                   const PinocchioInterface& pinocchioInterface,
                                                   const MpcRobotModelBase<ad_scalar_t>& mpcRobotModel,
                                                   size_t contactPointIndex,
                                                   std::string costName,
                                                   const ModelSettings& modelSettings)
    : StateInputConstraintCppAd(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelPtr_(&mpcRobotModel),
      contactPointIndex_(contactPointIndex) {
  initialize(mpcRobotModelPtr_->getStateDim(), mpcRobotModelPtr_->getInputDim(), 0, costName, modelSettings.modelFolderCppAd,
             modelSettings.recompileLibrariesCppAd, modelSettings.verboseCppAd);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ContactMomentZConstraint::ContactMomentZConstraint(const ContactMomentZConstraint& rhs)
    : StateInputConstraintCppAd(rhs),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      pinocchioInterfaceCppAd_(rhs.pinocchioInterfaceCppAd_),
      mpcRobotModelPtr_(rhs.mpcRobotModelPtr_),
      contactPointIndex_(rhs.contactPointIndex_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

bool ContactMomentZConstraint::isActive(scalar_t time) const {
  if (!isActive_) return false;
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ad_vector_t ContactMomentZConstraint::constraintFunction(ad_scalar_t time,
                                                         const ad_vector_t& state,
                                                         const ad_vector_t& input,
                                                         const ad_vector_t& parameters) const {
  const auto& model = pinocchioInterfaceCppAd_.getModel();
  auto data = pinocchioInterfaceCppAd_.getData();  // make copy of model since method is const
  updateFramePlacements(mpcRobotModelPtr_->getGeneralizedCoordinates(state), model, data);
  pinocchio::FrameIndex frameID = getContactFrameIndex(pinocchioInterfaceCppAd_, *mpcRobotModelPtr_, contactPointIndex_);

  const ad_vector3_t localForce =
      rotateVectorWorldToLocal<ad_scalar_t>(mpcRobotModelPtr_->getContactForce(input, contactPointIndex_), data, frameID);
  const ad_vector3_t localMoments =
      rotateVectorWorldToLocal<ad_scalar_t>(mpcRobotModelPtr_->getContactMoment(input, contactPointIndex_), data, frameID);

  // Constraint Value in local frame to ensure:
  // Mz = Fy * r_cop_x - Fx * r_cop_y
  // By subsituting Mx = Fz * r_cop_y and My = - Fz * r_cop_x
  ad_vector_t value(1);
  value << localMoments.x() * localForce.x() + localMoments.y() * localForce.y() + localMoments.z() * localForce.z();
  return value;
}

}  // namespace ocs2::humanoid
