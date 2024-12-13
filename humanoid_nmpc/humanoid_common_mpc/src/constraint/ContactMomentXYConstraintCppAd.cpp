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

#include "humanoid_common_mpc/constraint/ContactMomentXYConstraintCppAd.h"

#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ContactMomentXYConstraintCppAd::ContactMomentXYConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                                               const ContactRectangle& contactRectangle,
                                                               size_t contactPointIndex,
                                                               const PinocchioInterface& pinocchioInterface,
                                                               const MpcRobotModelBase<ad_scalar_t>& mpcRobotModel,
                                                               std::string costName,
                                                               const ModelSettings& modelSettings)
    : StateInputConstraintCppAd(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      mpcRobotModelPtr_(&mpcRobotModel),
      contactRectangle_(contactRectangle),
      contactPointIndex_(contactPointIndex),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()) {
  initialize(mpcRobotModelPtr_->getStateDim(), mpcRobotModelPtr_->getInputDim(), 0, costName, modelSettings.modelFolderCppAd,
             modelSettings.recompileLibrariesCppAd, modelSettings.verboseCppAd);
}
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ContactMomentXYConstraintCppAd::ContactMomentXYConstraintCppAd(const ContactMomentXYConstraintCppAd& other)
    : StateInputConstraintCppAd(other),
      referenceManagerPtr_(other.referenceManagerPtr_),
      mpcRobotModelPtr_(other.mpcRobotModelPtr_),
      contactRectangle_(other.contactRectangle_),
      contactPointIndex_(other.contactPointIndex_),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool ContactMomentXYConstraintCppAd::isActive(scalar_t time) const {
  if (!isActive_) return false;
  return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ad_vector_t ContactMomentXYConstraintCppAd::constraintFunction(ad_scalar_t time,
                                                               const ad_vector_t& state,
                                                               const ad_vector_t& input,
                                                               const ad_vector_t& parameters) const {
  const auto& model = pinocchioInterfaceCppAd_.getModel();
  auto data = pinocchioInterfaceCppAd_.getData();  // make copy of model since method is const
  updateFramePlacements(mpcRobotModelPtr_->getGeneralizedCoordinates(state), model, data);
  pinocchio::FrameIndex frameID = getContactFrameIndex(pinocchioInterfaceCppAd_, *mpcRobotModelPtr_, contactPointIndex_);

  const ad_vector3_t localForce = rotateVectorWorldToLocal(mpcRobotModelPtr_->getContactForce(input, contactPointIndex_), data, frameID);
  const ad_vector3_t localMoments = rotateVectorWorldToLocal(mpcRobotModelPtr_->getContactMoment(input, contactPointIndex_), data, frameID);

  ad_vector_t constraintValue(4);
  constraintValue << localMoments.x() - contactRectangle_.getBounds().y_min * localForce.z(),
      -localMoments.x() + contactRectangle_.getBounds().y_max * localForce.z(),
      -localMoments.y() - contactRectangle_.getBounds().x_min * localForce.z(),
      localMoments.y() + contactRectangle_.getBounds().x_max * localForce.z();
  return constraintValue;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

}  // namespace ocs2::humanoid