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

#include "humanoid_common_mpc/cost/EndEffectorKinematicsQuadraticCost.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

EndEffectorKinematicsQuadraticCost::EndEffectorKinematicsQuadraticCost(EndEffectorKinematicsWeights weights,
                                                                       const PinocchioInterface& pinocchioInterface,
                                                                       const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                                       const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,
                                                                       std::string endEffectorName,
                                                                       const ModelSettings& modelSettings)
    : StateInputCostGaussNewtonAd(),
      sqrtWeights_(weights.toVector().cwiseSqrt()),
      endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelADPtr(mpcRobotModelAD.clone()) {
  std::cout << "Initialized EndEffectorKinematicsQuadraticCost with weights: " << weights.toVector().transpose() << std::endl;
  std::cout << "Frame name: " << endEffectorName << std::endl;
  frameID_ = pinocchioInterface.getModel().getFrameId(endEffectorName);
  std::cout << "Frame ID: " << frameID_ << std::endl;
  std::cout << "State dim: " << mpcRobotModelADPtr->getStateDim() << std::endl;
  std::cout << "Input dim: " << mpcRobotModelADPtr->getInputDim() << std::endl;
  std::cout << "Parameters dim: " << n_parameters_ << std::endl;

  initialize(mpcRobotModelADPtr->getStateDim(), mpcRobotModelADPtr->getInputDim(), n_parameters_,
             endEffectorName + "_KinematicsQuadraticCost", modelSettings.modelFolderCppAd, modelSettings.recompileLibrariesCppAd,
             modelSettings.verboseCppAd);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

EndEffectorKinematicsQuadraticCost::EndEffectorKinematicsQuadraticCost(const EndEffectorKinematicsQuadraticCost& other)
    : StateInputCostGaussNewtonAd(other),
      sqrtWeights_(other.sqrtWeights_),
      n_parameters_(other.n_parameters_),
      frameID_(other.frameID_),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_),
      endEffectorKinematicsPtr_(other.endEffectorKinematicsPtr_->clone()),
      mpcRobotModelADPtr(other.mpcRobotModelADPtr->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector_t EndEffectorKinematicsQuadraticCost::getParameters(scalar_t time,
                                                           const TargetTrajectories& targetTrajectories,
                                                           const PreComputation& preComputation) const {
  // Interpolate reference
  const vector_t xRef = targetTrajectories.getDesiredState(time);
  const vector_t uRef = targetTrajectories.getDesiredInput(time);

  vector_t parameters(n_parameters_);
  parameters << getReferenceCostElement(xRef, uRef, *endEffectorKinematicsPtr_).getValues(), sqrtWeights_;
  return parameters;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

EndEffectorKinematicsCostElement<scalar_t> EndEffectorKinematicsQuadraticCost::getReferenceCostElement(
    const vector_t& state, const vector_t& input, const EndEffectorKinematics<scalar_t>& endEffectorKinematics) {
  EndEffectorKinematicsCostElement<scalar_t> costElement;
  costElement.setPosition(endEffectorKinematics.getPosition(state)[0]);
  costElement.setOrientation(endEffectorKinematics.getOrientation(state)[0]);
  costElement.setLinearVelocity(endEffectorKinematics.getVelocity(state, input)[0]);
  costElement.setAngularVelocity(endEffectorKinematics.getAngularVelocity(state, input)[0]);
  return costElement;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ad_vector_t EndEffectorKinematicsQuadraticCost::costVectorFunction(ad_scalar_t time,
                                                                   const ad_vector_t& state,
                                                                   const ad_vector_t& input,
                                                                   const ad_vector_t& parameters) {
  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;

  const auto& model = pinocchioInterfaceCppAd_.getModel();
  auto& data = pinocchioInterfaceCppAd_.getData();

  const ad_vector_t q = mpcRobotModelADPtr->getGeneralizedCoordinates(state);
  const ad_vector_t v = mpcRobotModelADPtr->getGeneralizedVelocities(state, input);
  pinocchio::forwardKinematics(model, data, q, v);
  auto frameData = pinocchio::updateFramePlacement(model, data, frameID_);

  // auto oMf = data.oMf;
  ad_vector_t position = frameData.translation();
  ad_vector_t linearVelocity = pinocchio::getFrameVelocity(model, data, frameID_, rf).linear();
  ad_quaternion_t orientation = matrixToQuaternion(frameData.rotation());
  ad_vector_t angularVelocity = pinocchio::getFrameVelocity(model, data, frameID_, rf).angular();
  ad_vector_t taskSpaceVec(13);
  taskSpaceVec << position, orientation.coeffs(), linearVelocity, angularVelocity;
  // ad_vector_t errors = ad_vector_t::Zero(12);
  ad_vector_t errors = computeTaskSpaceErrors(EndEffectorKinematicsCostElement<ad_scalar_t>(taskSpaceVec),
                                              EndEffectorKinematicsCostElement<ad_scalar_t>(parameters.head(13)));

  const ad_vector_t sqrtWeightParams = parameters.segment<12>(13);  // EndEffectorKinematicsWeights vector element

  return errors.cwiseProduct(sqrtWeightParams);
}

}  // namespace ocs2::humanoid
