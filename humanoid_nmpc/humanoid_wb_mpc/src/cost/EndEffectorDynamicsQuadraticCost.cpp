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

#include "humanoid_wb_mpc/cost/EndEffectorDynamicsQuadraticCost.h"

#include "humanoid_wb_mpc/dynamics/DynamicsHelperFunctions.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

EndEffectorDynamicsQuadraticCost::EndEffectorDynamicsQuadraticCost(EndEffectorDynamicsWeights weights,
                                                                   const PinocchioInterface& pinocchioInterface,
                                                                   const EndEffectorDynamics<scalar_t>& endEffectorDynamics,
                                                                   const WBAccelMpcRobotModel<ad_scalar_t>& mpcRobotModel,
                                                                   std::string endEffectorName,
                                                                   std::string costName,
                                                                   const ModelSettings& modelSettings,
                                                                   size_t n_parameters)
    : StateInputCostGaussNewtonAd(),
      sqrtWeights_(weights.toVector().cwiseSqrt().cast<ad_scalar_t>()),
      endEffectorDynamicsPtr_(endEffectorDynamics.clone()),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelPtr_(mpcRobotModel.clone()) {
  initialize(mpcRobotModelPtr_->getStateDim(), mpcRobotModelPtr_->getInputDim(), n_parameters, costName, modelSettings.modelFolderCppAd,
             modelSettings.recompileLibrariesCppAd);
  frameID_ = pinocchioInterface.getModel().getFrameId(endEffectorName);
  std::cout << "Frame ID: " << frameID_ << std::endl;
  std::cout << "Initialized CentroidalMpcEndEffectorFootCost with weights: " << weights.toVector().transpose() << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

EndEffectorDynamicsQuadraticCost::EndEffectorDynamicsQuadraticCost(const EndEffectorDynamicsQuadraticCost& other)
    : StateInputCostGaussNewtonAd(other),
      sqrtWeights_(other.sqrtWeights_),
      frameID_(other.frameID_),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_),
      endEffectorDynamicsPtr_(other.endEffectorDynamicsPtr_->clone()),
      mpcRobotModelPtr_(other.mpcRobotModelPtr_->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector_t EndEffectorDynamicsQuadraticCost::getParameters(scalar_t time,
                                                         const TargetTrajectories& targetTrajectories,
                                                         const PreComputation& preComputation) const {
  // Interpolate reference
  const vector_t xRef = targetTrajectories.getDesiredState(time);
  const vector_t uRef = targetTrajectories.getDesiredInput(time);

  return getReferenceCostElement(xRef, uRef, *endEffectorDynamicsPtr_).getValues();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ad_vector_t EndEffectorDynamicsQuadraticCost::costVectorFunction(ad_scalar_t time,
                                                                 const ad_vector_t& state,
                                                                 const ad_vector_t& input,
                                                                 const ad_vector_t& parameters) {
  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;

  const auto& model = pinocchioInterfaceCppAd_.getModel();
  auto& data = pinocchioInterfaceCppAd_.getData();

  const ad_vector_t q = mpcRobotModelPtr_->getGeneralizedCoordinates(state);
  const ad_vector_t v = mpcRobotModelPtr_->getGeneralizedVelocities(state, input);
  const ad_vector_t a = computeGeneralizedAccelerations<ad_scalar_t>(state, input, pinocchioInterfaceCppAd_, *mpcRobotModelPtr_);

  pinocchio::forwardKinematics(model, data, q, v, a);
  auto frameData = pinocchio::updateFramePlacement(model, data, frameID_);

  ad_vector3_t position = frameData.translation();
  ad_vector3_t linearVelocity = pinocchio::getFrameVelocity(model, data, frameID_, rf).linear();
  ad_quaternion_t orientation = matrixToQuaternion(frameData.rotation());
  ad_vector3_t angularVelocity = pinocchio::getFrameVelocity(model, data, frameID_, rf).angular();
  auto accel = pinocchio::getFrameClassicalAcceleration(model, data, frameID_, rf);
  ad_vector3_t linearAccel = accel.linear();
  ad_vector3_t angularAccel = accel.angular();

  const ad_vector_t refOrientationCoeffs = parameters.segment(3, 4);
  const ad_quaternion_t refQuat(refOrientationCoeffs[0], refOrientationCoeffs[1], refOrientationCoeffs[2], refOrientationCoeffs[3]);

  const ad_vector_t orientationError = quaternionDistance<ad_scalar_t>(orientation, refQuat);

  ad_vector_t errors(12);
  errors << (position - parameters.head(3)), orientationError, (linearVelocity - parameters.segment(7, 3)),
      (angularVelocity - parameters.segment(10, 3)), (linearAccel - parameters.segment(13, 3)), (angularAccel - parameters.segment(16, 3));

  return errors.cwiseProduct(sqrtWeights_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

EndEffectorDynamicsCostElement<scalar_t> EndEffectorDynamicsQuadraticCost::getReferenceCostElement(
    const vector_t& state, const vector_t& input, const EndEffectorDynamics<scalar_t>& endEffectorDynamics) {
  EndEffectorDynamicsCostElement<scalar_t> costElement;
  costElement.setPosition(endEffectorDynamics.getPosition(state)[0]);
  costElement.setOrientation(endEffectorDynamics.getOrientation(state)[0]);
  costElement.setLinearVelocity(endEffectorDynamics.getVelocity(state, input)[0]);
  costElement.setAngularVelocity(endEffectorDynamics.getAngularVelocity(state, input)[0]);
  costElement.setLinearAcceleration(endEffectorDynamics.getLinearAcceleration(state, input)[0]);
  costElement.setAngularAcceleration(endEffectorDynamics.getAngularAcceleration(state, input)[0]);
  return costElement;
}

}  // namespace ocs2::humanoid
