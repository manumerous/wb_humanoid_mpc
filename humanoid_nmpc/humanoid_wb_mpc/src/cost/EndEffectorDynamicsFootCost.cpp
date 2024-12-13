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

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_wb_mpc/cost/EndEffectorDynamicsFootCost.h"
#include "humanoid_wb_mpc/dynamics/DynamicsHelperFunctions.h"

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

EndEffectorDynamicsFootCost::EndEffectorDynamicsFootCost(const SwitchedModelReferenceManager& referenceManager,
                                                         EndEffectorDynamicsWeights weights,
                                                         const PinocchioInterface& pinocchioInterface,
                                                         const EndEffectorDynamics<scalar_t>& endEffectorDynamics,
                                                         const WBAccelMpcRobotModel<ad_scalar_t>& mpcRobotModel,
                                                         size_t contactIndex,
                                                         std::string costName,
                                                         const ModelSettings& modelSettings)
    : StateInputCostGaussNewtonAd(),
      referenceManagerPtr_(&referenceManager),
      sqrtWeights_(weights.toVector().cwiseSqrt()),
      frameID_(pinocchioInterface.getModel().getFrameId(modelSettings.contactNames[contactIndex])),
      contactIndex_(contactIndex),
      endEffectorDynamicsPtr_(endEffectorDynamics.clone()),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelPtr_(mpcRobotModel.clone()) {
  initialize(mpcRobotModel.getStateDim(), mpcRobotModel.getInputDim(), 37, costName, modelSettings.modelFolderCppAd,
             modelSettings.recompileLibrariesCppAd);
  std::cout << "Frame ID: " << frameID_ << std::endl;
  std::cout << "Initialized EndEffectorDynamicsFootCost with weights: " << weights.toVector().transpose() << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

EndEffectorDynamicsFootCost::EndEffectorDynamicsFootCost(const EndEffectorDynamicsFootCost& other)
    : StateInputCostGaussNewtonAd(other),
      referenceManagerPtr_(other.referenceManagerPtr_),
      sqrtWeights_(other.sqrtWeights_),
      frameID_(other.frameID_),
      contactIndex_(other.contactIndex_),
      endEffectorDynamicsPtr_(other.endEffectorDynamicsPtr_->clone()),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_),
      mpcRobotModelPtr_(other.mpcRobotModelPtr_->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ad_vector_t EndEffectorDynamicsFootCost::costVectorFunction(ad_scalar_t time,
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

  ad_vector3_t linearVelocity = pinocchio::getFrameVelocity(model, data, frameID_, rf).linear();
  ad_quaternion_t orientation = matrixToQuaternion(frameData.rotation());
  ad_vector3_t angularVelocity = pinocchio::getFrameVelocity(model, data, frameID_, rf).angular();
  auto accel = pinocchio::getFrameClassicalAcceleration(model, data, frameID_, rf);
  ad_vector3_t linearAccel = accel.linear();
  ad_vector3_t angularAccel = accel.angular();

  const PlanarEndEffectorDynamicsReference<ad_scalar_t> reference(parameters.head(18));
  const ad_vector_t sqrtWeightParams = parameters.segment(18, 18);  // EndEffectorDynamicsWeights vector element
  const ad_scalar_t impactProximityScaler = parameters[36];

  ad_vector_t errors(18);
  errors << ad_vector3_t::Zero(), quaternionDistanceToPlane<ad_scalar_t>(orientation, reference.getPlaneNormal()),
      (linearVelocity - reference.getLinearVelocity()), (angularVelocity - reference.getAngularVelocity()),
      (linearAccel - reference.getLinearAcceleration()), (angularAccel - reference.getAngularAcceleration());

  return errors.cwiseProduct(sqrtWeightParams) * impactProximityScaler;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector_t EndEffectorDynamicsFootCost::getParameters(scalar_t time,
                                                    const TargetTrajectories& targetTrajectories,
                                                    const PreComputation& preComputation) const {
  // Interpolate reference
  const vector_t xRef = targetTrajectories.getDesiredState(time);
  const vector_t uRef = targetTrajectories.getDesiredInput(time);

  const scalar_t impactProximityScaler = referenceManagerPtr_->getSwingTrajectoryPlanner()->getImpactProximityFactor(contactIndex_, time);

  // TODO Update this reference for non flat ground in the future
  vector_t parameters(37);
  parameters.head(3) = vector3_t(0.0, 0.0, 0.0);         // Reference position
  parameters.segment(3, 3) = vector3_t(0.0, 0.0, 1.0);   // Ground plane normal
  parameters.segment(6, 3) = vector3_t(0.0, 0.0, 0.0);   // Reference linear velocity
  parameters.segment(9, 3) = vector3_t(0.0, 0.0, 0.0);   // Reference angular velocity
  parameters.segment(12, 3) = vector3_t(0.0, 0.0, 0.0);  // Reference linear acceleration
  parameters.segment(15, 3) = vector3_t(0.0, 0.0, 0.0);  // Reference angular acceleration
  parameters.segment(18, 18) = sqrtWeights_;             // EndEffectorDynamicsWeights vector element

  parameters[36] = impactProximityScaler;

  return parameters;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
}  // namespace ocs2::humanoid
