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

#include "humanoid_centroidal_mpc/cost/CentroidalMpcEndEffectorFootCost.h"

#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

CentroidalMpcEndEffectorFootCost::CentroidalMpcEndEffectorFootCost(const SwitchedModelReferenceManager& referenceManager,
                                                                   EndEffectorKinematicsWeights weights,
                                                                   const PinocchioInterface& pinocchioInterface,
                                                                   const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,
                                                                   size_t contactIndex,
                                                                   std::string costName,
                                                                   const ModelSettings& modelSettings)
    : StateInputCostGaussNewtonAd(),
      referenceManagerPtr_(&referenceManager),
      sqrtWeights_(weights.toVector().cwiseSqrt()),
      frameID_(pinocchioInterface.getModel().getFrameId(modelSettings.contactNames[contactIndex])),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelAdPtr_(mpcRobotModelAD.clone()),
      contactIndex_(contactIndex) {
  initialize(mpcRobotModelAD.getStateDim(), mpcRobotModelAD.getInputDim(), 25, costName, modelSettings.modelFolderCppAd,
             modelSettings.recompileLibrariesCppAd);
  std::cout << "Frame ID: " << frameID_ << std::endl;
  std::cout << "Initialized CentroidalMpcEndEffectorFootCost with weights: " << weights.toVector().transpose() << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

CentroidalMpcEndEffectorFootCost::CentroidalMpcEndEffectorFootCost(const CentroidalMpcEndEffectorFootCost& other)
    : StateInputCostGaussNewtonAd(other),
      referenceManagerPtr_(other.referenceManagerPtr_),
      sqrtWeights_(other.sqrtWeights_),
      frameID_(other.frameID_),
      contactIndex_(other.contactIndex_),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_),
      mpcRobotModelAdPtr_(other.mpcRobotModelAdPtr_->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ad_vector_t CentroidalMpcEndEffectorFootCost::costVectorFunction(ad_scalar_t time,
                                                                 const ad_vector_t& state,
                                                                 const ad_vector_t& input,
                                                                 const ad_vector_t& parameters) {
  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;

  const PlanarEndEffectorKinematicsPlanarReference<ad_scalar_t> reference(parameters.head(12));
  const ad_vector_t sqrtWeightParams = parameters.segment(12, 12);  // EndEffectorKinematicsWeights vector element
  const ad_scalar_t impactProximityScaler = parameters[24];

  const auto& model = pinocchioInterfaceCppAd_.getModel();
  auto& data = pinocchioInterfaceCppAd_.getData();

  const ad_vector_t q = mpcRobotModelAdPtr_->getGeneralizedCoordinates(state);
  const ad_vector_t v = mpcRobotModelAdPtr_->getGeneralizedVelocities(state, input);
  pinocchio::forwardKinematics(model, data, q, v);
  auto frameData = pinocchio::updateFramePlacement(model, data, frameID_);

  // auto oMf = data.oMf;
  ad_vector_t position = frameData.translation();
  ad_vector_t linearVelocity = pinocchio::getFrameVelocity(model, data, frameID_, rf).linear();
  ad_matrix3_t orientation = frameData.rotation();
  ad_vector_t angularVelocity = pinocchio::getFrameVelocity(model, data, frameID_, rf).angular();

  ad_vector_t errors(12);
  errors << (position - reference.getPosition()), rotationMatrixDistanceToPlane<ad_scalar_t>(orientation, reference.getPlaneNormal()),
      (linearVelocity - reference.getLinearVelocity()) * impactProximityScaler, (angularVelocity - reference.getAngularVelocity());

  return errors.cwiseProduct(sqrtWeightParams);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector_t CentroidalMpcEndEffectorFootCost::getParameters(scalar_t time,
                                                         const TargetTrajectories& targetTrajectories,
                                                         const PreComputation& preComputation) const {
  // Interpolate reference
  const vector_t xRef = targetTrajectories.getDesiredState(time);
  const vector_t uRef = targetTrajectories.getDesiredInput(time);

  const scalar_t impactProximityScaler = referenceManagerPtr_->getSwingTrajectoryPlanner()->getImpactProximityFactor(contactIndex_, time);

  // TODO Update this reference for non flat ground in the future
  vector_t parameters(25);
  parameters.head(3) = vector3_t(0.0, 0.0, 0.0);        // Reference position
  parameters.segment(3, 3) = vector3_t(0.0, 0.0, 1.0);  // Ground plane normal
  parameters.segment(6, 3) = vector3_t(0.0, 0.0, 0.0);  // Reference linear velocity
  parameters.segment(9, 3) = vector3_t(0.0, 0.0, 0.0);  // Reference angular velocity
  parameters.segment(12, 12) = sqrtWeights_;            // EndEffectorKinematicsWeights vector element

  parameters[24] = impactProximityScaler;

  return parameters;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
}  // namespace ocs2::humanoid
