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

#include "humanoid_centroidal_mpc/cost/ICPCost.h"

#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>

#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include <cmath>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ICPCost::ICPCost(const SwitchedModelReferenceManager& referenceManager,
                 vector2_t weights,
                 const PinocchioInterface& pinocchioInterface,
                 const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,
                 std::string costName,
                 const ModelSettings& modelSettings)
    : StateInputCostGaussNewtonAd(),
      referenceManagerPtr_(&referenceManager),
      sqrtWeights_(weights.cwiseSqrt()),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelAdPtr_(mpcRobotModelAD.clone()) {
  initialize(mpcRobotModelAD.getStateDim(), mpcRobotModelAD.getInputDim(), 2, costName, modelSettings.modelFolderCppAd,
             modelSettings.recompileLibrariesCppAd);
  std::cout << "Initialized ICPCost with weights: " << weights.transpose() << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ICPCost::ICPCost(const ICPCost& other)
    : StateInputCostGaussNewtonAd(other),
      referenceManagerPtr_(other.referenceManagerPtr_),
      sqrtWeights_(other.sqrtWeights_),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_),
      mpcRobotModelAdPtr_(other.mpcRobotModelAdPtr_->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ad_vector_t ICPCost::costVectorFunction(ad_scalar_t time,
                                        const ad_vector_t& state,
                                        const ad_vector_t& input,
                                        const ad_vector_t& parameters) {
  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;
  const ad_vector_t sqrtWeightParams = parameters.head(2);  // EndEffectorKinematicsWeights vector element

  const auto& model = pinocchioInterfaceCppAd_.getModel();
  auto& data = pinocchioInterfaceCppAd_.getData();
  scalar_t omega = std::sqrt(9.81 / 0.7);  // sqrt(g / z_0) This default com height should be added from the config.

  const ad_vector_t q = mpcRobotModelAdPtr_->getGeneralizedCoordinates(state);

  pinocchio::centerOfMass(model, data, q, false);
  ad_vector2_t com = data.com[0].head(2);

  pinocchio::updateFramePlacements(model, data);
  auto contactPositions = getContactPositions<ad_scalar_t>(pinocchioInterfaceCppAd_, *mpcRobotModelAdPtr_);
  ad_vector2_t desiredCOMPosition = (contactPositions[0] + contactPositions[1]).head(2) / ad_scalar_t(2.0);

  ad_vector_t com_vel(2);
  com_vel[0] = state[0];
  com_vel[1] = state[1];

  ad_vector_t capturePoint = com;
  // ad_vector_t capturePoint = com + com_vel / ad_scalar_t(omega);
  ad_vector_t errors = desiredCOMPosition - capturePoint;

  return errors.cwiseProduct(sqrtWeightParams);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector_t ICPCost::getParameters(scalar_t time, const TargetTrajectories& targetTrajectories, const PreComputation& preComputation) const {
  // TODO Update this reference for non flat ground in the future
  vector_t parameters = sqrtWeights_;  // EndEffectorKinematicsWeights vector element

  return parameters;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector2_t ICPCost::getWeights(const std::string& taskFile, const std::string prefix, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);

  // Load all weights
  scalar_t icpErrorWeight = 0;

  if (verbose) {
    std::cerr << "\n #### ICP Cost Weights: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, icpErrorWeight, prefix + "icpErrorWeight", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  vector2_t weights;
  weights << icpErrorWeight, icpErrorWeight;

  return weights;
}
}  // namespace ocs2::humanoid
