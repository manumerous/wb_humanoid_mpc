/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.

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

#include "humanoid_common_mpc/cost/ExternalTorqueQuadraticCostAD.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ExternalTorqueQuadraticCostAD::ExternalTorqueQuadraticCostAD(size_t endEffectorIndex,
                                                             Config config,
                                                             const SwitchedModelReferenceManager& referenceManager,
                                                             const PinocchioInterface& pinocchioInterface,
                                                             const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,
                                                             const ModelSettings& modelSettings)
    : StateInputCostGaussNewtonAd(),
      contactPointIndex_(endEffectorIndex),
      frameID_(pinocchioInterface.getModel().getFrameId(modelSettings.contactNames[endEffectorIndex])),
      n_parameters_(1 + config.weights.size()),
      sqrtWeights_(config.weights.cwiseSqrt()),
      activeJointNames_(config.activeJointNames),
      referenceManagerPtr_(&referenceManager),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelADPtr(mpcRobotModelAD.clone()) {
  assert(config.weights.size() == config.activeJointNames.size());
  std::cout << "Initialized ExternalTorqueQuadraticCostAD with weights: " << config.weights.cwiseSqrt() << std::endl;
  const std::string endEffectorName = modelSettings.contactNames[endEffectorIndex];
  std::cout << "Frame name: " << endEffectorName << std::endl;

  std::cout << "Frame ID: " << frameID_ << std::endl;
  std::cout << "State dim: " << mpcRobotModelADPtr->getStateDim() << std::endl;
  std::cout << "Input dim: " << mpcRobotModelADPtr->getInputDim() << std::endl;
  std::cout << "Parameters dim: " << n_parameters_ << std::endl;

  initialize(mpcRobotModelADPtr->getStateDim(), mpcRobotModelADPtr->getInputDim(), n_parameters_,
             endEffectorName + "_ExternalTorqueQuadraticCost", modelSettings.modelFolderCppAd, modelSettings.recompileLibrariesCppAd,
             modelSettings.verboseCppAd);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ExternalTorqueQuadraticCostAD::ExternalTorqueQuadraticCostAD(const ExternalTorqueQuadraticCostAD& other)
    : StateInputCostGaussNewtonAd(other),
      contactPointIndex_(other.contactPointIndex_),
      frameID_(other.frameID_),
      n_parameters_(other.n_parameters_),
      sqrtWeights_(other.sqrtWeights_),
      activeJointNames_(other.activeJointNames_),
      referenceManagerPtr_(other.referenceManagerPtr_),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_),
      mpcRobotModelADPtr(other.mpcRobotModelADPtr->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

bool ExternalTorqueQuadraticCostAD::isActive(scalar_t time) const {
  if (!isActive_) return false;
  return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

vector_t ExternalTorqueQuadraticCostAD::getParameters(scalar_t time,
                                                      const TargetTrajectories& targetTrajectories,
                                                      const PreComputation& preComputation) const {
  vector_t params(n_parameters_);
  const scalar_t impactProximityScaler = referenceManagerPtr_->getSwingTrajectoryPlanner()->getImpactProximityFactor(
      (1 - contactPointIndex_), time);  // Get impactproximity scaler from swing foot.
  params << sqrtWeights_, impactProximityScaler;
  return params;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ad_vector_t ExternalTorqueQuadraticCostAD::costVectorFunction(ad_scalar_t time,
                                                              const ad_vector_t& state,
                                                              const ad_vector_t& input,
                                                              const ad_vector_t& parameters) {
  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;

  const auto& model = pinocchioInterfaceCppAd_.getModel();
  auto& data = pinocchioInterfaceCppAd_.getData();

  const ad_vector_t q = mpcRobotModelADPtr->getGeneralizedCoordinates(state);
  ad_matrix_t J_ee = ad_matrix_t::Zero(6, mpcRobotModelADPtr->getGenCoordinatesDim());
  pinocchio::computeFrameJacobian(model, data, q, frameID_, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, J_ee);

  ad_vector_t tauExt = J_ee.transpose() * mpcRobotModelADPtr->getContactWrench(input, contactPointIndex_);

  ad_vector_t tauExtActive = ad_vector_t::Zero(sqrtWeights_.size());
  for (size_t i = 0; i < sqrtWeights_.size(); i++) {
    tauExtActive[i] = tauExt[6 + mpcRobotModelADPtr->getJointIndex(activeJointNames_[i])];
  }

  const ad_vector_t sqrtWeightsAD = parameters.head(sqrtWeights_.size());
  const ad_scalar_t midSwingScaler =
      1 - parameters[sqrtWeights_.size()];  // large when in the middle of the swing phase, close when close to impact.

  return tauExtActive.cwiseProduct(sqrtWeightsAD) * midSwingScaler;  // multiply with weights
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ExternalTorqueQuadraticCostAD::Config ExternalTorqueQuadraticCostAD::loadConfigFromFile(const std::string& filename,
                                                                                        const std::string& fieldname,
                                                                                        bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  Config config;

  if (verbose) {
    std::cerr << "\n #### External Torque Quadratic Cost Weights: ";
    std::cerr << "Loading weigths from: " << fieldname;
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadStdVector(filename, fieldname + "activeJointNames", config.activeJointNames, verbose);

  vector_t weights(config.activeJointNames.size());
  loadData::loadEigenMatrix(filename, fieldname + "weights", weights);

  if (verbose) {
    std::cerr << "weights: " << weights.transpose() << "\n";
    std::cerr << " #### =============================================================================\n";
  }

  config.weights = weights;
  assert(config.weights.size() == config.activeJointNames.size());

  return config;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

}  // namespace ocs2::humanoid
