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

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_wb_mpc/cost/JointTorqueCostCppAd.h"
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

JointTorqueCostCppAd::JointTorqueCostCppAd(const vector_t& weights,
                                           const PinocchioInterface& pinocchioInterface,
                                           const WBAccelMpcRobotModel<ad_scalar_t>& mpcRobotModel,
                                           std::string costName,
                                           const ModelSettings& modelSettings)
    : StateInputCostGaussNewtonAd(),
      sqrtWeights_(weights.cwiseSqrt()),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelPtr_(mpcRobotModel.clone()) {
  assert(weights.size() == mpcRobotModel.getJointDim());
  initialize(mpcRobotModel.getStateDim(), mpcRobotModel.getInputDim(), mpcRobotModel.getJointDim(), costName,
             modelSettings.modelFolderCppAd, modelSettings.recompileLibrariesCppAd);
  std::cout << "Initialized JointTorqueCostCppAd with weights: " << weights.transpose() << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

JointTorqueCostCppAd::JointTorqueCostCppAd(const JointTorqueCostCppAd& other)
    : StateInputCostGaussNewtonAd(other),
      sqrtWeights_(other.sqrtWeights_),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_),
      mpcRobotModelPtr_(other.mpcRobotModelPtr_->clone()) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ad_vector_t JointTorqueCostCppAd::costVectorFunction(ad_scalar_t time,
                                                     const ad_vector_t& state,
                                                     const ad_vector_t& input,
                                                     const ad_vector_t& parameters) {
  return computeJointTorques<ad_scalar_t>(state, input, pinocchioInterfaceCppAd_, *mpcRobotModelPtr_).cwiseProduct(parameters);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
}  // namespace ocs2::humanoid
