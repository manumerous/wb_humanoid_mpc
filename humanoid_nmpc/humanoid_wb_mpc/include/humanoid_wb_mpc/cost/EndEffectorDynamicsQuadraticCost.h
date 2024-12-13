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

#include <ocs2_core/automatic_differentiation/Types.h>
#include <ocs2_core/cost/StateInputGaussNewtonCostAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <pinocchio/algorithm/frames.hpp>

#include <humanoid_common_mpc/common/ModelSettings.h>

#include "humanoid_wb_mpc/common/WBAccelMpcRobotModel.h"
#include "humanoid_wb_mpc/cost/EndEffectorDynamicsCostHelpers.h"
#include "humanoid_wb_mpc/end_effector/EndEffectorDynamics.h"

namespace ocs2::humanoid {

class EndEffectorDynamicsQuadraticCost : public ocs2::StateInputCostGaussNewtonAd {
 public:
  EndEffectorDynamicsQuadraticCost(EndEffectorDynamicsWeights weights,
                                   const PinocchioInterface& pinocchioInterface,
                                   const EndEffectorDynamics<scalar_t>& endEffectorDynamics,
                                   const WBAccelMpcRobotModel<ad_scalar_t>& mpcRobotModel,
                                   std::string endEffectorName,
                                   std::string costName,
                                   const ModelSettings& modelSettings,
                                   size_t n_parameters = 19);

  ~EndEffectorDynamicsQuadraticCost() override = default;
  EndEffectorDynamicsQuadraticCost* clone() const override { return new EndEffectorDynamicsQuadraticCost(*this); }

  virtual vector_t getParameters(scalar_t time,
                                 const TargetTrajectories& targetTrajectories,
                                 const PreComputation& preComputation) const override;

  static EndEffectorDynamicsWeights getWeights(const std::string& taskFile, const std::string prefix, bool verbose = false);

 protected:
  EndEffectorDynamicsQuadraticCost(const EndEffectorDynamicsQuadraticCost& other);

  static EndEffectorDynamicsCostElement<scalar_t> getReferenceCostElement(const vector_t& state,
                                                                          const vector_t& input,
                                                                          const EndEffectorDynamics<scalar_t>& endEffectorDynamics);

  ad_vector_t costVectorFunction(ad_scalar_t time,
                                 const ad_vector_t& state,
                                 const ad_vector_t& input,
                                 const ad_vector_t& parameters) override;

  Eigen::Matrix<ad_scalar_t, 18, 1> sqrtWeights_;

  size_t contactIndex_;
  pinocchio::FrameIndex frameID_;
  PinocchioInterfaceCppAd pinocchioInterfaceCppAd_;
  std::shared_ptr<EndEffectorDynamics<scalar_t>> endEffectorDynamicsPtr_;
  WBAccelMpcRobotModel<ad_scalar_t>* mpcRobotModelPtr_;
};

EndEffectorDynamicsWeights loadWeightsFromFile(const std::string& filename, const std::string& fieldname, bool verbose = true);

}  // namespace ocs2::humanoid
