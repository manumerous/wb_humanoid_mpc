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

#include <ocs2_core/cost/StateInputGaussNewtonCostAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <pinocchio/algorithm/frames.hpp>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"

#include "humanoid_common_mpc/cost/EndEffectorKinematicCostHelpers.h"

namespace ocs2::humanoid {

class EndEffectorKinematicsQuadraticCost : public ocs2::StateInputCostGaussNewtonAd {
 public:
  EndEffectorKinematicsQuadraticCost(EndEffectorKinematicsWeights weights,
                                     const PinocchioInterface& pinocchioInterface,
                                     const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                     const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,
                                     std::string endEffectorName,
                                     const ModelSettings& modelSettings);

  ~EndEffectorKinematicsQuadraticCost() override = default;
  EndEffectorKinematicsQuadraticCost* clone() const override { return new EndEffectorKinematicsQuadraticCost(*this); }

  virtual vector_t getParameters(scalar_t time,
                                 const TargetTrajectories& targetTrajectories,
                                 const PreComputation& preComputation) const override;

  bool isActive(scalar_t time) const override { return isActive_; }
  void setActive(bool active) { isActive_ = active; }
  bool getActive() const { return isActive_; }

  static EndEffectorKinematicsWeights getWeights(const std::string& taskFile, const std::string prefix, bool verbose = false);

  void getWeights(vector12_t& weights) const { weights = sqrtWeights_.cwiseProduct(sqrtWeights_); }
  void setWeights(const vector12_t& weights) { sqrtWeights_ = weights.cwiseSqrt(); }

 protected:
  EndEffectorKinematicsQuadraticCost(const EndEffectorKinematicsQuadraticCost& other);

  static EndEffectorKinematicsCostElement<scalar_t> getReferenceCostElement(const vector_t& state,
                                                                            const vector_t& input,
                                                                            const EndEffectorKinematics<scalar_t>& endEffectorKinematics);

  ad_vector_t costVectorFunction(ad_scalar_t time,
                                 const ad_vector_t& state,
                                 const ad_vector_t& input,
                                 const ad_vector_t& parameters) override;

  vector12_t sqrtWeights_;
  size_t n_parameters_ = 25;
  pinocchio::FrameIndex frameID_;
  PinocchioInterfaceCppAd pinocchioInterfaceCppAd_;
  const std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
  std::unique_ptr<MpcRobotModelBase<ad_scalar_t>> mpcRobotModelADPtr;
  bool isActive_ = true;
};

EndEffectorKinematicsWeights loadWeightsFromFile(const std::string& filename, const std::string& fieldname, bool verbose = true);

}  // namespace ocs2::humanoid
