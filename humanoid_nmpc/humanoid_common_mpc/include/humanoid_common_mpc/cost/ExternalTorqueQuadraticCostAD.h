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

#pragma once

#include <ocs2_core/cost/StateInputGaussNewtonCostAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <pinocchio/algorithm/frames.hpp>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

#include "humanoid_common_mpc/cost/EndEffectorKinematicCostHelpers.h"

namespace ocs2::humanoid {

class ExternalTorqueQuadraticCostAD : public ocs2::StateInputCostGaussNewtonAd {
 public:
  struct Config {
    std::vector<std::string> activeJointNames;
    vector_t weights;
  };

  ExternalTorqueQuadraticCostAD(size_t endEffectorIndex,
                                Config config,
                                const SwitchedModelReferenceManager& referenceManager,
                                const PinocchioInterface& pinocchioInterface,
                                const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,
                                const ModelSettings& modelSettings);

  ~ExternalTorqueQuadraticCostAD() override = default;
  ExternalTorqueQuadraticCostAD* clone() const override { return new ExternalTorqueQuadraticCostAD(*this); }

  virtual vector_t getParameters(scalar_t time,
                                 const TargetTrajectories& targetTrajectories,
                                 const PreComputation& preComputation) const override;

  bool isActive(scalar_t time) const override;
  void setActive(bool active) { isActive_ = active; }
  bool getActive() const { return isActive_; }

  void getWeights(vector_t& weights) const { weights = sqrtWeights_.cwiseProduct(sqrtWeights_); }
  void setWeights(const vector_t& weights) { sqrtWeights_ = weights.cwiseSqrt(); }

  static ExternalTorqueQuadraticCostAD::Config loadConfigFromFile(const std::string& filename,
                                                                  const std::string& fieldname,
                                                                  bool verbose = true);

 protected:
  ExternalTorqueQuadraticCostAD(const ExternalTorqueQuadraticCostAD& other);

  ad_vector_t costVectorFunction(ad_scalar_t time,
                                 const ad_vector_t& state,
                                 const ad_vector_t& input,
                                 const ad_vector_t& parameters) override;

  const size_t contactPointIndex_;
  const pinocchio::FrameIndex frameID_;
  const size_t n_parameters_;
  vector_t sqrtWeights_;
  const std::vector<std::string> activeJointNames_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
  PinocchioInterfaceCppAd pinocchioInterfaceCppAd_;
  const std::unique_ptr<MpcRobotModelBase<ad_scalar_t>> mpcRobotModelADPtr;
  bool isActive_ = true;
};

}  // namespace ocs2::humanoid
