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
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include <pinocchio/algorithm/frames.hpp>

#include <ocs2_centroidal_model/CentroidalModelInfo.h>

#include "humanoid_centroidal_mpc/common/CentroidalMpcRobotModel.h"
#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid {

class ICPCost final : public StateInputCostGaussNewtonAd {
 public:
  ICPCost(const SwitchedModelReferenceManager& referenceManager,
          vector2_t weights,
          const PinocchioInterface& pinocchioInterface,
          const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,
          std::string costName,
          const ModelSettings& modelSettings);

  ~ICPCost() override = default;
  ICPCost* clone() const override { return new ICPCost(*this); }

  vector_t getParameters(scalar_t time, const TargetTrajectories& targetTrajectories, const PreComputation& preComputation) const override;

  bool isActive(scalar_t time) const override {
    if (!isActive_) return false;
    const contact_flag_t contactFlags = referenceManagerPtr_->getContactFlags(time);
    return (contactFlags[0] && contactFlags[1]);
  }

  void setActive(bool active) { isActive_ = active; }
  bool getActive() const { return isActive_; }

  void setWeights(const vector2_t& weights) { sqrtWeights_ = weights.cwiseSqrt(); }
  void getWeights(vector2_t& weights) const { weights = sqrtWeights_.cwiseProduct(sqrtWeights_); }

  static vector2_t getWeights(const std::string& taskFile, const std::string prefix, bool verbose);

 private:
  ICPCost(const ICPCost& other);

  ad_vector_t costVectorFunction(ad_scalar_t time,
                                 const ad_vector_t& state,
                                 const ad_vector_t& input,
                                 const ad_vector_t& parameters) override;

  const SwitchedModelReferenceManager* referenceManagerPtr_;

  vector2_t sqrtWeights_;
  bool isActive_ = true;

  PinocchioInterfaceCppAd pinocchioInterfaceCppAd_;
  std::unique_ptr<MpcRobotModelBase<ad_scalar_t>> mpcRobotModelAdPtr_;
};

}  // namespace ocs2::humanoid
