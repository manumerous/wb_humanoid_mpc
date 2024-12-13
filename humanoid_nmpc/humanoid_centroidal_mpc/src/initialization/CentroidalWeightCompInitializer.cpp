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

#include "humanoid_centroidal_mpc/initialization/CentroidalWeightCompInitializer.h"

#include "humanoid_centroidal_mpc/dynamics/DynamicsHelperFunctions.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
CentroidalWeightCompInitializer::CentroidalWeightCompInitializer(CentroidalModelInfo info,
                                                                 const SwitchedModelReferenceManager& referenceManager,
                                                                 const CentroidalMpcRobotModel<scalar_t>& mpcRobotModel,
                                                                 bool extendNormalizedMomentum)
    : info_(std::move(info)),
      referenceManagerPtr_(&referenceManager),
      mpcRobotModelPtr_(&mpcRobotModel),
      extendNormalizedMomentum_(extendNormalizedMomentum) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

CentroidalWeightCompInitializer::CentroidalWeightCompInitializer(const CentroidalWeightCompInitializer& rhs)
    : info_(rhs.info_),
      referenceManagerPtr_(rhs.referenceManagerPtr_),
      mpcRobotModelPtr_(rhs.mpcRobotModelPtr_),
      extendNormalizedMomentum_(rhs.extendNormalizedMomentum_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

CentroidalWeightCompInitializer* CentroidalWeightCompInitializer::clone() const {
  return new CentroidalWeightCompInitializer(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void CentroidalWeightCompInitializer::compute(
    scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) {
  const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
  input = weightCompensatingInput(info_, contactFlags, *mpcRobotModelPtr_);
  nextState = state;
  if (!extendNormalizedMomentum_) {
    centroidal_model::getNormalizedMomentum(nextState, info_).setZero();
  }
}

}  // namespace ocs2::humanoid
