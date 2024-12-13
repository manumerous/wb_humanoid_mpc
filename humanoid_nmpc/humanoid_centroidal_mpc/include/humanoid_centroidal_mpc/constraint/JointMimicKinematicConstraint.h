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

#include <ocs2_core/constraint/StateInputConstraint.h>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

class JointMimicKinematicConstraint final : public StateInputConstraint {
 public:
  struct Config {
    Config() = delete;
    explicit Config(const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                    std::string parentJointNameParam,
                    std::string childJointNameParam,
                    scalar_t multiplierParam,
                    scalar_t positionGainParam)
        : parentJointName(parentJointNameParam),
          childJointName(childJointNameParam),
          parentJointIndex(mpcRobotModel.getJointIndex(parentJointNameParam)),
          childJointIndex(mpcRobotModel.getJointIndex(childJointNameParam)),
          multiplier(multiplierParam),
          positionGain(positionGainParam) {
      assert(positionGain > 0.0);
    }

    const std::string parentJointName;
    const std::string childJointName;
    const size_t parentJointIndex;
    const size_t childJointIndex;
    const scalar_t multiplier;  // q_child = multiplier* q_parent
    scalar_t positionGain;
  };

  /**
   * @param [in] contactPointIndex : The 3 DoF contact index.
   */
  JointMimicKinematicConstraint(const MpcRobotModelBase<scalar_t>& mpcRobotModel, Config config);

  ~JointMimicKinematicConstraint() override = default;
  JointMimicKinematicConstraint* clone() const override { return new JointMimicKinematicConstraint(*this); }

  bool isActive(scalar_t time) const override;
  void setActive(bool isActive) override { isActive_ = isActive; }
  bool getActive() const override { return isActive_; }
  size_t getNumConstraints(scalar_t time) const override { return 1; }
  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
  VectorFunctionLinearApproximation getLinearApproximation(scalar_t time,
                                                           const vector_t& state,
                                                           const vector_t& input,
                                                           const PreComputation& preComp) const override;

 private:
  JointMimicKinematicConstraint(const JointMimicKinematicConstraint& rhs);

  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;
  const Config config_;

  bool isActive_ = true;
};

}  // namespace ocs2::humanoid
