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

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/contact/ContactRectangle.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid {

/**
 * Implements the constraint h(t,x,u) >= 0 to constrain the contact moment in the x-y plane.
 */

class HumanoidCostConstraintFactory {
 public:
  HumanoidCostConstraintFactory(const std::string& taskFile,
                                const std::string& referenceFile,
                                const SwitchedModelReferenceManager& referenceManager,
                                const PinocchioInterface& pinocchioInterface,
                                const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,
                                const ModelSettings& modelSettings,
                                bool verbose = false);

  ~HumanoidCostConstraintFactory() = default;
  HumanoidCostConstraintFactory(const HumanoidCostConstraintFactory& other) = delete;

  std::unique_ptr<StateInputCost> getStateInputQuadraticCost() const;

  std::unique_ptr<StateCost> getTerminalCost() const;

  std::unique_ptr<StateCost> getFootCollisionConstraint() const;

  std::unique_ptr<StateCost> getJointLimitsConstraint() const;

  std::unique_ptr<StateInputCost> getContactMomentXYConstraint(size_t contactPointIndex, const std::string& name) const;

  std::unique_ptr<StateInputConstraint> getZeroWrenchConstraint(size_t contactPointIndex) const;

  std::unique_ptr<StateInputCost> getFrictionForceConeConstraint(size_t contactPointIndex) const;

  std::unique_ptr<StateInputCost> getExternalTorqueQuadraticCost(size_t contactPointIndex) const;

 private:
  std::string taskFile_;
  std::string referenceFile_;
  const SwitchedModelReferenceManager* referenceManagerPtr_;
  const PinocchioInterface* pinocchioInterfacePtr_;
  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;
  const MpcRobotModelBase<ad_scalar_t>* mpcRobotModelADPtr_;
  const ModelSettings& modelSettings_;
  const bool verbose_;
};

}  // namespace ocs2::humanoid