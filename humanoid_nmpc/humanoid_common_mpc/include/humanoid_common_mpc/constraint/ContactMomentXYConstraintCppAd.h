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

#include <ocs2_core/constraint/StateInputConstraintCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/contact/ContactRectangle.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid {

/**
 * Implements the constraint h(t,x,u) >= 0 to constrain the contact moment in the x-y plane.
 */

class ContactMomentXYConstraintCppAd final : public StateInputConstraintCppAd {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ContactMomentXYConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                 const ContactRectangle& contactRectangle,
                                 size_t contactPointIndex,
                                 const PinocchioInterface& pinocchioInterface,
                                 const MpcRobotModelBase<ad_scalar_t>& mpcRobotModel,
                                 std::string costName,
                                 const ModelSettings& modelSettings);

  ~ContactMomentXYConstraintCppAd() override = default;
  ContactMomentXYConstraintCppAd* clone() const override { return new ContactMomentXYConstraintCppAd(*this); }

  bool isActive(scalar_t time) const override;
  void setActive(bool isActive) override { isActive_ = isActive; }
  bool getActive() const override { return isActive_; }
  size_t getNumConstraints(scalar_t time) const override { return numConstraints_; };

 private:
  ContactMomentXYConstraintCppAd(const ContactMomentXYConstraintCppAd& other);

  ad_vector_t constraintFunction(ad_scalar_t time,
                                 const ad_vector_t& state,
                                 const ad_vector_t& input,
                                 const ad_vector_t& parameters) const override;

  const SwitchedModelReferenceManager* referenceManagerPtr_;
  const MpcRobotModelBase<ad_scalar_t>* mpcRobotModelPtr_;
  const ContactRectangle contactRectangle_;
  const size_t contactPointIndex_;
  PinocchioInterfaceCppAd pinocchioInterfaceCppAd_;

  const static size_t numConstraints_ = 4;
  bool isActive_ = true;
};

}  // namespace ocs2::humanoid