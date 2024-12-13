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

#include <humanoid_centroidal_mpc_ros2/gains/GainsUpdaterInterface.h>
#include <humanoid_common_mpc/constraint/FootCollisionConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>

namespace ocs2::humanoid {

class FootCollisionGainsUpdater : public GainsUpdaterInterface {
 public:
  FootCollisionGainsUpdater(std::shared_ptr<GenericGuiInterface> gui) : GainsUpdaterInterface(gui) {}
  ~FootCollisionGainsUpdater() override = default;

  bool initialize(OptimalControlProblem& optimalControlProblem, const std::string& description) override {
    try {
      auto& generic = optimalControlProblem.stateSoftConstraintPtr->get(description);
      auto& custom = dynamic_cast<StateSoftConstraint&>(generic);
      auto& penaltyPtrArray = custom.getPenalty().getPenaltyPtrArray();
      if (penaltyPtrArray.size() != 1) {
        throw std::runtime_error("[FootCollisionGainsUpdater] Unexpected number of penalties!");
      }
      auto& constraint = dynamic_cast<FootCollisionConstraint&>(custom.get());

      name_ = description;
      component_ = &constraint;
      penalty_ = penaltyPtrArray.at(0).get();

      return true;
    } catch (const std::bad_cast& e) {
      // Nothing to do here
    } catch (const std::out_of_range& e) {
      // Nothing to do here
    }
    return false;
  }

  bool drawGui() override {
    if (!gui_) return false;

    bool hasBeenTriggered = false;
    if (gui_->TreeNode(name_.c_str())) {
      // Get parameters
      bool active = component_->getActive();
      vector_t parameters;
      penalty_->getParameters(parameters);
      if (parameters.size() != 2) {
        throw std::runtime_error("[StateInputSoftConstraintGainsUpdater] Unexpected number of parameters!");
      }

      scalar_t footCollisionSphereRadius, kneeCollisionSphereRadius;
      component_->getSphereRadii(footCollisionSphereRadius, kneeCollisionSphereRadius);

      // Draw gui
      if (gui_->Checkbox("Active", &active)) hasBeenTriggered = true;
      if (active) {
        if (gui_->InputDouble("Penalty - mu", &parameters[0])) hasBeenTriggered = true;
        if (gui_->InputDouble("Penalty - delta", &parameters[1])) hasBeenTriggered = true;
        if (gui_->InputDouble("Sphere Radius - Foot", &footCollisionSphereRadius)) hasBeenTriggered = true;
        if (gui_->InputDouble("Sphere Radius - Knee", &kneeCollisionSphereRadius)) hasBeenTriggered = true;
      }

      // Update parameters
      if (hasBeenTriggered) {
        component_->setActive(active);
        penalty_->setParameters(parameters);
        component_->setSphereRadii(footCollisionSphereRadius, kneeCollisionSphereRadius);
      }

      gui_->TreePop();
    }
    return hasBeenTriggered;
  }

  void addToMessage(ocs2_ros2_msgs::msg::Gains& msg) override {
    msg.value.emplace_back();
    msg.value.back().name = name_;
    msg.value.back().is_active = component_->getActive();

    vector_t parameters;
    penalty_->getParameters(parameters);
    if (parameters.size() != 2) {
      throw std::runtime_error("[StateInputSoftConstraintGainsUpdater] Unexpected number of parameters!");
    }

    scalar_t footCollisionSphereRadius, kneeCollisionSphereRadius;
    component_->getSphereRadii(footCollisionSphereRadius, kneeCollisionSphereRadius);

    auto& data = msg.value.back().values;
    data.reserve(4);
    data.emplace_back(parameters[0]);
    data.emplace_back(parameters[1]);
    data.push_back(footCollisionSphereRadius);
    data.push_back(kneeCollisionSphereRadius);
  }

  void setFromMessage(const ocs2_ros2_msgs::msg::IndividualGains& gains) override {
    if (gains.values.size() != 4) {
      throw std::runtime_error("[FootCollisionGainsUpdater] Invalid number of values!");
    }
    component_->setActive(gains.is_active);
    vector_t parameters(2);
    parameters[0] = gains.values[0];
    parameters[1] = gains.values[1];
    penalty_->setParameters(parameters);
    component_->setSphereRadii(gains.values[2], gains.values[3]);
  }

 private:
  FootCollisionConstraint* component_;
  ocs2::augmented::AugmentedPenaltyBase* penalty_;
};

}  // namespace ocs2::humanoid