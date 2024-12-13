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
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>

namespace ocs2::humanoid {

class StateInputSoftConstraintGainsUpdater : public GainsUpdaterInterface {
 public:
  StateInputSoftConstraintGainsUpdater(std::shared_ptr<GenericGuiInterface> gui) : GainsUpdaterInterface(gui) {}
  ~StateInputSoftConstraintGainsUpdater() override = default;

  bool initialize(ocs2::OptimalControlProblem& optimalControlProblem, const std::string& description) override {
    try {
      auto& generic = optimalControlProblem.softConstraintPtr->get(description);
      auto& custom = dynamic_cast<ocs2::StateInputSoftConstraint&>(generic);
      auto& penaltyPtrArray = custom.getPenalty().getPenaltyPtrArray();
      if (penaltyPtrArray.size() != 1) {
        throw std::runtime_error("[StateInputSoftConstraintGainsUpdater] Unexpected number of penalties!");
      }
      auto& constraint = custom.getConstraintPtr();

      constraint_ = constraint.get();
      penalty_ = penaltyPtrArray.at(0).get();
      name_ = description;

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
      bool active = constraint_->getActive();
      vector_t parameters;
      penalty_->getParameters(parameters);
      if (parameters.size() != 2) {
        throw std::runtime_error("[StateInputSoftConstraintGainsUpdater] Unexpected number of parameters!");
      }

      // Draw gui
      if (gui_->Checkbox("Active", &active)) hasBeenTriggered = true;
      if (active) {
        if (gui_->InputDouble("mu", &parameters[0])) hasBeenTriggered = true;
        if (gui_->InputDouble("delta", &parameters[1])) hasBeenTriggered = true;
      }

      // Update gains
      if (hasBeenTriggered) {
        penalty_->setParameters(parameters);
        constraint_->setActive(active);
      }

      gui_->TreePop();
    }
    return hasBeenTriggered;
  }

  void addToMessage(ocs2_ros2_msgs::msg::Gains& msg) override {
    msg.value.emplace_back();
    msg.value.back().name = name_;
    msg.value.back().is_active = constraint_->getActive();

    vector_t parameters;
    penalty_->getParameters(parameters);
    if (parameters.size() != 2) {
      throw std::runtime_error("[StateInputSoftConstraintGainsUpdater] Unexpected number of parameters!");
    }
    auto& data = msg.value.back().values;
    data.reserve(2);
    data.emplace_back(parameters[0]);
    data.emplace_back(parameters[1]);
  }

  void setFromMessage(const ocs2_ros2_msgs::msg::IndividualGains& gains) override {
    if (gains.values.size() != 2) {
      throw std::runtime_error("[StateInputSoftConstraintGainsUpdater] Unexpected number of parameters!");
    }
    vector_t parameters(2);
    parameters[0] = gains.values[0];
    parameters[1] = gains.values[1];
    penalty_->setParameters(parameters);
    constraint_->setActive(gains.is_active);
  }

 private:
  StateInputConstraint* constraint_;
  ocs2::augmented::AugmentedPenaltyBase* penalty_;
};

}  // namespace ocs2::humanoid