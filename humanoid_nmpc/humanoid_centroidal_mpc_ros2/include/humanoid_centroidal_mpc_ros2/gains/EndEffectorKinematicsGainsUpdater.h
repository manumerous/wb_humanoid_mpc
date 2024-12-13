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
#include <humanoid_common_mpc/cost/EndEffectorKinematicsQuadraticCost.h>

namespace ocs2::humanoid {

class EndEffectorKinematicsGainsUpdater : public GainsUpdaterInterface {
 public:
  EndEffectorKinematicsGainsUpdater(std::shared_ptr<GenericGuiInterface> gui) : GainsUpdaterInterface(gui) {}
  ~EndEffectorKinematicsGainsUpdater() override = default;

  bool initialize(ocs2::OptimalControlProblem& optimalControlProblem, const std::string& description) override {
    try {
      auto& generic = optimalControlProblem.costPtr->get(description);
      auto& custom = dynamic_cast<ocs2::humanoid::EndEffectorKinematicsQuadraticCost&>(generic);

      name_ = description;
      component_ = &custom;

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
      ocs2::vector12_t weights;
      component_->getWeights(weights);

      // Get descriptions
      static const auto& descriptions = ocs2::humanoid::EndEffectorKinematicsWeights::getDescriptions();
      if (descriptions.size() != weights.size()) {
        throw std::runtime_error("[EndEffectorKinematicsGainsUpdater] Invalid number of descriptions!");
      }

      // Draw gui
      if (gui_->Checkbox("Active", &active)) hasBeenTriggered = true;
      if (active) {
        for (auto i = 0; i < weights.size(); i++) {
          if (gui_->InputDouble(descriptions.at(i).c_str(), &weights(i))) hasBeenTriggered = true;
        }
      }

      // Update gains
      if (hasBeenTriggered) {
        component_->setActive(active);
        component_->setWeights(weights);
      }

      gui_->TreePop();
    }
    return hasBeenTriggered;
  }

  void addToMessage(ocs2_ros2_msgs::msg::Gains& msg) override {
    msg.value.emplace_back();
    msg.value.back().name = name_;
    msg.value.back().is_active = component_->getActive();

    ocs2::vector12_t weights;
    component_->getWeights(weights);

    auto& data = msg.value.back().values;
    data.reserve(weights.size());
    data.insert(data.end(), weights.data(), weights.data() + weights.size());
  }

  void setFromMessage(const ocs2_ros2_msgs::msg::IndividualGains& gains) override {
    if (gains.name != name_ || gains.values.size() != 12) {
      throw std::runtime_error("[EndEffectorKinematicsGainsUpdater] Invalid message received!");
    }

    component_->setActive(gains.is_active);
    std::vector<double> data = gains.values;
    const vector12_t weights = Eigen::Map<vector12_t>(data.data(), 12, 1);
    component_->setWeights(weights);
  }

 private:
  ocs2::humanoid::EndEffectorKinematicsQuadraticCost* component_;
};

}  // namespace ocs2::humanoid