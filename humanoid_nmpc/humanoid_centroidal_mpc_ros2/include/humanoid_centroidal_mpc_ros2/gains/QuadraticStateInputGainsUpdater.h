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
#include <humanoid_centroidal_mpc_ros2/gains/GainsUpdaterUtils.h>
#include <humanoid_common_mpc/cost/StateInputQuadraticCost.h>

namespace ocs2::humanoid {

class QuadraticStateInputGainsUpdater : public GainsUpdaterInterface {
 public:
  QuadraticStateInputGainsUpdater(const CentroidalMpcRobotModel<scalar_t>& mpcRobotModel,
                                  const ocs2::humanoid::ModelSettings& modelSettings,
                                  std::shared_ptr<GenericGuiInterface> gui)
      : GainsUpdaterInterface(gui), mpcRobotModel_(mpcRobotModel), modelSettings_(modelSettings) {}
  ~QuadraticStateInputGainsUpdater() override = default;

  bool initialize(ocs2::OptimalControlProblem& optimalControlProblem, const std::string& description) override {
    try {
      auto& generic = optimalControlProblem.costPtr->get(description);
      auto& custom = dynamic_cast<ocs2::humanoid::StateInputQuadraticCost&>(generic);

      auto getMatrixEntries = [](std::vector<std::pair<int, int>>& matrixEntries, const ocs2::matrix_t& matrix) -> void {
        matrixEntries.clear();
        const Eigen::SparseMatrix<double> matrix_sparse = matrix.sparseView();
        for (int k = 0; k < matrix_sparse.outerSize(); ++k) {
          for (Eigen::SparseMatrix<double>::InnerIterator it(matrix_sparse, k); it; ++it) {
            matrixEntries.emplace_back(it.row(), it.col());
          }
        }
      };
      ocs2::matrix_t Q, R, P;
      custom.getGains(Q, R, P);

      name_ = description;
      component_ = &custom;
      getMatrixEntries(qIndices_, Q);
      getMatrixEntries(rIndices_, R);

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
      // Get gains
      ocs2::matrix_t Q, R, P;
      component_->getGains(Q, R, P);

      // Draw gui
      auto drawEntries = [&](ocs2::matrix_t& matrix, const std::string& name, const std::vector<std::pair<int, int>>& entries,
                             const std::vector<std::string>& descriptions) -> bool {
        bool triggered = false;
        if (gui_->TreeNode(name.c_str())) {
          for (const auto& [row, col] : entries) {
            if (row == col) {
              if (gui_->InputDouble(descriptions.at(row).c_str(), &matrix(row, col))) triggered = true;
            } else {
              if (gui_->InputDouble(std::string(descriptions.at(row) + "-" + descriptions.at(col)).c_str(), &matrix(row, col)))
                triggered = true;
            }
          }
          gui_->TreePop();
        }
        return triggered;
      };
      static const auto& stateDescriptions = utils::getStateDescriptions(modelSettings_);
      static const auto& inputDescriptions = utils::getInputDescriptions(modelSettings_);

      if (drawEntries(Q, "Q Entries", qIndices_, stateDescriptions)) hasBeenTriggered = true;
      if (drawEntries(R, "R Entries", rIndices_, inputDescriptions)) hasBeenTriggered = true;

      // Set gains
      if (hasBeenTriggered) {
        component_->setGains(Q, R, P);
      }

      gui_->TreePop();
    }
    return hasBeenTriggered;
  }

  void addToMessage(ocs2_ros2_msgs::msg::Gains& msg) override {
    msg.value.emplace_back();
    msg.value.back().name = name_;
    msg.value.back().is_active = true;

    ocs2::matrix_t Q, R, P;
    component_->getGains(Q, R, P);

    auto& data = msg.value.back().values;
    data.reserve(Q.size() + R.size());
    data.insert(data.end(), Q.data(), Q.data() + Q.size());
    data.insert(data.end(), R.data(), R.data() + R.size());
  }

  void setFromMessage(const ocs2_ros2_msgs::msg::IndividualGains& gains) override {
    const int messageSize =
        mpcRobotModel_.getStateDim() * mpcRobotModel_.getStateDim() + mpcRobotModel_.getInputDim() * mpcRobotModel_.getInputDim();
    if (gains.name != name_ || gains.values.size() != messageSize) {
      throw std::runtime_error("[QuadraticStateInputGainsUpdater] Invalid message received!]");
    }

    std::vector<double> data = gains.values;
    const matrix_t Q = Eigen::Map<matrix_t>(data.data(), mpcRobotModel_.getStateDim(), mpcRobotModel_.getStateDim());
    const matrix_t R = Eigen::Map<matrix_t>(data.data() + Q.size(), mpcRobotModel_.getInputDim(), mpcRobotModel_.getInputDim());

    // Update the gains
    component_->setGains(Q, R);
  }

 private:
  ocs2::humanoid::StateInputQuadraticCost* component_;
  std::vector<std::pair<int, int>> qIndices_, rIndices_;  // Keep a list of matrix indices that are non-zero at initialization time
  const CentroidalMpcRobotModel<scalar_t>& mpcRobotModel_;
  const ocs2::humanoid::ModelSettings& modelSettings_;
};

}  // namespace ocs2::humanoid