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

#include "humanoid_common_mpc_ros2/gait/GaitScheduleUpdaterRos2.h"

#include "humanoid_common_mpc_ros2/gait/ModeSequenceTemplateRos.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
GaitScheduleUpdaterRos2::GaitScheduleUpdaterRos2(rclcpp::Node::SharedPtr& nodeHandle,
                                                 std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                 const std::string& robotName)
    : GaitScheduleUpdater(gaitSchedulePtr), gaitUpdatedAtomic_(false) {
  auto qos = rclcpp::QoS(1);
  qos.best_effort();
  mpcModeSequenceSubscriber_ = nodeHandle->create_subscription<ocs2_ros2_msgs::msg::ModeSchedule>(
      robotName + "_mpc_mode_schedule", qos, std::bind(&GaitScheduleUpdaterRos2::mpcModeSequenceCallback, this, std::placeholders::_1));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void GaitScheduleUpdaterRos2::preSolverRun(scalar_t initTime,
                                           scalar_t finalTime,
                                           const vector_t& currentState,
                                           const ReferenceManagerInterface& referenceManager) {
  if (gaitUpdatedAtomic_.load()) {
    updateGaitSchedule(gaitSchedulePtr_, getReceivedGait(), initTime, finalTime);
    gaitUpdatedAtomic_.store(false);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ModeSequenceTemplate GaitScheduleUpdaterRos2::getReceivedGait() {
  std::lock_guard<std::mutex> lock(receivedGaitMutex_);
  return receivedGait_;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void GaitScheduleUpdaterRos2::mpcModeSequenceCallback(const ocs2_ros2_msgs::msg::ModeSchedule::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(receivedGaitMutex_);
  updateModeSequence(readModeSequenceTemplateMsg(*msg));
  gaitUpdatedAtomic_.store(true);
}

}  // namespace ocs2::humanoid
