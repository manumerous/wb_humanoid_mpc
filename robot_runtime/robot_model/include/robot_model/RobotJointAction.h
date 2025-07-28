#pragma once
#include <robot_core/Types.h>
#include <robot_model/JointIDMap.h>
#include <robot_model/RobotDescription.h>

namespace robot::model {

struct JointAction {
  scalar_t q_des = 0;
  scalar_t qd_des = 0;
  scalar_t kp = 0;
  scalar_t kd = 0;
  scalar_t feed_forward_effort = 0;

  scalar_t getTotalFeedbackTorque(scalar_t q, scalar_t qd) const { return (kp * (q_des - q) + kd * (qd_des - qd) + feed_forward_effort); }
};

class RobotJointAction : public JointIdMap<JointAction> {
 public:
  explicit RobotJointAction(const RobotDescription& robotDescription) : JointIdMap(robotDescription) {
    for (joint_index_t idx : robotDescription.getJointIndices()) {
      this->at(idx).emplace();
    }
  }
};

}  // namespace robot::model
