#pragma once

#include <robot_core/ThreadSafe.h>
#include <robot_model/RobotJointAction.h>
#include <robot_model/RobotState.h>

#include "robot_model/RobotDescription.h"

namespace robot::model {

// Provides  apublic interface of how to interact with the robot.

class RobotHWInterfaceBase {
 public:
  RobotHWInterfaceBase(const std::string& urdfPath)
      : robotDescription_(urdfPath),
        robotState_((model::RobotState(robotDescription_))),
        robotJointAction_(model::RobotJointAction(robotDescription_)),
        threadSafeRobotState_(robotState_),
        threadSafeRobotJointAction_(robotJointAction_){};

  const model::RobotDescription& getRobotDescription() const { return robotDescription_; }

  // Get Access to a read only version of the robot state
  const RobotState& getRobotState() const { return robotState_; }

  // Update the internal robot state
  void updateInterfaceStateFromRobot() { threadSafeRobotState_.copy_value(robotState_); }

  // Get a reference to fill in the updated joint control action
  RobotJointAction& getRobotJointAction() { return robotJointAction_; }

  // Send the new action to the robot.
  void applyJointAction() { threadSafeRobotJointAction_.set(robotJointAction_); }

 private:
  const RobotDescription robotDescription_;
  RobotState robotState_;
  RobotJointAction robotJointAction_;

 protected:
  ThreadSafe<RobotState> threadSafeRobotState_;
  ThreadSafe<RobotJointAction> threadSafeRobotJointAction_;
};

}  // namespace robot::model
