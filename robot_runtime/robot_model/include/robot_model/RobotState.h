#pragma once
#include <robot_core/Types.h>
#include <robot_model/JointIDMap.h>
#include <robot_model/RobotDescription.h>

namespace robot::model {

struct JointState {
  scalar_t position = 0.0;
  scalar_t velocity = 0.0;
  scalar_t measuredEffort = 0.0;

  JointState() = default;
};

class RobotState {
 public:
  RobotState(const RobotDescription& robotDescription, size_t contactSize = 2);

  // orientation of the root joint wrt world frame, corresponds to the passive
  // rotation from local to world R_l_to_w
  quaternion_t getRootRotationLocalToWorldFrame() const { return rootOrientation_; }
  vector3_t getRootPositionInWorldFrame() const { return rootPosition_; }

  vector3_t getRootLinearVelocityInLocalFrame() const { return rootLinearVelocity_; }
  vector3_t getRootAngularVelocityInLocalFrame() const { return rootAngularVelocity_; }

  // orientation of the root joint wrt world frame, corresponds to the passive
  // rotation from local to world R_l_to_w
  void setRootRotationLocalToWorldFrame(const quaternion_t& orientation) { rootOrientation_ = orientation; }
  void setRootPositionInWorldFrame(const vector3_t& position) { rootPosition_ = position; }

  void setRootLinearVelocityInLocalFrame(const vector3_t& linearVelocity) { rootLinearVelocity_ = linearVelocity; }
  void setRootAngularVelocityInLocalFrame(const vector3_t& angularVelocity) { rootAngularVelocity_ = angularVelocity; }

  void setJointPosition(size_t jointId, scalar_t jointPosition);
  scalar_t getJointPosition(size_t jointId) const;
  void setJointVelocity(size_t jointId, scalar_t jointVelocity);
  scalar_t getJointVelocity(size_t jointId) const;

  //  Get a vector_t of joint positions given a vector of joint IDs
  template <typename E>
  vector_t getJointPositions(std::vector<E> jointIds, scalar_t defaultValue = std::numeric_limits<scalar_t>::quiet_NaN()) const {
    return jointStateMap_.toVector(jointIds, [](const JointState& js) { return js.position; }, defaultValue);
  }

  //  Get a vector_t of joint velocities given a vector of joint IDs
  template <typename E>
  vector_t getJointVelocities(std::vector<E> jointIds, scalar_t defaultValue = std::numeric_limits<scalar_t>::quiet_NaN()) const {
    return jointStateMap_.toVector(jointIds, [](const JointState& js) { return js.velocity; }, defaultValue);
  }

  bool getContactFlag(size_t index) const { return contactFlags_.at(index); }

  void setContactFlag(size_t index, bool contactFlag) { contactFlags_.at(index) = contactFlag; }

  std::vector<bool> getContactFlags() const { return contactFlags_; }

  scalar_t getTime() const { return time_; }

  void setTime(scalar_t time) { time_ = time; }

  void setConfigurationToZero();

 private:
  JointIdMap<JointState> jointStateMap_;

  scalar_t time_;

  vector3_t rootPosition_;
  vector3_t rootLinearVelocity_;
  quaternion_t rootOrientation_;
  vector3_t rootAngularVelocity_;
  std::vector<bool> contactFlags_;
};

}  // namespace robot::model
