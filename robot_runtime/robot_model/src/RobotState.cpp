#include <robot_model/RobotState.h>

namespace robot::model {

RobotState::RobotState(const RobotDescription& robotDescription, size_t contactSize)
    : jointStateMap_(robotDescription),
      rootPosition_(vector3_t::Zero()),
      rootLinearVelocity_(vector3_t::Zero()),
      rootOrientation_(quaternion_t::Identity()),
      rootAngularVelocity_(vector3_t::Zero()),
      contactFlags_(contactSize) {
  for (joint_index_t idx : robotDescription.getJointIndices()) {
    jointStateMap_[idx].emplace();
  }

  setConfigurationToZero();

  std::fill(contactFlags_.begin(), contactFlags_.end(), true);  // Assume robot is in contact
}

void RobotState::setJointPosition(size_t jointId, double jointPosition) {
  if (jointStateMap_.inRange(jointId)) {
    auto& opt = jointStateMap_.at(jointId);
    if (opt) {
      opt->position = jointPosition;
    }
  }
}

double RobotState::getJointPosition(size_t jointId) const {
  auto& opt = jointStateMap_.at(jointId);
  if (opt) {
    return opt->position;
  } else {
    throw std::runtime_error("Joint ID not found");
  }
}

void RobotState::setJointVelocity(size_t jointId, double jointVelocity) {
  if (jointStateMap_.inRange(jointId)) {
    auto& opt = jointStateMap_.at(jointId);
    if (opt) {
      opt->velocity = jointVelocity;
    }
  }
}

double RobotState::getJointVelocity(size_t jointId) const {
  auto& opt = jointStateMap_.at(jointId);
  if (opt) {
    return opt->velocity;
  } else {
    throw std::runtime_error("Joint ID not found");
  }
}

void RobotState::setConfigurationToZero() {
  rootPosition_.setZero();
  rootOrientation_.setIdentity();
  rootLinearVelocity_.setZero();
  rootAngularVelocity_.setZero();

  for (auto& joint : jointStateMap_) {
    joint.position = 0.0;
    joint.velocity = 0.0;
    joint.measuredEffort = 0.0;
  }

  std::fill(contactFlags_.begin(), contactFlags_.end(), true);  // Assume robot is in contact
}

}  // namespace robot::model