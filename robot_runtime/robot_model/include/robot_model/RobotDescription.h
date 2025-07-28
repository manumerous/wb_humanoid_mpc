#pragma once

#include <robot_core/Types.h>

#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"

namespace robot::model {

struct JointDescription {
  joint_index_t id;
  scalar_t min_angle = std::numeric_limits<scalar_t>::min();
  scalar_t max_angle = std::numeric_limits<scalar_t>::max();
  scalar_t max_velocity = std::numeric_limits<scalar_t>::max();
  scalar_t max_effort = std::numeric_limits<scalar_t>::max();

  friend std::ostream& operator<<(std::ostream& os, const JointDescription& joint);
};

// A class collecting all the info used during setup of the runtime and robot.
// Since a lot of the std::unordered_map related operations use hashtable
// lookups they are not realtime this should not be used in the main loop of the
// runtime.

class RobotDescription {
 public:
  explicit RobotDescription(const std::string& urdfPath);

  RobotDescription() = delete;

  RobotDescription(const RobotDescription&) = delete;
  RobotDescription& operator=(const RobotDescription&) = delete;
  RobotDescription(RobotDescription&&) = delete;
  RobotDescription& operator=(RobotDescription&&) = delete;

  virtual ~RobotDescription() = default;

  const std::vector<joint_index_t>& getJointIndices() const { return joint_indices; }
  const std::vector<std::string>& getJointNames() const { return joint_names; }

  bool containsJoint(const std::string& jointName) const;

  const JointDescription& getJointDescription(const std::string& jointName) const { return joint_name_description_map_.at(jointName); };

  size_t getNumJoints() const { return joint_name_description_map_.size(); }
  const std::string& getURDFPath() const { return urdf_path_; }
  const std::string getURDFName() const;

  joint_index_t getJointIndex(const std::string& jointName) const { return joint_name_description_map_.at(jointName).id; };
  std::vector<joint_index_t> getJointIndices(const std::vector<std::string>& jointNames) const;

  std::string getJointName(joint_index_t jointIndex) const { return joint_id_name_map_.at(jointIndex); };

  friend std::ostream& operator<<(std::ostream& os, const RobotDescription& robot);

 private:
  const std::string urdf_path_;
  absl::flat_hash_map<std::string, JointDescription> joint_name_description_map_;
  absl::flat_hash_map<joint_index_t, std::string> joint_id_name_map_;  // Duplicates data in memory for easier and faster
                                                                       // index to string mapping.
  std::vector<joint_index_t> joint_indices;
  std::vector<std::string> joint_names;

  // Additional sensors like IMUs can be added here later.
};

}  // namespace robot::model
