#pragma once

#include <robot_model/IDMapBase.h>
#include <robot_model/RobotDescription.h>

#include <functional>
#include <vector>

namespace robot::model {

template <typename T>
class JointIdMap : public IDMapBase<T> {
 public:
  explicit JointIdMap(const RobotDescription& robotDescription) : IDMapBase<T>(robotDescription.getNumJoints()) {}

  JointIdMap() = delete;

  //  Get a vector of joint properties given a vector of joint IDs
  vector_t toVector(std::vector<joint_index_t> jointIds,
                    IDMapExtractor<T, scalar_t> auto valueExtractor,
                    scalar_t defaultValue = std::numeric_limits<scalar_t>::quiet_NaN()) const {
    return this->toEigenVector(jointIds.begin(), jointIds.end(), valueExtractor, defaultValue);
  }

 private:
};

}  // namespace robot::model