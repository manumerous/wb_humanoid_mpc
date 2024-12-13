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

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/Types.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <stdexcept>

#include <ocs2_core/misc/LoadData.h>

namespace ocs2::humanoid {

template <typename SCALAR_T>
class MpcRobotModelBase {
 public:
  MpcRobotModelBase(const ModelSettings& modelSettings, scalar_t state_dim, scalar_t input_dim)
      : modelSettings(modelSettings),
        state_dim(state_dim),
        input_dim(input_dim),
        base_dim(6),
        gen_coordinates_dim(base_dim + modelSettings.mpc_joint_dim){};

  virtual ~MpcRobotModelBase() = default;
  virtual MpcRobotModelBase* clone() const = 0;

  /******************************************************************************************************/
  /*                                           Dimensions                                               */
  /******************************************************************************************************/

  size_t getStateDim() const { return state_dim; };
  size_t getInputDim() const { return input_dim; };
  size_t getBaseDim() const { return base_dim; };
  size_t getJointDim() const { return modelSettings.mpc_joint_dim; };
  size_t getFullModelJointDim() const { return modelSettings.full_joint_dim; };
  size_t getGenCoordinatesDim() const { return gen_coordinates_dim; };

  /******************************************************************************************************/
  /*                                          Start indices                                             */
  /******************************************************************************************************/

  virtual size_t getBaseStartindex() const = 0;
  virtual size_t getJointStartindex() const = 0;
  virtual size_t getJointVelocitiesStartindex() const = 0;

  // Assumes contact wrench [f_x, f_y, f_z, M_x, M_y, M_z]^T
  virtual size_t getContactWrenchStartIndices(size_t contactIndex) const = 0;
  virtual size_t getContactForceStartIndices(size_t contactIndex) const { return getContactWrenchStartIndices(contactIndex); }
  virtual size_t getContactMomentStartIndices(size_t contactIndex) const { return 6 * contactIndex + 3; }

  /******************************************************************************************************/
  /*                                     Generalized coordinates                                        */
  /******************************************************************************************************/

  virtual VECTOR_T<SCALAR_T> getGeneralizedCoordinates(const VECTOR_T<SCALAR_T>& state) const = 0;

  virtual VECTOR6_T<SCALAR_T> getBasePose(const VECTOR_T<SCALAR_T>& state) const = 0;

  virtual VECTOR3_T<SCALAR_T> getBasePosition(const VECTOR_T<SCALAR_T>& state) const = 0;

  virtual VECTOR3_T<SCALAR_T> getBaseOrientationEulerZYX(const VECTOR_T<SCALAR_T>& state) const = 0;

  virtual VECTOR3_T<SCALAR_T> getBaseComLinearVelocity(const VECTOR_T<SCALAR_T>& state) const = 0;

  virtual VECTOR6_T<SCALAR_T> getBaseComVelocity(const VECTOR_T<SCALAR_T>& state) const = 0;

  virtual VECTOR_T<SCALAR_T> getJointAngles(const VECTOR_T<SCALAR_T>& state) const = 0;

  virtual VECTOR_T<SCALAR_T> getJointVelocities(const VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& input) const = 0;

  virtual VECTOR_T<SCALAR_T> getGeneralizedVelocities(const VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& input) = 0;

  /******************************************************************************************************/

  virtual void setGeneralizedCoordinates(VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& generalizedCorrdinates) const = 0;

  virtual void setBasePose(VECTOR_T<SCALAR_T>& state, const VECTOR6_T<SCALAR_T>& basePose) const = 0;

  virtual void setBasePosition(VECTOR_T<SCALAR_T>& state, const VECTOR3_T<SCALAR_T>& position) const = 0;

  virtual void setBaseOrientationEulerZYX(VECTOR_T<SCALAR_T>& state, const VECTOR3_T<SCALAR_T>& eulerAnglesZYX) const = 0;

  virtual void setJointAngles(VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& jointAngles) const = 0;

  virtual void setJointVelocities(VECTOR_T<SCALAR_T>& state,
                                  VECTOR_T<SCALAR_T>& input,
                                  const VECTOR_T<SCALAR_T>& jointVelocities) const = 0;

  virtual void adaptBasePoseHeight(VECTOR_T<SCALAR_T>& state, scalar_t heightChange) const = 0;

  /******************************************************************************************************/
  /*                                          Contacts                                                  */
  /******************************************************************************************************/

  virtual VECTOR6_T<SCALAR_T> getContactWrench(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const = 0;

  virtual VECTOR3_T<SCALAR_T> getContactForce(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const = 0;

  virtual VECTOR3_T<SCALAR_T> getContactMoment(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const = 0;

  /******************************************************************************************************/

  virtual void setContactWrench(VECTOR_T<SCALAR_T>& input, const VECTOR6_T<SCALAR_T>& wrench, size_t contactIndex) const = 0;

  virtual void setContactForce(VECTOR_T<SCALAR_T>& input, const VECTOR3_T<SCALAR_T>& force, size_t contactIndex) const = 0;

  virtual void setContactMoment(VECTOR_T<SCALAR_T>& input, const VECTOR3_T<SCALAR_T>& moment, size_t contactIndex) const = 0;

  /******************************************************************************************************/
  /*                                         Joint angle                                                */
  /******************************************************************************************************/

  size_t getJointIndex(const std::string& jointName) const {
    auto it = modelSettings.jointIndexMap.find(jointName);
    if (it != modelSettings.jointIndexMap.end()) {
      return it->second;  // Return the found index
    } else {
      throw std::runtime_error("Joint name " + jointName + " is not contained in MPC model!");
    }
  }

  VECTOR_T<SCALAR_T> getFullModelJointAngles(const VECTOR_T<SCALAR_T>& mpcModelJointAngles,
                                             const VECTOR_T<SCALAR_T>& defaultFullModelJointAngles) const {
    VECTOR_T<SCALAR_T> fullModelJointAngles = VECTOR_T<SCALAR_T>(defaultFullModelJointAngles);
    assert(mpcModelJointAngles.size() == modelSettings.mpc_joint_dim);
    assert(defaultFullModelJointAngles.size() == modelSettings.full_joint_dim);
    for (size_t i = 0; i < modelSettings.mpc_joint_dim; ++i) {
      size_t currJointFullIndex = modelSettings.mpcModelToFullJointsIndices[i];
      fullModelJointAngles[currJointFullIndex] = mpcModelJointAngles[i];
    }
    return fullModelJointAngles;
  }

  VECTOR_T<SCALAR_T> getMpcModelJointAngles(const VECTOR_T<SCALAR_T>& fullModelJointAngles) const {
    VECTOR_T<SCALAR_T> mpcModelJointAngles(modelSettings.mpc_joint_dim);
    assert(fullModelJointAngles.size() == modelSettings.full_joint_dim);
    for (size_t i = 0; i < modelSettings.mpc_joint_dim; ++i) {
      mpcModelJointAngles[i] = fullModelJointAngles[modelSettings.mpcModelToFullJointsIndices[i]];
    }
    return mpcModelJointAngles;
  }

 protected:
  MpcRobotModelBase(const MpcRobotModelBase& rhs)
      : modelSettings(rhs.modelSettings),
        state_dim(rhs.state_dim),
        input_dim(rhs.input_dim),
        base_dim(6),
        gen_coordinates_dim(rhs.gen_coordinates_dim){};

 public:
  const ModelSettings& modelSettings;

  const size_t state_dim;
  const size_t input_dim;
  const size_t base_dim;
  const size_t gen_coordinates_dim;
};

}  // namespace ocs2::humanoid
