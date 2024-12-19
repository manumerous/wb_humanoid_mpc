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
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

/******************************************************************************************************/
/* State Vector definition

  Define the state vector x = [q_b_lin q_b_ang, q_j, qd_b_lin qd_b_ang, qd_j]^T

    Base linear position = [p_base_x, p_base_y, p_base_z] in world frame
    Base euler angles = [euler_z, euler_y, euler_x] from world to local frame?
    Joint angles q_j
    Base linear velocity = [v_base_x, v_base_y, v_base_z] in world frame
    Base euler angle derivatives = [euler_d_z, euler_d_y, euler_D_x]
    Joint velocities qd_j

*/
/******************************************************************************************************/

/******************************************************************************************************/
/* Input Vector definition

  Define the input vector u = [W_l, W_r, qdd_j]^T

    Left contact Wrench W_l in inertial frame
    Right contact Wrench W_r in inertial frame
    Joint accelerations qdd_j

*/
/******************************************************************************************************/

template <typename SCALAR_T>
class WBAccelMpcRobotModel : public MpcRobotModelBase<SCALAR_T> {
 public:
  WBAccelMpcRobotModel(const ModelSettings& modelSettings)
      : MpcRobotModelBase<SCALAR_T>(modelSettings, 2 * (6 + modelSettings.mpc_joint_dim), 6 * N_CONTACTS + modelSettings.mpc_joint_dim) {}
  ~WBAccelMpcRobotModel() override = default;
  WBAccelMpcRobotModel* clone() const override { return new WBAccelMpcRobotModel(*this); }

  /******************************************************************************************************/
  /*                                          Start indices                                             */
  /******************************************************************************************************/

  size_t getBaseStartindex() const override { return 0; };
  size_t getJointStartindex() const override { return 6; };
  // Be careful, the joint Velocities are part of the state vector here.
  size_t getJointVelocitiesStartindex() const override { return (12 + this->modelSettings.mpc_joint_dim); };
  size_t getJointAccelerationsStartindex() const { return (6 * N_CONTACTS); };

  // Assumes contact wrench [f_x, f_y, f_z, M_x, M_y, M_z]^T
  size_t getContactWrenchStartIndices(size_t contactIndex) const override { return 6 * contactIndex; };
  size_t getContactForceStartIndices(size_t contactIndex) const override { return getContactWrenchStartIndices(contactIndex); };
  size_t getContactMomentStartIndices(size_t contactIndex) const override { return getContactWrenchStartIndices(contactIndex) + 3; };

  /******************************************************************************************************/
  /*                                     Generalized coordinates                                        */
  /******************************************************************************************************/

  VECTOR_T<SCALAR_T> getGeneralizedCoordinates(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.head((6 + this->modelSettings.mpc_joint_dim));
  };

  VECTOR6_T<SCALAR_T> getBasePose(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.head(6);
  };

  VECTOR3_T<SCALAR_T> getBasePosition(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.head(3);
  }

  VECTOR3_T<SCALAR_T> getBaseOrientationEulerZYX(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.segment(3, 3);
  }

  VECTOR3_T<SCALAR_T> getBaseComLinearVelocity(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.segment((6 + this->modelSettings.mpc_joint_dim), 3);
  }

  VECTOR3_T<SCALAR_T> getBaseLinearVelocity(const VECTOR_T<SCALAR_T>& state) const {
    assert(state.size() == this->state_dim);
    return state.segment((6 + this->modelSettings.mpc_joint_dim), 3);
  }

  // Contains Euler angle derivatives, not angular velocity!
  VECTOR6_T<SCALAR_T> getBaseComVelocity(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.segment((6 + this->modelSettings.mpc_joint_dim), 6);
  };

  VECTOR3_T<SCALAR_T> getBaseEulerZYXDerivatives(const VECTOR_T<SCALAR_T>& state) const {
    assert(state.size() == this->state_dim);
    return state.segment((6 + this->modelSettings.mpc_joint_dim) + 3, 3);
  }

  VECTOR_T<SCALAR_T> getJointAngles(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.segment(getJointStartindex(), this->modelSettings.mpc_joint_dim);
  };

  VECTOR_T<SCALAR_T> getJointVelocities(const VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& input) const override {
    assert(state.size() == this->state_dim);
    assert(input.size() == this->input_dim);
    return state.tail(this->modelSettings.mpc_joint_dim);
  };

  VECTOR_T<SCALAR_T> getGeneralizedVelocities(const VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& input) override {
    assert(state.size() == this->state_dim);
    return state.tail((6 + this->modelSettings.mpc_joint_dim));
  };

  VECTOR_T<SCALAR_T> getJointAccelerations(const VECTOR_T<SCALAR_T>& input) const {
    assert(input.size() == this->input_dim);
    return input.tail(this->modelSettings.mpc_joint_dim);
  };

  void setGeneralizedCoordinates(VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& generalizedCorrdinates) const override {
    assert(state.size() == this->state_dim);
    assert(generalizedCorrdinates.size() == (6 + this->modelSettings.mpc_joint_dim));
    state.head((6 + this->modelSettings.mpc_joint_dim)) = generalizedCorrdinates;
  }

  void setBasePose(VECTOR_T<SCALAR_T>& state, const VECTOR6_T<SCALAR_T>& basePose) const override {
    assert(state.size() == this->state_dim);
    state.head(6) = basePose;
  }

  void setBasePosition(VECTOR_T<SCALAR_T>& state, const VECTOR3_T<SCALAR_T>& position) const override {
    assert(state.size() == this->state_dim);
    state.head(3) = position;
  }

  void setBaseOrientationEulerZYX(VECTOR_T<SCALAR_T>& state, const VECTOR3_T<SCALAR_T>& eulerAnglesZYX) const override {
    assert(state.size() == this->state_dim);
    state.segment(3, 3) = eulerAnglesZYX;
  }

  void setBaseLinearVelocity(VECTOR_T<SCALAR_T>& state, const VECTOR3_T<SCALAR_T>& velocity) const {
    assert(state.size() == this->state_dim);
    state.segment((6 + this->modelSettings.mpc_joint_dim), 3) = velocity;
  }

  void setBaseOrientationEulerZYXDerivatives(VECTOR_T<SCALAR_T>& state, const VECTOR3_T<SCALAR_T>& eulerAnglesZYXDerivative) const {
    assert(state.size() == this->state_dim);
    state.segment((6 + this->modelSettings.mpc_joint_dim) + 3, 3) = eulerAnglesZYXDerivative;
  }

  void setJointAngles(VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& jointAngles) const override {
    assert(state.size() == this->state_dim);
    state.segment(getJointStartindex(), this->modelSettings.mpc_joint_dim) = jointAngles;
  }

  void setJointVelocities(VECTOR_T<SCALAR_T>& state, VECTOR_T<SCALAR_T>& input, const VECTOR_T<SCALAR_T>& jointVelocities) const override {
    assert(state.size() == this->state_dim);
    assert(input.size() == this->input_dim);
    state.tail(this->modelSettings.mpc_joint_dim) = jointVelocities;
  }

  void adaptBasePoseHeight(VECTOR_T<SCALAR_T>& state, scalar_t heightChange) const {
    assert(state.size() == this->state_dim);
    state[2] += heightChange;
  }

  /******************************************************************************************************/
  /*                                          Contacts                                                  */
  /******************************************************************************************************/

  VECTOR6_T<SCALAR_T> getContactWrench(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    return input.segment(getContactWrenchStartIndices(contactIndex), 6);
  };

  VECTOR3_T<SCALAR_T> getContactForce(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    return input.segment(getContactForceStartIndices(contactIndex), 3);
  };

  VECTOR3_T<SCALAR_T> getContactMoment(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    return input.segment(getContactMomentStartIndices(contactIndex), 3);
  };

  void setContactWrench(VECTOR_T<SCALAR_T>& input, const VECTOR6_T<SCALAR_T>& wrench, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    input.segment(getContactWrenchStartIndices(contactIndex), 6) = wrench;
  };

  void setContactForce(VECTOR_T<SCALAR_T>& input, const VECTOR3_T<SCALAR_T>& force, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    input.segment(getContactForceStartIndices(contactIndex), 3) = force;
  };

  void setContactMoment(VECTOR_T<SCALAR_T>& input, const VECTOR3_T<SCALAR_T>& moment, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    input.segment(getContactMomentStartIndices(contactIndex), 3) = moment;
  };

 private:
  WBAccelMpcRobotModel(const WBAccelMpcRobotModel& rhs) : MpcRobotModelBase<SCALAR_T>(rhs){};
};

}  // namespace ocs2::humanoid
