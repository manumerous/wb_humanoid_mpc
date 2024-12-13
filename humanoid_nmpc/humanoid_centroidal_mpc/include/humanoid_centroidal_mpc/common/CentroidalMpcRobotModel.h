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

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

namespace ocs2::humanoid {

/******************************************************************************************************/
/* State Vector definition

  Define the state vector x = [h, q_b, q_j]^T

    Centroidal momentum h = [vcom_x, vcom_y, vcom_z, L_x / mass, L_y / mass, L_z / mass]^T
    Base pose q_b = [p_base_x, p_base_y, p_base_z, theta_base_z, theta_base_y, theta_base_x]
    Joint angles q_j

*/
/******************************************************************************************************/

/******************************************************************************************************/
/* Input Vector definition

  Define the input vector u = [W_l, W_r, q_dot_j]^T

    Left contact Wrench W_l in inertial frame
    Right contact Wrench W_r in inertial frame
    Joint velocities q_dot_j

*/
/******************************************************************************************************/

template <typename SCALAR_T>
class CentroidalMpcRobotModel : public MpcRobotModelBase<SCALAR_T> {
 public:
  CentroidalMpcRobotModel(const ModelSettings& modelSettings,
                          const PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                          const CentroidalModelInfoTpl<SCALAR_T>& centroidalModelInfo)
      : MpcRobotModelBase<SCALAR_T>(modelSettings, 12 + modelSettings.mpc_joint_dim, 6 * N_CONTACTS + modelSettings.mpc_joint_dim),
        pinocchioInterface_(pinocchioInterface),
        centroidalModelInfo_(centroidalModelInfo),
        pinocchioMappingPtr_(new CentroidalModelPinocchioMappingTpl<SCALAR_T>(centroidalModelInfo)) {
    pinocchioMappingPtr_->setPinocchioInterface(pinocchioInterface_);
  };

  ~CentroidalMpcRobotModel() override = default;
  CentroidalMpcRobotModel* clone() const override { return new CentroidalMpcRobotModel(*this); }

  /******************************************************************************************************/
  /*                                          Start indices                                             */
  /******************************************************************************************************/

  size_t getBaseStartindex() const override { return 6; };
  size_t getJointStartindex() const override { return 12; };
  size_t getJointVelocitiesStartindex() const override { return 6 * N_CONTACTS; };

  // Assumes contact wrench [f_x, f_y, f_z, M_x, M_y, M_z]^T
  size_t getContactWrenchStartIndices(size_t contactIndex) const override { return 6 * contactIndex; };
  size_t getContactForceStartIndices(size_t contactIndex) const override { return getContactWrenchStartIndices(contactIndex); };
  size_t getContactMomentStartIndices(size_t contactIndex) const override { return getContactWrenchStartIndices(contactIndex) + 3; };

  /******************************************************************************************************/
  /*                                     Generalized coordinates                                        */
  /******************************************************************************************************/

  VECTOR_T<SCALAR_T> getGeneralizedCoordinates(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.tail(6 + this->modelSettings.mpc_joint_dim);
  };

  VECTOR6_T<SCALAR_T> getBasePose(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.segment(6, 6);
  };

  VECTOR3_T<SCALAR_T> getBasePosition(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.segment(6, 3);
  }

  VECTOR3_T<SCALAR_T> getBaseOrientationEulerZYX(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.segment(6 + 3, 3);
  }

  // Return Com linear velocity
  VECTOR3_T<SCALAR_T> getBaseComLinearVelocity(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.head(3);
  };

  VECTOR6_T<SCALAR_T> getBaseComVelocity(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.head(6);
  };

  VECTOR_T<SCALAR_T> getJointAngles(const VECTOR_T<SCALAR_T>& state) const override {
    assert(state.size() == this->state_dim);
    return state.tail(this->modelSettings.mpc_joint_dim);
  };

  VECTOR_T<SCALAR_T> getJointVelocities(const VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& input) const override {
    assert(input.size() == this->input_dim);
    return input.tail(this->modelSettings.mpc_joint_dim);
  };

  VECTOR_T<SCALAR_T> getGeneralizedVelocities(const VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& input) override {
    assert(state.size() == this->state_dim);
    assert(input.size() == this->input_dim);
    updateCentroidalDynamics<SCALAR_T>(pinocchioInterface_, centroidalModelInfo_, pinocchioMappingPtr_->getPinocchioJointPosition(state));
    return pinocchioMappingPtr_->getPinocchioJointVelocity(state, input);
  };

  void setGeneralizedCoordinates(VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& generalizedCorrdinates) const override {
    assert(state.size() == this->state_dim);
    assert(generalizedCorrdinates.size() == 6 + this->modelSettings.mpc_joint_dim);
    state.tail(6 + this->modelSettings.mpc_joint_dim) = generalizedCorrdinates;
  }

  void setBasePose(VECTOR_T<SCALAR_T>& state, const VECTOR6_T<SCALAR_T>& basePose) const override {
    assert(state.size() == this->state_dim);
    state.segment(6, 6) = basePose;
  }

  void setBasePosition(VECTOR_T<SCALAR_T>& state, const VECTOR3_T<SCALAR_T>& position) const override {
    assert(state.size() == this->state_dim);
    state.segment(6, 3) = position;
  }

  void setBaseOrientationEulerZYX(VECTOR_T<SCALAR_T>& state, const VECTOR3_T<SCALAR_T>& eulerAnglesZYX) const override {
    assert(state.size() == this->state_dim);
    state.segment(6 + 3, 3) = eulerAnglesZYX;
  }

  void setJointAngles(VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& jointAngles) const override {
    assert(state.size() == this->state_dim);
    state.tail(this->modelSettings.mpc_joint_dim) = jointAngles;
  }

  void setJointVelocities(VECTOR_T<SCALAR_T>& state, VECTOR_T<SCALAR_T>& input, const VECTOR_T<SCALAR_T>& jointVelocities) const override {
    assert(state.size() == this->state_dim);
    assert(input.size() == this->input_dim);
    input.tail(this->modelSettings.mpc_joint_dim) = jointVelocities;
  }

  void adaptBasePoseHeight(VECTOR_T<SCALAR_T>& state, scalar_t heightChange) const {
    assert(state.size() == this->state_dim);
    state[6 + 2] += heightChange;
  }

  /******************************************************************************************************/
  /*                                          Contacts                                                  */
  /******************************************************************************************************/

  VECTOR6_T<SCALAR_T> getContactWrench(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    return input.segment(getContactWrenchStartIndices(contactIndex), CONTACT_WRENCH_DIM);
  };

  VECTOR3_T<SCALAR_T> getContactForce(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    return input.segment(getContactWrenchStartIndices(contactIndex), 3);
  };

  VECTOR3_T<SCALAR_T> getContactMoment(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    return input.segment((getContactWrenchStartIndices(contactIndex) + 3), 3);
  };

  void setContactWrench(VECTOR_T<SCALAR_T>& input, const VECTOR6_T<SCALAR_T>& wrench, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    input.segment(getContactWrenchStartIndices(contactIndex), 6) = wrench;
  };

  void setContactForce(VECTOR_T<SCALAR_T>& input, const VECTOR3_T<SCALAR_T>& force, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    input.segment(getContactWrenchStartIndices(contactIndex), 3) = force;
  };

  void setContactMoment(VECTOR_T<SCALAR_T>& input, const VECTOR3_T<SCALAR_T>& moment, size_t contactIndex) const override {
    assert(input.size() == this->input_dim);
    input.segment(getContactWrenchStartIndices(contactIndex) + 3, 3) = moment;
  };

  /******************************************************************************************************/
  /*                                    Custom Centroidal Methods                                       */
  /******************************************************************************************************/

  VECTOR_T<SCALAR_T> getCentroidalMomentum(const VECTOR_T<SCALAR_T>& state) const {
    assert(state.size() == this->state_dim);
    return state.head(6);
  };

 private:
  CentroidalMpcRobotModel(const CentroidalMpcRobotModel& rhs)
      : MpcRobotModelBase<SCALAR_T>(rhs),
        pinocchioInterface_(rhs.pinocchioInterface_),
        centroidalModelInfo_(rhs.centroidalModelInfo_),
        pinocchioMappingPtr_(rhs.pinocchioMappingPtr_->clone()) {
    pinocchioMappingPtr_->setPinocchioInterface(pinocchioInterface_);
  };

  PinocchioInterfaceTpl<SCALAR_T> pinocchioInterface_;
  const CentroidalModelInfoTpl<SCALAR_T> centroidalModelInfo_;
  const std::unique_ptr<CentroidalModelPinocchioMappingTpl<SCALAR_T>> pinocchioMappingPtr_;
};

}  // namespace ocs2::humanoid
