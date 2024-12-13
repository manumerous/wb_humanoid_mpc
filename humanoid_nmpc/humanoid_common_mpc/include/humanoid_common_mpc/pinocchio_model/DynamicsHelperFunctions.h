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

#include <pinocchio/fwd.hpp>

#include <array>
#include <cppad/cg.hpp>
#include <iostream>
#include <memory>

#include <pinocchio/algorithm/center-of-mass.hpp>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/pinocchio_model/PinocchioFrameConversions.h"

namespace ocs2::humanoid {

///
/// @brief Updates all frame placement in the PinocchioInterface.
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param q Current generalized coordinates.
/// @param pinocchioInterface Pinocchio interface.
///

template <typename SCALAR_T>
void updateFramePlacements(const VECTOR_T<SCALAR_T>& q, PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface);

///
/// @brief Computes and returns all the contact positions in the inertial frame.
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param q Current generalized coordinates.
/// @param pinocchioInterface Pinocchio interface.
///
/// @return Vector of contact positions in the inertial frame.

template <typename SCALAR_T>
void updateFramePlacements(const VECTOR_T<SCALAR_T>& q, const pinocchio::ModelTpl<SCALAR_T>& model, pinocchio::DataTpl<SCALAR_T>& data);

///
/// @brief Computes and returns all the contact positions in the inertial frame.
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param q Current generalized coordinates.
/// @param pinocchioInterface Pinocchio interface.
///
/// @return Vector of contact positions in the inertial frame.

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> computeContactPositions(const VECTOR_T<SCALAR_T>& q,
                                                         PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                         const MpcRobotModelBase<SCALAR_T>& mpcRobotModel);

///
/// @brief Returns all the contact positions in the inertial frame.
/// Assumes that the frame placements are up to date (e.g. updateFramePlacements has been called).
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param pinocchioInterface Pinocchio interface.
///
/// @return Vector of contact positions in the inertial frame.

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> getContactPositions(const PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                     const MpcRobotModelBase<SCALAR_T>& mpcRobotModel);

///
/// @brief Computes and returns all the frame positions in the inertial frame.
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param q Current generalized coordinates.
/// @param pinocchioInterface Pinocchio interface.
/// @param frameNames Names of the frames for which the positions should be computed.
///
/// @return Vector of contact positions in the inertial frame.

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> computeFramePositions(const VECTOR_T<SCALAR_T>& q,
                                                       PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                       std::vector<std::string> frameNames);

///
/// @brief Returns all the frame positions in the inertial frame.
/// Assumes that the frame placements are up to date (e.g. updateFramePlacements has been called).
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param pinocchioInterface Pinocchio interface.
/// @param frameNames Names of the frames for which the positions should be returned.
///
/// @return Vector of contact positions in the inertial frame.

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> getFramePositions(const PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                   std::vector<std::string> frameNames);

///
/// @brief Gets the estimated ground height using the feet in contact for a pinocchio model with updated frame placements.
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param pinocchioInterface Pinocchio interface.
/// @param measuredMode mode of which feet are in contact.
///
/// @return the estimated ground height.

scalar_t getGroundHeightEstimate(PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                 const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                 size_t measuredMode);

///
/// @brief Computes the estimated ground height using the feet in contact.
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param pinocchioInterface Pinocchio interface.
/// @param q Current generalized coordinates.
/// @param measuredMode mode of which feet are in contact.
///
/// @return the estimated ground height.

scalar_t computeGroundHeightEstimate(PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                     const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                     const vector_t& q,
                                     size_t measuredMode);

///
/// @brief Return amount of legs in contact
///
/// @return number of clodes leg contacts as unsigned integer

inline size_t numberOfLegsInContacts(const contact_flag_t& contactFlags) {
  size_t numStanceLegs = 0;
  for (auto legInContact : contactFlags) {
    if (legInContact) {
      ++numStanceLegs;
    }
  }
  return numStanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

inline vector_t weightCompensatingInput(const PinocchioInterface& pinocchioInterface,
                                        const contact_flag_t& contactFlags,
                                        const MpcRobotModelBase<scalar_t>& mpcRobotModel) {
  const static scalar_t totalGravitationalForce = computeTotalMass(pinocchioInterface.getModel()) * 9.81;
  const auto numStanceLegs = numberOfLegsInContacts(contactFlags);
  vector_t input = vector_t::Zero(mpcRobotModel.getInputDim());
  if (numStanceLegs > 0) {
    const vector3_t forceInInertialFrame(0.0, 0.0, totalGravitationalForce / numStanceLegs);
    for (size_t i = 0; i < contactFlags.size(); i++) {
      if (contactFlags[i]) {
        mpcRobotModel.setContactForce(input, forceInInertialFrame, i);
      }
    }
  }
  return input;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
inline pinocchio::FrameIndex getContactFrameIndex(const PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                  const MpcRobotModelBase<SCALAR_T>& mpcRobotModel,
                                                  size_t contactIndex) {
  return pinocchioInterface.getModel().getFrameId(mpcRobotModel.modelSettings.contactNames[contactIndex]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
inline std::vector<pinocchio::FrameIndex> getContactFrameIndices(const PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                                 const MpcRobotModelBase<SCALAR_T>& mpcRobotModel) {
  std::vector<pinocchio::FrameIndex> contactFrameIndices;
  contactFrameIndices.reserve(N_CONTACTS);
  for (size_t i = 0; i < N_CONTACTS; i++) {
    contactFrameIndices[i] = getContactFrameIndex<SCALAR_T>(pinocchioInterface, mpcRobotModel, i);
  }
  return contactFrameIndices;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

///
/// @brief Computes the center of pressure (CoP) in the inertial frame.
///
/// @warning Assumes that the frame placements are up to date. Does not work when frame is not in contact -> f_z = 0 results in NaN.
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param input Current input.
/// @param pinocchioInterface Pinocchio interface.
///
/// @return Location of center of pressure in the inertial frame.

template <typename SCALAR_T>
inline VECTOR3_T<SCALAR_T> computeContactCoP(const VECTOR_T<SCALAR_T> input,
                                             const PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                             size_t contactIndex,
                                             const MpcRobotModelBase<scalar_t>& mpcRobotModel) {
  const auto localContactWrench =
      rotateVectorWorldToLocal<SCALAR_T>(mpcRobotModel.getContactWrench(input, contactIndex), pinocchioInterface.getData(),
                                         getContactFrameIndex(pinocchioInterface, mpcRobotModel, contactIndex));
  SCALAR_T copX = -localContactWrench[4] / localContactWrench[2];
  SCALAR_T copY = localContactWrench[3] / localContactWrench[2];
  VECTOR3_T<SCALAR_T> copInLocalFrame(copX, copY, 0.0);
  return transformPointLocalToWorld(copInLocalFrame, pinocchioInterface.getData(),
                                    getContactFrameIndex(pinocchioInterface, mpcRobotModel, contactIndex));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

///
/// @brief Computes the center of pressure (CoP) for all contacts in the inertial frame.
///
/// @warning Assumes that the frame placements are up to date.
///
/// @tparam SCALAR_T Scalar type [scalar_t/ad_scalar_t].
/// @param input Current input.
/// @param pinocchioInterface Pinocchio interface.
/// @param contactFlags Flags indicating which contacts are in contact. Returns 0 vector if not in contact.
///
/// @return Locations of center of pressure in the inertial frame.

inline std::vector<vector3_t> computeContactsCoP(const vector_t input,
                                                 const PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                                 const contact_flag_t& contactFlags,
                                                 const MpcRobotModelBase<scalar_t>& mpcRobotModel) {
  std::vector<vector3_t> contactCoPs;
  contactCoPs.reserve(N_CONTACTS);
  for (size_t contactIndex = 0; contactIndex < N_CONTACTS; contactIndex++) {
    if (contactFlags[contactIndex]) {
      contactCoPs.emplace_back(computeContactCoP<scalar_t>(input, pinocchioInterface, contactIndex, mpcRobotModel));
    } else {
      contactCoPs.emplace_back(vector3_t::Zero());
    }
  }
  return contactCoPs;
}

}  // namespace ocs2::humanoid
