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

#include <pinocchio/fwd.hpp>

#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"

// Pinnochio
#include <pinocchio/algorithm/contact-dynamics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <humanoid_common_mpc/gait/MotionPhaseDefinition.h>

namespace ocs2::humanoid {

template <typename SCALAR_T>
void updateFramePlacements(const VECTOR_T<SCALAR_T>& q, PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface) {
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  updateFramePlacements(q, model, data);
}
template void updateFramePlacements(const ad_vector_t& q, PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface);
template void updateFramePlacements(const vector_t& q, PinocchioInterfaceTpl<scalar_t>& pinocchioInterface);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
void updateFramePlacements(const VECTOR_T<SCALAR_T>& q, const pinocchio::ModelTpl<SCALAR_T>& model, pinocchio::DataTpl<SCALAR_T>& data) {
  pinocchio::forwardKinematics(model, data, q);
  updateFramePlacements(model, data);
}
template void updateFramePlacements(const ad_vector_t& q,
                                    const pinocchio::ModelTpl<ad_scalar_t>& model,
                                    pinocchio::DataTpl<ad_scalar_t>& data);
template void updateFramePlacements(const vector_t& q, const pinocchio::ModelTpl<scalar_t>& model, pinocchio::DataTpl<scalar_t>& data);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> computeContactPositions(const VECTOR_T<SCALAR_T>& q,
                                                         PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                         const MpcRobotModelBase<SCALAR_T>& mpcRobotModel) {
  updateFramePlacements<SCALAR_T>(q, pinocchioInterface);
  return getContactPositions<SCALAR_T>(pinocchioInterface, mpcRobotModel);
}
template std::vector<VECTOR3_T<ad_scalar_t>> computeContactPositions(const VECTOR_T<ad_scalar_t>& q,
                                                                     PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface,
                                                                     const MpcRobotModelBase<ad_scalar_t>& mpcRobotModel);
template std::vector<VECTOR3_T<scalar_t>> computeContactPositions(const VECTOR_T<scalar_t>& q,
                                                                  PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                                                  const MpcRobotModelBase<scalar_t>& mpcRobotModel);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> getContactPositions(const PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                     const MpcRobotModelBase<SCALAR_T>& mpcRobotModel) {
  assert(mpcRobotModel.modelSettings.contactNames.size() == N_CONTACTS);
  std::vector<VECTOR3_T<SCALAR_T>> footPositions;
  footPositions.reserve(N_CONTACTS);
  const auto& data = pinocchioInterface.getData();
  std::vector<pinocchio::FrameIndex> contactFrameIndices = getContactFrameIndices(pinocchioInterface, mpcRobotModel);

  for (size_t i = 0; i < N_CONTACTS; i++) {
    const VECTOR3_T<SCALAR_T>& footPosition = data.oMf[getContactFrameIndex(pinocchioInterface, mpcRobotModel, i)].translation();
    footPositions.emplace_back(footPosition);
  }
  return footPositions;
}
template std::vector<VECTOR3_T<ad_scalar_t>> getContactPositions(const PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface,
                                                                 const MpcRobotModelBase<ad_scalar_t>& mpcRobotModel);
template std::vector<VECTOR3_T<scalar_t>> getContactPositions(const PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                                              const MpcRobotModelBase<scalar_t>& mpcRobotModel);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> computeFramePositions(const VECTOR_T<SCALAR_T>& q,
                                                       PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                       std::vector<std::string> frameNames) {
  updateFramePlacements<SCALAR_T>(q, pinocchioInterface);
  return getFramePositions<SCALAR_T>(pinocchioInterface, frameNames);
}
template std::vector<VECTOR3_T<ad_scalar_t>> computeFramePositions(const VECTOR_T<ad_scalar_t>& q,
                                                                   PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface,
                                                                   std::vector<std::string> frameNames);
template std::vector<VECTOR3_T<scalar_t>> computeFramePositions(const VECTOR_T<scalar_t>& q,
                                                                PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                                                std::vector<std::string> frameNames);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
std::vector<VECTOR3_T<SCALAR_T>> getFramePositions(const PinocchioInterfaceTpl<SCALAR_T>& pinocchioInterface,
                                                   std::vector<std::string> frameNames) {
  std::vector<VECTOR3_T<SCALAR_T>> positions;
  positions.reserve(frameNames.size());
  const auto& data = pinocchioInterface.getData();
  for (size_t i = 0; i < frameNames.size(); i++) {
    const pinocchio::FrameIndex frameIndex = pinocchioInterface.getModel().getFrameId(frameNames[i]);
    const VECTOR3_T<SCALAR_T>& position = data.oMf[frameIndex].translation();
    positions.emplace_back(position);
  }
  return positions;
}
template std::vector<VECTOR3_T<ad_scalar_t>> getFramePositions(const PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface,
                                                               std::vector<std::string> frameNames);
template std::vector<VECTOR3_T<scalar_t>> getFramePositions(const PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                                            std::vector<std::string> frameNames);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

scalar_t computeGroundHeightEstimate(PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                     const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                     const vector_t& q,
                                     size_t measuredMode) {
  updateFramePlacements<scalar_t>(q, pinocchioInterface);
  return getGroundHeightEstimate(pinocchioInterface, mpcRobotModel, measuredMode);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

scalar_t getGroundHeightEstimate(PinocchioInterfaceTpl<scalar_t>& pinocchioInterface,
                                 const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                 size_t measuredMode) {
  contact_flag_t measuredContactFlags = modeNumber2StanceLeg(measuredMode);

  std::vector<vector3_t> contactPositions = getContactPositions<scalar_t>(pinocchioInterface, mpcRobotModel);

  static scalar_t terrainHeight = 0.0;

  // Use right foot if in contact
  if (measuredContactFlags[0] && measuredContactFlags[1]) {
    vector3_t footPosition1 = contactPositions[0];
    vector3_t footPosition2 = contactPositions[1];
    terrainHeight = 0.5 * (footPosition1[2] + footPosition2[2]);
  } else if (measuredContactFlags[0]) {
    vector3_t footPosition = contactPositions[0];
    terrainHeight = footPosition[2];
  } else if (measuredContactFlags[1]) {
    vector3_t footPosition = contactPositions[1];
    terrainHeight = footPosition[2];
  }
  return terrainHeight;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

}  // namespace ocs2::humanoid
