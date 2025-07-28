/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
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

#include "humanoid_wb_mpc/dynamics/DynamicsHelperFunctions.h"
#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"

#include "humanoid_common_mpc/common/ModelSettings.h"

#include <ocs2_robotic_tools/common/RotationDerivativesTransforms.h>

// Pinnochio
#include <pinocchio/algorithm/contact-dynamics.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
VECTOR6_T<SCALAR_T> computeBaseAcceleration(const VECTOR_T<SCALAR_T>& state,
                                            const VECTOR_T<SCALAR_T>& input,
                                            const PinocchioInterfaceTpl<SCALAR_T>& pinInterface,
                                            WBAccelMpcRobotModel<SCALAR_T>& mpcRobotModel) {
  const auto& model = pinInterface.getModel();
  auto data = pinInterface.getData();
  const VECTOR_T<SCALAR_T> q = mpcRobotModel.getGeneralizedCoordinates(state);
  const VECTOR_T<SCALAR_T> qd = mpcRobotModel.getGeneralizedVelocities(state, input);
  const VECTOR_T<SCALAR_T> qdd_joints = mpcRobotModel.getJointAccelerations(input);

  data.M.fill(SCALAR_T(0.0));
  pinocchio::crba(model, data, q);
  pinocchio::nonLinearEffects(model, data, q, qd);

  // Compute Jacobians for the foot frames
  MATRIX_T<SCALAR_T> J_foot_l = MATRIX_T<SCALAR_T>::Zero(6, mpcRobotModel.getGenCoordinatesDim());
  MATRIX_T<SCALAR_T> J_foot_r = MATRIX_T<SCALAR_T>::Zero(6, mpcRobotModel.getGenCoordinatesDim());

  pinocchio::computeFrameJacobian(model, data, q, model.getFrameId("foot_l_contact"), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                  J_foot_l);
  pinocchio::computeFrameJacobian(model, data, q, model.getFrameId("foot_r_contact"), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                  J_foot_r);

  MATRIX6_T<SCALAR_T> J_foot_l_b = J_foot_l.block(0, 0, 6, 6);
  MATRIX6_T<SCALAR_T> J_foot_r_b = J_foot_r.block(0, 0, 6, 6);

  VECTOR6_T<SCALAR_T> baseExternalForces =
      J_foot_l_b.transpose() * mpcRobotModel.getContactWrench(input, 0) + J_foot_r_b.transpose() * mpcRobotModel.getContactWrench(input, 1);

  return computeBaseAcceleration<SCALAR_T>(data.M, data.nle, qdd_joints, baseExternalForces);
}
template ad_vector6_t computeBaseAcceleration(const ad_vector_t& state,
                                              const ad_vector_t& input,
                                              const PinocchioInterfaceTpl<ad_scalar_t>& pinInterface,
                                              WBAccelMpcRobotModel<ad_scalar_t>& mpcRobotModel);
template vector6_t computeBaseAcceleration(const vector_t& state,
                                           const vector_t& input,
                                           const PinocchioInterfaceTpl<scalar_t>& pinInterface,
                                           WBAccelMpcRobotModel<scalar_t>& mpcRobotModel);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
VECTOR_T<SCALAR_T> computeGeneralizedAccelerations(const VECTOR_T<SCALAR_T>& state,
                                                   const VECTOR_T<SCALAR_T>& input,
                                                   const PinocchioInterfaceTpl<SCALAR_T>& pinInterface,
                                                   WBAccelMpcRobotModel<SCALAR_T>& mpcRobotModel) {
  // Generalized Accelerations = [ddq_base, ddq_joints]
  VECTOR_T<SCALAR_T> generalizedAccelerations = VECTOR_T<SCALAR_T>::Zero(mpcRobotModel.getGenCoordinatesDim());
  generalizedAccelerations.head(6) = computeBaseAcceleration<SCALAR_T>(state, input, pinInterface, mpcRobotModel);
  generalizedAccelerations.tail(mpcRobotModel.getJointDim()) = mpcRobotModel.getJointAccelerations(input);
  return generalizedAccelerations;
}
template ad_vector_t computeGeneralizedAccelerations(const ad_vector_t& state,
                                                     const ad_vector_t& input,
                                                     const PinocchioInterfaceTpl<ad_scalar_t>& pinInterface,
                                                     WBAccelMpcRobotModel<ad_scalar_t>& mpcRobotModel);
template vector_t computeGeneralizedAccelerations(const vector_t& state,
                                                  const vector_t& input,
                                                  const PinocchioInterfaceTpl<scalar_t>& pinInterface,
                                                  WBAccelMpcRobotModel<scalar_t>& mpcRobotModel);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
VECTOR_T<SCALAR_T> computeStateDerivative(const VECTOR_T<SCALAR_T>& state,
                                          const VECTOR_T<SCALAR_T>& input,
                                          const PinocchioInterfaceTpl<SCALAR_T>& pinInterface,
                                          WBAccelMpcRobotModel<SCALAR_T>& mpcRobotModel) {
  // State derivative = [dq; ddq_base, ddq_joints]
  VECTOR_T<SCALAR_T> state_derivative = VECTOR_T<SCALAR_T>::Zero(mpcRobotModel.getStateDim());
  state_derivative.head(3) = state.segment(mpcRobotModel.getGenCoordinatesDim(), 3);
  // Derivatives of the euler angles ZYX
  state_derivative.segment(3, 3) = state.segment(mpcRobotModel.getGenCoordinatesDim() + 3, 3);
  state_derivative.segment(6, mpcRobotModel.getJointDim()) = mpcRobotModel.getJointVelocities(state, input);
  state_derivative.tail(mpcRobotModel.getGenCoordinatesDim()) =
      computeGeneralizedAccelerations<SCALAR_T>(state, input, pinInterface, mpcRobotModel);
  return state_derivative;
}
template ad_vector_t computeStateDerivative(const ad_vector_t& state,
                                            const ad_vector_t& input,
                                            const PinocchioInterfaceTpl<ad_scalar_t>& pinInterface,
                                            WBAccelMpcRobotModel<ad_scalar_t>& mpcRobotModel);
template vector_t computeStateDerivative(const vector_t& state,
                                         const vector_t& input,
                                         const PinocchioInterfaceTpl<scalar_t>& pinInterface,
                                         WBAccelMpcRobotModel<scalar_t>& mpcRobotModel);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

// template <typename SCALAR_T>
// VECTOR_T<SCALAR_T> computeJointTorques(const VECTOR_T<SCALAR_T>& state,
//                                        const VECTOR_T<SCALAR_T>& input,
//                                        const PinocchioInterfaceTpl<SCALAR_T>& pinInterface,
//                                        WBAccelMpcRobotModel<SCALAR_T>& mpcRobotModel) {
//   const auto& model = pinInterface.getModel();
//   pinocchio::DataTpl<SCALAR_T>& data = pinInterface.getData();
//   const VECTOR_T<SCALAR_T> q = mpcRobotModel.getGeneralizedCoordinates(state);
//   const VECTOR_T<SCALAR_T> qd = mpcRobotModel.getGeneralizedVelocities(state, input);
//   const VECTOR_T<SCALAR_T> qdd_joints = mpcRobotModel.getJointAccelerations(input);

//   data.M.fill(SCALAR_T(0.0));
//   pinocchio::crba(model, data, q);
//   pinocchio::nonLinearEffects(model, data, q, qd);

//   // Compute Jacobians for the foot frames
//   MATRIX_T<SCALAR_T> J_foot_l = MATRIX_T<SCALAR_T>::Zero(6, mpcRobotModel.getGenCoordinatesDim());
//   MATRIX_T<SCALAR_T> J_foot_r = MATRIX_T<SCALAR_T>::Zero(6, mpcRobotModel.getGenCoordinatesDim());

//   ////////////////////////////////////////////////////////////////////////////

//   pinocchio::computeFrameJacobian(model, data, q, model.getFrameId("foot_l_contact"), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
//                                   J_foot_l);
//   pinocchio::computeFrameJacobian(model, data, q, model.getFrameId("foot_r_contact"), pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
//                                   J_foot_r);

//   // Project contact wrenches into the joint space

//   VECTOR_T<SCALAR_T> externalForcesInJointSpace =
//       J_foot_l.transpose() * mpcRobotModel.getContactWrench(input, 0) + J_foot_r.transpose() * mpcRobotModel.getContactWrench(input, 1);

//   VECTOR6_T<SCALAR_T> baseAccelerations = computeBaseAcceleration(data.M, data.nle, qdd_joints, externalForcesInJointSpace);

//   VECTOR_T<SCALAR_T> q_dd(mpcRobotModel.getGenCoordinatesDim());
//   q_dd << baseAccelerations, qdd_joints;

//   VECTOR_T<SCALAR_T> jointTorques = data.M.bottomRows(mpcRobotModel.getJointDim()) * q_dd + data.nle.tail(mpcRobotModel.getJointDim()) -
//                                     externalForcesInJointSpace.tail(mpcRobotModel.getJointDim());

//   return jointTorques;
// }
// template ad_vector_t computeJointTorques(const ad_vector_t& state,
//                                          const ad_vector_t& input,
//                                          const PinocchioInterfaceTpl<ad_scalar_t>& pinInterface,
//                                          WBAccelMpcRobotModel<ad_scalar_t>& mpcRobotModel);
// template vector_t computeJointTorques(const vector_t& state,
//                                       const vector_t& input,
//                                       const PinocchioInterfaceTpl<scalar_t>& pinInterface,
//                                       WBAccelMpcRobotModel<scalar_t>& mpcRobotModel);

template <typename SCALAR_T>
VECTOR_T<SCALAR_T> computeJointTorques(const VECTOR_T<SCALAR_T>& state,
                                       const VECTOR_T<SCALAR_T>& input,
                                       PinocchioInterfaceTpl<SCALAR_T>& pinInterface,
                                       WBAccelMpcRobotModel<SCALAR_T>& mpcRobotModel) {
  const VECTOR_T<SCALAR_T> q = mpcRobotModel.getGeneralizedCoordinates(state);
  const VECTOR_T<SCALAR_T> qd = mpcRobotModel.getGeneralizedVelocities(state, input);
  const VECTOR_T<SCALAR_T> qdd_joints = mpcRobotModel.getJointAccelerations(input);

  const std::array<VECTOR6_T<SCALAR_T>, 2> footWrenches{mpcRobotModel.getContactWrench(input, 0), mpcRobotModel.getContactWrench(input, 1)};

  return computeJointTorques<SCALAR_T>(q, qd, qdd_joints, footWrenches, pinInterface);
}
template ad_vector_t computeJointTorques(const ad_vector_t& state,
                                         const ad_vector_t& input,
                                         PinocchioInterfaceTpl<ad_scalar_t>& pinInterface,
                                         WBAccelMpcRobotModel<ad_scalar_t>& mpcRobotModel);
template vector_t computeJointTorques(const vector_t& state,
                                      const vector_t& input,
                                      PinocchioInterfaceTpl<scalar_t>& pinInterface,
                                      WBAccelMpcRobotModel<scalar_t>& mpcRobotModel);

}  // namespace ocs2::humanoid