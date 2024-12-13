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

#include <functional>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

#include <humanoid_wb_mpc/common/WBAccelMpcRobotModel.h>
#include <humanoid_wb_mpc/end_effector/EndEffectorDynamics.h>

namespace ocs2::humanoid {

/**
 * This class provides the CppAD implementation the end-effector Kinematics based on pinocchio. No pre-computation
 * is required. The class has two constructors. The constructor with an additional argument, "updateCallback", is meant
 * for cases where PinocchioStateInputMapping requires some extra update calls on PinocchioInterface, such as the
 * centroidal model mapping (refer to CentroidalModelPinocchioMapping).
 *
 * See also PinocchioEndEffectorKinematics, which uses analytical computation and caching.
 */
class PinocchioEndEffectorDynamicsCppAd final : public EndEffectorDynamics<scalar_t> {
 public:
  using EndEffectorKinematics<scalar_t>::vector3_t;
  using EndEffectorKinematics<scalar_t>::matrix3x_t;
  using EndEffectorKinematics<scalar_t>::vector6_t;
  using EndEffectorKinematics<scalar_t>::matrix6x_t;
  using EndEffectorKinematics<scalar_t>::quaternion_t;
  using EndEffectorKinematics<scalar_t>::vector_t;
  using update_pinocchio_interface_callback =
      std::function<void(const ad_vector_t& state, PinocchioInterfaceTpl<ad_scalar_t>& pinocchioInterface)>;

  /** Constructor
   * @param [in] pinocchioInterface pinocchio interface.
   * @param [in] mpcRobotModel mapping from OCS2 to pinocchio state.
   * @param [in] endEffectorIds array of end effector names.
   * @param [in] modelName : name of the generate model library
   * @param [in] modelFolder : folder to save the model library files to
   * @param [in] recompileLibraries : If true, the model library will be newly compiled. If false, an existing library will be loaded if
   *                                  available.
   * @param [in] verbose : print information.
   */
  PinocchioEndEffectorDynamicsCppAd(const PinocchioInterface& pinocchioInterface,
                                    WBAccelMpcRobotModel<ad_scalar_t>& mpcRobotModel,
                                    std::vector<std::string> endEffectorIds,
                                    const std::string& modelName,
                                    const std::string& modelFolder = "/tmp/ocs2",
                                    bool recompileLibraries = true,
                                    bool verbose = false);

  /** Constructor
   * @param [in] pinocchioInterface pinocchio interface.
   * @param [in] mpcRobotModel mapping from OCS2 to pinocchio state.
   * @param [in] endEffectorIds array of end effector names.
   * @param [in] updateCallback : In the cases that PinocchioStateInputMapping requires some additional update calls on PinocchioInterface,
   *                              use this callback.
   * @param [in] modelName : name of the generate model library
   * @param [in] modelFolder : folder to save the model library files to
   * @param [in] recompileLibraries : If true, the model library will be newly compiled. If false, an existing library will be loaded if
   *                                  available.
   * @param [in] verbose : print information.
   */
  PinocchioEndEffectorDynamicsCppAd(const PinocchioInterface& pinocchioInterface,
                                    WBAccelMpcRobotModel<ad_scalar_t>& mpcRobotModel,
                                    std::vector<std::string> endEffectorIds,
                                    update_pinocchio_interface_callback updateCallback,
                                    const std::string& modelName,
                                    const std::string& modelFolder = "/tmp/ocs2",
                                    bool recompileLibraries = true,
                                    bool verbose = false);

  ~PinocchioEndEffectorDynamicsCppAd() override = default;
  PinocchioEndEffectorDynamicsCppAd* clone() const override;
  PinocchioEndEffectorDynamicsCppAd& operator=(const PinocchioEndEffectorDynamicsCppAd&) = delete;

  const std::vector<std::string>& getIds() const override;

  std::vector<vector3_t> getPosition(const vector_t& state) const override;
  std::vector<vector3_t> getVelocity(const vector_t& state, const vector_t& input) const override;

  // This function contains no linear approximation due to the non-minimal quaternion representation of SO(3)
  // Use getOrientationErrorLinearApproximation or getOrientationErrorWrtPlaneLinearApproximation is a linear approximation is needed.
  std::vector<quaternion_t> getOrientation(const vector_t& state) const override;
  std::vector<vector3_t> getOrientationError(const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const override;
  std::vector<vector3_t> getOrientationErrorWrtPlane(const vector_t& state, const std::vector<vector3_t>& planeNormals) const override;
  std::vector<vector3_t> getAngularVelocity(const vector_t& state, const vector_t& input) const override;
  std::vector<vector6_t> getTwist(const vector_t& state, const vector_t& input) const override;
  std::vector<vector3_t> getLinearAcceleration(const vector_t& state, const vector_t& input) const override;
  std::vector<vector3_t> getAngularAcceleration(const vector_t& state, const vector_t& input) const override;
  std::vector<vector6_t> getAccelerations(const vector_t& state, const vector_t& input) const override;

  std::vector<VectorFunctionLinearApproximation> getPositionLinearApproximation(const vector_t& state) const override;
  std::vector<VectorFunctionLinearApproximation> getVelocityLinearApproximation(const vector_t& state,
                                                                                const vector_t& input) const override;
  std::vector<VectorFunctionLinearApproximation> getOrientationErrorLinearApproximation(
      const vector_t& state, const std::vector<quaternion_t>& referenceOrientations) const override;
  std::vector<VectorFunctionLinearApproximation> getOrientationErrorWrtPlaneLinearApproximation(
      const vector_t& state, const std::vector<vector3_t>& planeNormals) const override;
  std::vector<VectorFunctionLinearApproximation> getAngularVelocityLinearApproximation(const vector_t& state,
                                                                                       const vector_t& input) const override;
  std::vector<VectorFunctionLinearApproximation> getTwistLinearApproximation(const vector_t& state, const vector_t& input) const override;
  std::vector<VectorFunctionLinearApproximation> getLinearAccelerationLinearApproximation(const vector_t& state,
                                                                                          const vector_t& input) const override;
  std::vector<VectorFunctionLinearApproximation> getAngularAccelerationLinearApproximation(const vector_t& state,
                                                                                           const vector_t& input) const override;
  std::vector<VectorFunctionLinearApproximation> getAccelerationsLinearApproximation(const vector_t& state,
                                                                                     const vector_t& input) const override;

  // Expose auto differentiation functions publicly to allow for inclusion in gauss-newton costs
  ad_vector_t getPositionCppAd(const ad_vector_t& state);
  ad_vector_t getVelocityCppAd(const ad_vector_t& state, const ad_vector_t& input);
  ad_vector_t getOrientationCppAd(const ad_vector_t& state);
  ad_vector_t getOrientationErrorCppAd(const ad_vector_t& state, const ad_vector_t& params);
  ad_vector_t getOrientationErrorWrtPlaneCppAd(const ad_vector_t& state, const ad_vector_t& params);
  ad_vector_t getAngularVelocityCppAd(const ad_vector_t& state, const ad_vector_t& input);
  ad_vector_t getTwistCppAd(const ad_vector_t& state, const ad_vector_t& input);
  ad_vector_t getLinearAccelerationCppAd(const ad_vector_t& state, const ad_vector_t& input);
  ad_vector_t getAngularAccelerationCppAd(const ad_vector_t& state, const ad_vector_t& input);
  ad_vector_t getAccelerationsCppAd(const ad_vector_t& state, const ad_vector_t& input);

 private:
  PinocchioEndEffectorDynamicsCppAd(const PinocchioEndEffectorDynamicsCppAd& rhs);

  std::unique_ptr<CppAdInterface> positionCppAdInterfacePtr_;
  std::unique_ptr<CppAdInterface> velocityCppAdInterfacePtr_;
  std::unique_ptr<CppAdInterface> orientationCppAdInterfacePtr_;
  std::unique_ptr<CppAdInterface> orientationErrorCppAdInterfacePtr_;
  std::unique_ptr<CppAdInterface> orientationErrorWrtPlaneCppAdInterfacePtr_;
  std::unique_ptr<CppAdInterface> angularVelocityCppAdInterfacePtr_;
  std::unique_ptr<CppAdInterface> twistCppAdInterfacePtr_;
  std::unique_ptr<CppAdInterface> linearAccelerationCppAdInterfacePtr_;
  std::unique_ptr<CppAdInterface> angularAccelerationCppAdInterfacePtr_;
  std::unique_ptr<CppAdInterface> accelerationsCppAdInterfacePtr_;

  const std::vector<std::string> endEffectorIds_;
  std::vector<size_t> endEffectorFrameIds_;

  PinocchioInterfaceCppAd pinocchioInterfaceCppAd_;
  WBAccelMpcRobotModel<ad_scalar_t>* mappingPtr_;
};

}  // namespace ocs2::humanoid
