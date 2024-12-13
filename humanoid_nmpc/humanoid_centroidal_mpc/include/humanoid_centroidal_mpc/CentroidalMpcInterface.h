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

#include <ocs2_core/Types.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_sqp/SqpSettings.h>

#include "humanoid_centroidal_mpc/common/CentroidalMpcRobotModel.h"
#include "humanoid_centroidal_mpc/initialization/CentroidalWeightCompInitializer.h"
#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid {

class CentroidalMpcInterface final : public RobotInterface {
 public:
  /**
   * Constructor
   *
   * @throw Invalid argument error if input task file or urdf file does not exist.
   *
   * @param [in] taskFile: The absolute path to the configuration file for the MPC.
   * @param [in] urdfFile: The absolute path to the URDF file for the robot.
   * @param [in] referenceFile: The absolute path to the reference configuration file.
   */
  CentroidalMpcInterface(const std::string& taskFile,
                         const std::string& urdfFile,
                         const std::string& referenceFile,
                         const std::string& gaitFile,
                         bool setupOCP = true);

  ~CentroidalMpcInterface() override = default;

  const OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }

  // CAREFUL: This function is not const, so it can easily be abused. It is currently only for gui purposes. Use with care!
  OptimalControlProblem& getOptimalControlProblemRef() const { return *problemPtr_; }

  const ModelSettings& modelSettings() const { return modelSettings_; }
  const ddp::Settings& ddpSettings() const { return ddpSettings_; }
  const mpc::Settings& mpcSettings() const { return mpcSettings_; }
  const rollout::Settings& rolloutSettings() const { return rolloutSettings_; }
  const sqp::Settings& sqpSettings() const { return sqpSettings_; }

  const vector_t& getInitialState() const { return initialState_; }
  const RolloutBase& getRollout() const { return *rolloutPtr_; }
  PinocchioInterface& getPinocchioInterface() { return *pinocchioInterfacePtr_; }
  const CentroidalModelInfo& getCentroidalModelInfo() const { return centroidalModelInfo_; }
  std::shared_ptr<SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const { return referenceManagerPtr_; }

  const CentroidalWeightCompInitializer& getInitializer() const override { return *initializerPtr_; }
  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

  const CentroidalMpcRobotModel<scalar_t>& getMpcRobotModel() const { return *mpcRobotModelPtr_; }
  const CentroidalMpcRobotModel<ad_scalar_t>& getMpcRobotModelAD() const { return *mpcRobotModelADPtr_; }

  std::vector<std::string> getCostNames() const;
  std::vector<std::string> getTerminalCostNames() const;
  std::vector<std::string> getStateSoftConstraintNames() const;
  std::vector<std::string> getSoftConstraintNames() const;
  std::vector<std::string> getEqualityConstraintNames() const;

 private:
  void setupOptimalControlProblem();

  std::unique_ptr<StateInputConstraint> getStanceFootConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                size_t contactPointIndex);
  std::unique_ptr<StateInputConstraint> getNormalVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                    size_t contactPointIndex);
  std::unique_ptr<StateInputConstraint> getJointMimicConstraint(size_t mimicIndex);

  void addTaskSpaceKinematicsCosts(const CentroidalModelPinocchioMappingCppAd& pinocchioMappingCppAd,
                                   const PinocchioEndEffectorKinematicsCppAd::update_pinocchio_interface_callback& velocityUpdateCallback);

  ModelSettings modelSettings_;
  ddp::Settings ddpSettings_;
  mpc::Settings mpcSettings_;
  sqp::Settings sqpSettings_;

  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
  CentroidalModelInfo centroidalModelInfo_;

  std::unique_ptr<OptimalControlProblem> problemPtr_;
  std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

  std::unique_ptr<CentroidalMpcRobotModel<scalar_t>> mpcRobotModelPtr_;
  std::unique_ptr<CentroidalMpcRobotModel<ad_scalar_t>> mpcRobotModelADPtr_;

  rollout::Settings rolloutSettings_;
  std::unique_ptr<RolloutBase> rolloutPtr_;
  std::unique_ptr<CentroidalWeightCompInitializer> initializerPtr_;

  vector_t initialState_;

  const std::string taskFile_;
  const std::string urdfFile_;
  const std::string referenceFile_;
  bool verbose_;
};

}  // namespace ocs2::humanoid
