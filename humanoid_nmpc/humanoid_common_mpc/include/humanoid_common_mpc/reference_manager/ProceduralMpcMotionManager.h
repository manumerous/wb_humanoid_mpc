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

#pragma once

#include <functional>
#include <string>
#include <vector>

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_mpc/SystemObservation.h>

#include <humanoid_common_mpc/command/WalkingVelocityCommand.h>
#include <humanoid_common_mpc/gait/GaitSchedule.h>
#include <humanoid_common_mpc/gait/ModeSequenceTemplate.h>

#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include "humanoid_common_mpc/common/MpcRobotModelBase.h"
#include "humanoid_common_mpc/reference_manager/BreakFrequencyAlphaFilter.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid {

class ProceduralMpcMotionManager : public SolverSynchronizedModule {
 public:
  using VelocityTargetToTargetTrajectories =
      std::function<TargetTrajectories(const vector4_t& velocityTarget, scalar_t initTime, scalar_t finalTime, const vector_t& initState)>;

  struct GaitModeStateConfig {
    std::string gaitCommand = "stance";
    scalar_t minLinVelCmd;
    scalar_t maxLinVelCmd;
    scalar_t minAngVelCmd;
    scalar_t maxAngVelCmd;
    scalar_t linVelErrorThresh;
    scalar_t angVelErrorThresh;
  };

  /**
   * Constructor
   *
   * @param [in] gaitFile: The file path that contains the different gait patterns.
   * @param [in] referenceFile: The file path containing the default references and velocity limits.
   * @param [in] velocityTargetToTargetTrajectories: A function which transforms the commanded velocities to TargetTrajectories.
   * @param [in] switchedModelReferenceManagerPtr: A pointer to the switched model reference manager used to update gait and references
   */
  ProceduralMpcMotionManager(const std::string& gaitFile,
                             const std::string& referenceFile,
                             std::shared_ptr<SwitchedModelReferenceManager> switchedModelReferenceManagerPtr,
                             const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                             VelocityTargetToTargetTrajectories velocityTargetToTargetTrajectories);

  ProceduralMpcMotionManager(const ProceduralMpcMotionManager& mpcMotionManager) = delete;

  /**
   * Method called right before the solver runs
   *
   * @param initTime : start time of the MPC horizon
   * @param finalTime : Final time of the MPC horizon
   * @param initState : State at the start of the MPC horizon
   * @param referenceManager : The ReferenceManager which manages both ModeSchedule and TargetTrajectories.
   */
  void preSolverRun(scalar_t initTime,
                    scalar_t finalTime,
                    const vector_t& initState,
                    const ReferenceManagerInterface& referenceManager) override;

  /**
   * Method called right after the solver runs
   *
   * @param primalSolution : primalSolution
   */
  void postSolverRun(const PrimalSolution& primalSolution) override {};

  virtual void setAndScaleVelocityCommand(const WalkingVelocityCommand& rawVelocityCommand);

  static bool transitionToFasterGait(const vector4_t& velCommandVec, const vector6_t& baseVelocity, const GaitModeStateConfig& cfg);

  static bool transitionToSlowerGait(const vector4_t& velCommandVec, const vector6_t& baseVelocity, const GaitModeStateConfig& cfg);

 protected:
  // clang-format off
  const std::vector<GaitModeStateConfig> gaitModeStates_ {
    { "stance",       -0.1,  0.1, -0.1,  0.1,  10.0,   10.0 }, // Large threshold allows switching aw3ay from stance purely command based. 
    { "slow_walk",     0.05,  0.3,  0.05,  0.2,    0.05,  0.05},
    { "walk",          0.25,  0.5, 0.15,  0.35,    0.05,  0.05},
    { "slower_trot",   0.45, 0.7, 0.3,  0.55,      0.1,  0.1},
    { "slow_trot",     0.65, 0.9,  0.5,  0.7,      0.2,  0.2},
    { "trot",          0.8,  1.3,  0.65,  10.0,    0.2,  0.2},
    { "run",           1.2,  10.0,  0.65,  10.0,   0.2,  0.2}  
  };  // clang-format on

  size_t currentGaitMode_{0};

  virtual WalkingVelocityCommand getScaledWalkingVelocityCommand() { return velocityCommand_; }

  WalkingVelocityCommand scaleWalkingVelocityCommand(const WalkingVelocityCommand& rawVelocityCommand) const;

  std::shared_ptr<SwitchedModelReferenceManager> switchedModelReferenceManagerPtr_;
  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  const MpcRobotModelBase<scalar_t>* mpcRobotModelPtr_;

  const vector_t targetCommandLimits_;
  VelocityTargetToTargetTrajectories velocityTargetToTargetTrajectoriesFun_;

  std::vector<std::string> gaitList_;
  std::map<std::string, ModeSequenceTemplate> gaitMap_;

  // For velocity control mode
  scalar_t maxDisplacementVelocityX_ = 0.6;
  scalar_t maxDisplacementVelocityY_ = 0.3;
  scalar_t maxDeltaPelvisHeight_ = 0.3;
  scalar_t maxRotationVelocity_ = 0.6;

  BreakFrequencyAlphaFilter velocityCommandFilter;
  WalkingVelocityCommand velocityCommand_;

  std::string currentGaitCommand_{"stance"};
  std::string lastGaitCommand_{"stance"};
  scalar_t lastGaitChangeTime_{0.0};
};

}  // namespace ocs2::humanoid
