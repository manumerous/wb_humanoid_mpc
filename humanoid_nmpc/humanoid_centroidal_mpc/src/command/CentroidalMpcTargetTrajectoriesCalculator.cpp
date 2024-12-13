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

#include "humanoid_centroidal_mpc/command/CentroidalMpcTargetTrajectoriesCalculator.h"

#include <boost/proto/proto_fwd.hpp>
#include <cmath>

#include <pinocchio/algorithm/center-of-mass.hpp>

#include <ocs2_core/misc/LoadData.h>
#include "ocs2_centroidal_model/ModelHelperFunctions.h"

namespace ocs2::humanoid {

CentroidalMpcTargetTrajectoriesCalculator::CentroidalMpcTargetTrajectoriesCalculator(const std::string& referenceFile,
                                                                                     const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                                                                     PinocchioInterface pinocchioInterface,
                                                                                     const CentroidalModelInfo& info,
                                                                                     scalar_t mpcHorizon)
    : TargetTrajectoriesCalculatorBase(referenceFile, mpcRobotModel, mpcHorizon),
      pinocchioInterface_(pinocchioInterface),
      info_(info),
      mass_(pinocchio::computeTotalMass(pinocchioInterface.getModel())) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

TargetTrajectories CentroidalMpcTargetTrajectoriesCalculator::commandedPositionToTargetTrajectories(const vector4_t& commadLinePoseTarget,
                                                                                                    scalar_t initTime,
                                                                                                    const vector_t& initState) {
  vector_t currentPoseTarget = getCurrentBasePoseTarget(initState);

  const vector_t targetPose = getDeltaBaseTarget(commadLinePoseTarget, currentPoseTarget);

  scalar_t targetReachingTime = initTime + estimateTimeToTarget(targetPose - currentPoseTarget);

  // desired time trajectory
  const scalar_array_t timeTrajectory{initTime, targetReachingTime};

  // desired state trajectory
  vector_array_t stateTrajectory(2, vector_t::Zero(mpcRobotModelPtr_->getStateDim()));
  stateTrajectory[0] << vector_t::Zero(6), currentPoseTarget, targetJointState_;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, targetJointState_;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(mpcRobotModelPtr_->getInputDim()));

  TargetTrajectories targetTrajectories{timeTrajectory, stateTrajectory, inputTrajectory};

  return targetTrajectories;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

TargetTrajectories CentroidalMpcTargetTrajectoriesCalculator::commandedVelocityToTargetTrajectories(const vector4_t& commandedVelocities,
                                                                                                    scalar_t initTime,
                                                                                                    const vector_t& initState) {
  // This function constructs a target trajectory that interpolates between the current momentum and desired momentum for the first part
  // of the horizon while applying the desired momentum fully to the latter half. All position targets are obtained integrating said
  // velocity profile.

  vector_t currentPoseTarget = getCurrentBasePoseTarget(initState);

  vector4_t commVelTargetGlobal = filterAndTransformVelCommandToLocal(commandedVelocities, currentPoseTarget(3), 0.8);

  /////////////////////////
  // Intermediate Target //
  /////////////////////////

  vector6_t targetBaseTwist;
  targetBaseTwist << commVelTargetGlobal[0], commVelTargetGlobal[1], 0.0, 0.0, 0.0, commVelTargetGlobal[3];

  updateCentroidalDynamics(pinocchioInterface_, info_, mpcRobotModelPtr_->getGeneralizedCoordinates(initState));
  const Eigen::Matrix<scalar_t, 6, Eigen::Dynamic>& A = getCentroidalMomentumMatrix(pinocchioInterface_);

  vector6_t targetMomentum;

  const Eigen::Matrix<scalar_t, 6, 6> Ab = A.template leftCols<6>();
  const Eigen::Matrix<scalar_t, 6, 6> Ab_inv = computeFloatingBaseCentroidalMomentumMatrixInverse(Ab);

  // This did not lead to meaningful commands around the z axis. Needs more investigations.
  // targetMomentum = (Ab * targetBaseTwist);
  // targetMomentum[2] = 0.0;
  // targetMomentum[3] = 0.0;
  // targetMomentum[4] = 0.0;

  targetMomentum << commVelTargetGlobal[0], commVelTargetGlobal[1], 0.0, 0.0, 0.0, commVelTargetGlobal[3] / mass_;

  // Comput base velocity from centroidal momentum, this assumes no joint velocities.
  vector6_t baseVel = Ab_inv * initState.head(6);
  vector3_t averageVel;
  averageVel(0) = (baseVel[0] + commVelTargetGlobal[0]) / 2;
  averageVel(1) = (baseVel[1] + commVelTargetGlobal[1]) / 2;
  averageVel(2) = (baseVel[5] + commVelTargetGlobal[3]) / 2;

  currentPoseTarget[2] = commVelTargetGlobal[2];
  scalar_t intermediateTargetTime = 0.7 * mpcHorizon_;
  vector6_t intermediateTargetPose = integrateTargetBasePose(currentPoseTarget, averageVel, commVelTargetGlobal(2), intermediateTargetTime);

  //////////////////
  // Final Target //
  //////////////////

  averageVel(0) = (commVelTargetGlobal[0]);
  averageVel(1) = (commVelTargetGlobal[1]);
  averageVel(2) = (commVelTargetGlobal[3]);

  vector6_t finalTargetPose =
      integrateTargetBasePose(intermediateTargetPose, averageVel, commVelTargetGlobal(2), (mpcHorizon_ - intermediateTargetTime));

  // desired time trajectory
  const scalar_array_t timeTrajectory{initTime, initTime + intermediateTargetTime, initTime + mpcHorizon_};

  // desired state trajectory
  vector_array_t stateTrajectory(3, vector_t::Zero(mpcRobotModelPtr_->getStateDim()));
  stateTrajectory[0] << targetMomentum, currentPoseTarget, targetJointState_;
  stateTrajectory[1] << targetMomentum, intermediateTargetPose, targetJointState_;
  stateTrajectory[2] << targetMomentum, finalTargetPose, targetJointState_;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(3, vector_t::Zero(mpcRobotModelPtr_->getInputDim()));

  TargetTrajectories targetTrajectories{timeTrajectory, stateTrajectory, inputTrajectory};

  return targetTrajectories;
}

}  // namespace ocs2::humanoid
