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

#include "humanoid_common_mpc/constraint/FootCollisionConstraint.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

FootCollisionConstraint::FootCollisionConstraint(const SwitchedModelReferenceManager& referenceManager,
                                                 const PinocchioInterface& pinocchioInterface,
                                                 const MpcRobotModelBase<ad_scalar_t>& mpcRobotModel,
                                                 const Config& config,
                                                 std::string costName,
                                                 const ModelSettings& modelSettings)
    : StateConstraintCppAd(ConstraintOrder::Linear),
      referenceManagerPtr_(&referenceManager),
      pinocchioInterfaceCppAd_(pinocchioInterface.toCppAd()),
      mpcRobotModelPtr_(&mpcRobotModel),
      cfg_(std::move(config)) {
  initialize(mpcRobotModelPtr_->getStateDim(), 2, costName, modelSettings.modelFolderCppAd, modelSettings.recompileLibrariesCppAd,
             modelSettings.verboseCppAd);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

FootCollisionConstraint::FootCollisionConstraint(const FootCollisionConstraint& other)
    : StateConstraintCppAd(other),
      referenceManagerPtr_(other.referenceManagerPtr_),
      pinocchioInterfaceCppAd_(other.pinocchioInterfaceCppAd_),
      mpcRobotModelPtr_(other.mpcRobotModelPtr_),
      cfg_(other.cfg_),
      numConstraints_(other.numConstraints_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool FootCollisionConstraint::isActive(scalar_t time) const {
  if (!isActive_) return false;

  // Inactivate the constraint if both feet are in contact. Prevents it from fighting against the stance foot constraints.
  auto contactFlags = referenceManagerPtr_->getContactFlags(time);
  return !(contactFlags[0] && contactFlags[1]);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

ad_vector_t FootCollisionConstraint::constraintFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& parameters) const {
  const pinocchio::ReferenceFrame rf = pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED;

  const auto& model = pinocchioInterfaceCppAd_.getModel();
  auto data = pinocchioInterfaceCppAd_.getData();

  const ad_vector_t q = mpcRobotModelPtr_->getGeneralizedCoordinates(state);
  pinocchio::forwardKinematics(model, data, q);

  // Ankle collision points
  ad_vector3_t pos_ankle_l = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.leftAnkleFrame)).translation();
  ad_vector3_t pos_ankle_r = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.rightAnkleFrame)).translation();

  // Foot collision points
  ad_vector3_t pos_f_l = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.leftFootCenterFrame)).translation();
  ad_vector3_t pos_f_r = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.rightFootCenterFrame)).translation();
  ad_vector3_t pos_f_l_p1 = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.leftFootFrame1)).translation();
  ad_vector3_t pos_f_r_p1 = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.rightFootFrame1)).translation();
  ad_vector3_t pos_f_l_p2 = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.leftFootFrame2)).translation();
  ad_vector3_t pos_f_r_p2 = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.rightFootFrame2)).translation();

  // Knee collision points
  ad_vector3_t pos_k_l = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.leftKneeFrame)).translation();
  ad_vector3_t pos_k_r = pinocchio::updateFramePlacement(model, data, model.getFrameId(cfg_.rightKneeFrame)).translation();

  // Calcualte the square of the min distance (2*radius)^2
  // parameters[0] is the collision sphere radius
  ad_scalar_t minDistFoot = 2.0 * parameters[0];
  ad_scalar_t minDistKnee = 2.0 * parameters[1];

  ad_vector_t constraintValues(numConstraints_);
  constraintValues[0] = ((pos_f_l_p1 - pos_f_r_p1).norm() - minDistFoot);
  constraintValues[1] = ((pos_f_l_p1 - pos_f_r_p2).norm() - minDistFoot);
  constraintValues[2] = ((pos_f_l_p2 - pos_f_r_p1).norm() - minDistFoot);
  constraintValues[3] = ((pos_f_l_p2 - pos_f_r_p2).norm() - minDistFoot);

  constraintValues[4] = ((pos_f_l - pos_f_r_p1).norm() - minDistFoot);
  constraintValues[5] = ((pos_f_l - pos_f_r_p2).norm() - minDistFoot);
  constraintValues[6] = ((pos_f_r - pos_f_l_p1).norm() - minDistFoot);
  constraintValues[7] = ((pos_f_r - pos_f_l_p2).norm() - minDistFoot);
  constraintValues[8] = ((pos_f_l - pos_f_r).norm() - minDistFoot);

  constraintValues[9] = ((pos_k_l - pos_k_r).norm() - minDistKnee);

  constraintValues[10] = ((pos_f_l - pos_ankle_r).norm() - minDistFoot);
  constraintValues[11] = ((pos_f_l_p1 - pos_ankle_r).norm() - minDistFoot);
  constraintValues[12] = ((pos_f_l_p2 - pos_ankle_r).norm() - minDistFoot);
  constraintValues[13] = ((pos_f_r - pos_ankle_l).norm() - minDistFoot);
  constraintValues[14] = ((pos_f_r_p1 - pos_ankle_l).norm() - minDistFoot);
  constraintValues[15] = ((pos_f_r_p2 - pos_ankle_l).norm() - minDistFoot);

  return constraintValues;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

FootCollisionConstraint::Config FootCollisionConstraint::loadFootCollisionConstraintConfig(const std::string taskFile, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "collision_constraint.";

  Config collisionConfig;

  loadData::loadPtreeValue(pt, collisionConfig.leftAnkleFrame, prefix + "foot.leftAnkleFrame", verbose);
  loadData::loadPtreeValue(pt, collisionConfig.rightAnkleFrame, prefix + "foot.rightAnkleFrame", verbose);
  loadData::loadPtreeValue(pt, collisionConfig.footCollisionSphereRadius, prefix + "foot.footCollisionSphereRadius", verbose);
  loadData::loadPtreeValue(pt, collisionConfig.leftKneeFrame, prefix + "knee.leftKneeFrame", verbose);
  loadData::loadPtreeValue(pt, collisionConfig.rightKneeFrame, prefix + "knee.rightKneeFrame", verbose);
  loadData::loadPtreeValue(pt, collisionConfig.kneeCollisionSphereRadius, prefix + "knee.kneeCollisionSphereRadius", verbose);

  return collisionConfig;
}

}  // namespace ocs2::humanoid