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

#include "humanoid_common_mpc/HumanoidCostConstraintFactory.h"

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/penalties/Penalties.h>
#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/constraint/StateInputConstraint.h>
#include <ocs2_core/cost/QuadraticStateCost.h>
#include <ocs2_core/penalties/penalties/PieceWisePolynomialBarrierPenalty.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>

#include <humanoid_common_mpc/constraint/FrictionForceConeConstraint.h>
#include <humanoid_common_mpc/constraint/ZeroWrenchConstraint.h>
#include <humanoid_common_mpc/contact/ContactRectangle.h>
#include <humanoid_common_mpc/cost/StateInputQuadraticCost.h>
#include <humanoid_common_mpc/pinocchio_model/pinocchioUtils.h>
#include "humanoid_common_mpc/constraint/ContactMomentXYConstraintCppAd.h"
#include "humanoid_common_mpc/constraint/FootCollisionConstraint.h"
#include "humanoid_common_mpc/constraint/JointLimitsSoftConstraint.h"
#include "humanoid_common_mpc/cost/ExternalTorqueQuadraticCostAD.h"

namespace ocs2::humanoid {

HumanoidCostConstraintFactory::HumanoidCostConstraintFactory(const std::string& taskFile,
                                                             const std::string& referenceFile,
                                                             const SwitchedModelReferenceManager& referenceManager,
                                                             const PinocchioInterface& pinocchioInterface,
                                                             const MpcRobotModelBase<scalar_t>& mpcRobotModel,
                                                             const MpcRobotModelBase<ad_scalar_t>& mpcRobotModelAD,
                                                             const ModelSettings& modelSettings,
                                                             bool verbose)
    : taskFile_(taskFile),
      referenceFile_(referenceFile),
      referenceManagerPtr_(&referenceManager),
      pinocchioInterfacePtr_(&pinocchioInterface),
      mpcRobotModelPtr_(&mpcRobotModel),
      mpcRobotModelADPtr_(&mpcRobotModelAD),
      modelSettings_(modelSettings),
      verbose_(verbose) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputCost> HumanoidCostConstraintFactory::getStateInputQuadraticCost() const {
  matrix_t Q(mpcRobotModelADPtr_->getStateDim(), mpcRobotModelADPtr_->getStateDim());
  loadData::loadEigenMatrix(taskFile_, "Q", Q);
  matrix_t R(mpcRobotModelADPtr_->getInputDim(), mpcRobotModelADPtr_->getInputDim());
  loadData::loadEigenMatrix(taskFile_, "R", R);

  if (verbose_) {
    std::cerr << "\n #### Base Tracking Cost Coefficients: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "Q:\n" << Q << "\n";
    std::cerr << "R:\n" << R << "\n";
    std::cerr << " #### =============================================================================\n";
  }

  return std::unique_ptr<StateInputCost>(
      new StateInputQuadraticCost(std::move(Q), std::move(R), *referenceManagerPtr_, *pinocchioInterfacePtr_, *mpcRobotModelPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateCost> HumanoidCostConstraintFactory::getFootCollisionConstraint() const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);
  const std::string prefix = "collision_constraint.";

  FootCollisionConstraint::Config collisionConfig = FootCollisionConstraint::loadFootCollisionConstraintConfig(taskFile_, verbose_);
  PieceWisePolynomialBarrierPenalty::Config barrierPenaltyConfig;

  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose_);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose_);

  std::unique_ptr<PieceWisePolynomialBarrierPenalty> penalty(new PieceWisePolynomialBarrierPenalty(barrierPenaltyConfig));

  std::unique_ptr<FootCollisionConstraint> footCollisionConstraintPtr(
      new FootCollisionConstraint(*referenceManagerPtr_, *pinocchioInterfacePtr_, *mpcRobotModelADPtr_, std::move(collisionConfig),
                                  "FootCollisionConstraint", modelSettings_));

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(footCollisionConstraintPtr), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateCost> HumanoidCostConstraintFactory::getJointLimitsConstraint() const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);
  const std::string prefix = "jointLimits.";

  PieceWisePolynomialBarrierPenalty::Config barrierPenaltyConfig;

  if (verbose_) {
    std::cerr << "\n #### Joint Limit Barrier Penalty Config: ";
    std::cerr << "\n #### "
                 "============================================================="
                 "================\n";
  }
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose_);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose_);
  if (verbose_) {
    std::cerr << " #### "
                 "============================================================="
                 "================\n";
  }

  std::cout << "Initialized joint limits constraint with zero crossing cost " << barrierPenaltyConfig.getZeroCrossingValue() << "."
            << std::endl;

  std::pair<vector_t, vector_t> jointLimits = readPinocchioJointLimits(*pinocchioInterfacePtr_, mpcRobotModelPtr_->modelSettings);

  return std::unique_ptr<StateCost>(new JointLimitsSoftConstraint(jointLimits, barrierPenaltyConfig, *mpcRobotModelPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputCost> HumanoidCostConstraintFactory::getContactMomentXYConstraint(size_t contactPointIndex,
                                                                                            const std::string& name) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);
  const std::string prefix = "contacts.contactMomentXYSoftConstraint.";

  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose_);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose_);

  std::unique_ptr<ContactMomentXYConstraintCppAd> contactMomentXYConstraintPtr(new ContactMomentXYConstraintCppAd(
      *referenceManagerPtr_, ContactRectangle::loadContactRectangle(taskFile_, mpcRobotModelPtr_->modelSettings, contactPointIndex),
      contactPointIndex, *pinocchioInterfacePtr_, *mpcRobotModelADPtr_, name, modelSettings_));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(contactMomentXYConstraintPtr), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputConstraint> HumanoidCostConstraintFactory::getZeroWrenchConstraint(size_t contactPointIndex) const {
  return std::unique_ptr<StateInputConstraint>(new ZeroWrenchConstraint(*referenceManagerPtr_, contactPointIndex, *mpcRobotModelPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputCost> HumanoidCostConstraintFactory::getFrictionForceConeConstraint(size_t contactPointIndex) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile_, pt);
  const std::string prefix = "contacts.frictionForceConeSoftConstraint.";

  scalar_t frictionCoefficient = 1.0;
  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (verbose_) {
    std::cerr << "\n #### Friction Cone Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, frictionCoefficient, prefix + "frictionCoefficient", verbose_);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", verbose_);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", verbose_);
  if (verbose_) {
    std::cerr << " #### =============================================================================\n";
  }

  FrictionForceConeConstraint::Config frictionConeConConfig(frictionCoefficient);
  std::unique_ptr<FrictionForceConeConstraint> frictionForceConeConstraintPtr(
      new FrictionForceConeConstraint(*referenceManagerPtr_, std::move(frictionConeConConfig), contactPointIndex, *mpcRobotModelPtr_));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(frictionForceConeConstraintPtr), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateCost> HumanoidCostConstraintFactory::getTerminalCost() const {
  scalar_t terminalCostScaling;
  loadData::loadCppDataType<scalar_t>(taskFile_, "terminalCostScaling", terminalCostScaling);
  std::cout << "terminalCostScaling: " << terminalCostScaling << std::endl;

  matrix_t Qf(mpcRobotModelPtr_->getStateDim(), mpcRobotModelPtr_->getStateDim());
  loadData::loadEigenMatrix(taskFile_, "Q_final", Qf);
  Qf *= terminalCostScaling;
  if (verbose_) std::cerr << "Q_final:\n" << Qf << std::endl;
  return std::unique_ptr<StateCost>(new QuadraticStateCost(Qf));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

std::unique_ptr<StateInputCost> HumanoidCostConstraintFactory::getExternalTorqueQuadraticCost(size_t contactPointIndex) const {
  std::string fieldName;
  if (contactPointIndex == 0) {
    fieldName = "left_leg_torque_cost.";
  }
  if (contactPointIndex == 1) {
    fieldName = "right_leg_torque_cost.";
  }
  ExternalTorqueQuadraticCostAD::Config config = ExternalTorqueQuadraticCostAD::loadConfigFromFile(taskFile_, fieldName, verbose_);
  return std::make_unique<ExternalTorqueQuadraticCostAD>(contactPointIndex, config, *referenceManagerPtr_, *pinocchioInterfacePtr_,
                                                         *mpcRobotModelADPtr_, modelSettings_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

}  // namespace ocs2::humanoid