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

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

#include "humanoid_centroidal_mpc/common/CentroidalMpcRobotModel.h"
#include "humanoid_common_mpc/pinocchio_model/createPinocchioModel.h"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace ocs2::humanoid {
struct CentroidalTestingModelInterface {
 public:
  std::string taskFile;
  std::string urdfFile;
  std::string referenceFile;

  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr;
  std::unique_ptr<ModelSettings> modelSettingsPtr;
  std::unique_ptr<CentroidalMpcRobotModel<scalar_t>> mpcRobotModelPtr_;
  std::unique_ptr<CentroidalMpcRobotModel<ad_scalar_t>> mpcRobotModelADPtr_;

  CentroidalTestingModelInterface() {
    taskFile = ament_index_cpp::get_package_share_directory("g1_centroidal_mpc") + "/config/mpc/task.info";

    urdfFile = ament_index_cpp::get_package_share_directory("g1_description") + "/urdf/g1_29dof.urdf";

    referenceFile = ament_index_cpp::get_package_share_directory("g1_centroidal_mpc") + "/config/command/reference.info";

    modelSettingsPtr.reset(new ModelSettings(taskFile, urdfFile, "centroidal_testing_interfce", false));

    pinocchioInterfacePtr.reset(new PinocchioInterface(createCustomPinocchioInterface(taskFile, urdfFile, *modelSettingsPtr)));
    mpcRobotModelPtr_.reset(new CentroidalMpcRobotModel<scalar_t>(*modelSettingsPtr, *pinocchioInterfacePtr, getCentroidalModelInfo()));
    mpcRobotModelADPtr_.reset(new CentroidalMpcRobotModel<ad_scalar_t>(*modelSettingsPtr, (*pinocchioInterfacePtr).toCppAd(),
                                                                       getCentroidalModelInfo().toCppAd()));
  }

  PinocchioInterface& getPinocchioInterface() const { return *pinocchioInterfacePtr; }

  CentroidalMpcRobotModel<scalar_t>& getMpcRobotModel() { return *mpcRobotModelPtr_; }
  CentroidalMpcRobotModel<ad_scalar_t>& getMpcRobotModelAD() { return *mpcRobotModelADPtr_; }

  const ModelSettings& getModelSettings() const { return *modelSettingsPtr; }

  CentroidalModelInfo getCentroidalModelInfo() const {
    return centroidal_model::createCentroidalModelInfo(
        *pinocchioInterfacePtr, centroidal_model::loadCentroidalType(taskFile),
        centroidal_model::loadDefaultJointState(pinocchioInterfacePtr->getModel().nq - 6, referenceFile),
        modelSettingsPtr->contactNames3DoF, modelSettingsPtr->contactNames6DoF);
  }
};
}  // namespace ocs2::humanoid