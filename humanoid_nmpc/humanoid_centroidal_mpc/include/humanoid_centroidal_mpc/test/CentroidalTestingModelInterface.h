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
  ModelSettings modelSettings(taskFile, urdfFile, "centroidal_testing_interfce", "false");

  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr;
  std::unique_ptr<CentroidalMpcRobotModel<scalar_t>> mpcRobotModelPtr_;
  std::unique_ptr<CentroidalMpcRobotModel<ad_scalar_t>> mpcRobotModelADPtr_;

  CentroidalTestingModelInterface() {
    const std::string path(__FILE__);
    const std::string humanoid_centroidal_mpc_install_dir = ament_index_cpp::get_package_share_directory("humanoid_centroidal_mpc");
    taskFile = humanoid_centroidal_mpc_install_dir + "/config/mpc/task.info";
    urdfFile = robot_definitions::URDF_FILE_PATH;
    referenceFile = humanoid_centroidal_mpc_install_dir + "/config/command/reference.info";

    pinocchioInterfacePtr.reset(
        new PinocchioInterface(createCustomPinocchioInterface(taskFile, urdfFile, modelSettings.mpcModelJointNames)));
    mpcRobotModelPtr_.reset(new CentroidalMpcRobotModel<scalar_t>(*pinocchioInterfacePtr, getCentroidalModelInfo(*pinocchioInterfacePtr)));
    mpcRobotModelADPtr_.reset(new CentroidalMpcRobotModel<ad_scalar_t>((*pinocchioInterfacePtr).toCppAd(),
                                                                       getCentroidalModelInfo(*pinocchioInterfacePtr).toCppAd()));
  }

  PinocchioInterface& getPinocchioInterface() const { return *pinocchioInterfacePtr; }

  CentroidalMpcRobotModel<scalar_t>& getMpcRobotModel() const { return *mpcRobotModelPtr_; }
  CentroidalMpcRobotModel<ad_scalar_t>& getMpcRobotModelAD() const { return *mpcRobotModelADPtr_; }

  CentroidalModelInfo getCentroidalModelInfo(const PinocchioInterface& pinocchioInterface) const {
    return centroidal_model::createCentroidalModelInfo(
        pinocchioInterface, centroidal_model::loadCentroidalType(taskFile),
        centroidal_model::loadDefaultJointState(pinocchioInterface.getModel().nq - 6, referenceFile), modelSettings.contactNames3DoF,
        modelSettings.contactNames6DoF);
  }
};
}  // namespace ocs2::humanoid