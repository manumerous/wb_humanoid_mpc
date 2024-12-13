
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

#include "humanoid_common_mpc_ros2/visualization/EquivalentContactCornerForcesVisualizer.h"

#include <ocs2_ros2_interfaces/visualization/VisualizationColors.h>
#include <ocs2_ros2_interfaces/visualization/VisualizationHelpers.h>

#include <humanoid_common_mpc/common/ModelSettings.h>
#include <humanoid_common_mpc/contact/ContactRectangle.h>
#include <humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h>
#include <humanoid_common_mpc/pinocchio_model/PinocchioFrameConversions.h>

namespace ocs2::humanoid {

EquivalentContactCornerForcesVisualizer::EquivalentContactCornerForcesVisualizer(const std::string& taskFile,
                                                                                 const PinocchioInterface& pinocchioInterface,
                                                                                 const MpcRobotModelBase<scalar_t>& mpcRobotModel)
    : pinocchioInterfacePtr_(&pinocchioInterface),
      contactFrameIndizes_(getContactFrameIndices(pinocchioInterface, mpcRobotModel)),
      mpcRobotModelPtr_(&mpcRobotModel) {
  contactMappers.reserve(N_CONTACTS);
  polygonPointFrameIndizes_.reserve(N_CONTACTS);
  for (int i = 0; i < N_CONTACTS; i++) {
    ContactWrenchMapper<N_CONTACT_POLYGON_POINTS> contactWrenchMapper(
        ContactRectangle::loadContactRectangle(taskFile, mpcRobotModel.modelSettings, i));
    contactMappers.emplace_back(contactWrenchMapper);
    std::array<pinocchio::FrameIndex, N_CONTACT_POLYGON_POINTS> polygonFootFrameIndizes;
    for (int j = 0; j < N_CONTACT_POLYGON_POINTS; j++) {
      polygonFootFrameIndizes[j] =
          pinocchioInterface.getModel().getFrameId(contactWrenchMapper.contactPolygon_.getPolygonPointFrameName(j));
    }
    polygonPointFrameIndizes_.emplace_back(polygonFootFrameIndizes);
  }
}

visualization_msgs::msg::MarkerArray EquivalentContactCornerForcesVisualizer::generateContactVisualizationForceMarkers(
    const vector_t& input, const contact_flag_t& contactFlags, const scalar_t& forceScale) const {
  visualization_msgs::msg::MarkerArray markerArray;
  markerArray.markers.reserve(N_CONTACTS * N_CONTACT_POLYGON_POINTS);
  const pinocchio::Model& model = pinocchioInterfacePtr_->getModel();
  const pinocchio::Data& data = pinocchioInterfacePtr_->getData();
  for (int i = 0; i < N_CONTACTS; i++) {
    if (contactFlags[i]) {
      // Convert wrench into set of equivalent visualization forces in each corner
      const ContactWrenchMapper<N_CONTACT_POLYGON_POINTS>& currContactWrenchMapper = contactMappers[i];
      vector6_t globalWrench = mpcRobotModelPtr_->getContactWrench(input, i);
      vector6_t localContactWrench = rotateVectorWorldToLocal(globalWrench, data, contactFrameIndizes_[i]);
      auto visualizationForces = currContactWrenchMapper.computeVisualizationForceArray(localContactWrench);
      // Fill marker array with visualization forces
      for (int j = 0; j < N_CONTACT_POLYGON_POINTS; j++) {
        vector3_t footPosition = data.oMf[polygonPointFrameIndizes_[i][j]].translation();
        markerArray.markers.emplace_back(getForceMarker(rotateVectorLocalToWorld(visualizationForces[j], data, contactFrameIndizes_[i]),
                                                        footPosition, contactFlags[i], Color::blue, forceScale,
                                                        "equivalentContactCornerForces"));
      }
    } else {
      for (int j = 0; j < N_CONTACT_POLYGON_POINTS; j++) {
        vector3_t footPosition = data.oMf[polygonPointFrameIndizes_[i][j]].translation();
        markerArray.markers.emplace_back(getForceMarker(vector3_t::Zero(), vector3_t::Zero(), contactFlags[i], Color::blue, forceScale,
                                                        "equivalentContactCornerForces"));
      }
    }
  }
  return markerArray;
}

}  // namespace ocs2::humanoid