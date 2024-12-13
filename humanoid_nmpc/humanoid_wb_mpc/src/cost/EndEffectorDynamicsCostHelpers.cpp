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

#include "humanoid_wb_mpc/cost/EndEffectorDynamicsCostHelpers.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

namespace ocs2::humanoid {

VECTOR18_T<scalar_t> EndEffectorDynamicsWeights::toVector() {
  VECTOR18_T<scalar_t> weightVector;
  weightVector << contactPositionErrorWeight, contactOrientationErrorWeight, contactLinearVelocityErrorWeight,
      contactAngularVelocityErrorWeight, contactLinearAccelerationErrorWeight, contactAngularAccelerationErrorWeight;
  return weightVector;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

EndEffectorDynamicsWeights EndEffectorDynamicsWeights::getWeights(const std::string& taskFile, const std::string prefix, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);

  // Load all weights
  scalar_t pos_x = 0;
  scalar_t pos_y = 0;
  scalar_t pos_z = 0;
  scalar_t orientation_x = 0;
  scalar_t orientation_y = 0;
  scalar_t orientation_z = 0;
  scalar_t lin_velocity_x = 0;
  scalar_t lin_velocity_y = 0;
  scalar_t lin_velocity_z = 0;
  scalar_t ang_velocity_x = 0;
  scalar_t ang_velocity_y = 0;
  scalar_t ang_velocity_z = 0;
  scalar_t lin_acceleration_x = 0;
  scalar_t lin_acceleration_y = 0;
  scalar_t lin_acceleration_z = 0;
  scalar_t ang_acceleration_x = 0;
  scalar_t ang_acceleration_y = 0;
  scalar_t ang_acceleration_z = 0;
  if (verbose) {
    std::cerr << "\n #### Humanoid End Effector Foot Cost Weights: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, pos_x, prefix + "pos_x", verbose);
  loadData::loadPtreeValue(pt, pos_y, prefix + "pos_y", verbose);
  loadData::loadPtreeValue(pt, pos_z, prefix + "pos_z", verbose);
  loadData::loadPtreeValue(pt, orientation_x, prefix + "orientation_x", verbose);
  loadData::loadPtreeValue(pt, orientation_y, prefix + "orientation_y", verbose);
  loadData::loadPtreeValue(pt, orientation_z, prefix + "orientation_z", verbose);
  loadData::loadPtreeValue(pt, lin_velocity_x, prefix + "lin_velocity_x", verbose);
  loadData::loadPtreeValue(pt, lin_velocity_y, prefix + "lin_velocity_y", verbose);
  loadData::loadPtreeValue(pt, lin_velocity_z, prefix + "lin_velocity_z", verbose);
  loadData::loadPtreeValue(pt, ang_velocity_x, prefix + "ang_velocity_x", verbose);
  loadData::loadPtreeValue(pt, ang_velocity_y, prefix + "ang_velocity_y", verbose);
  loadData::loadPtreeValue(pt, ang_velocity_z, prefix + "ang_velocity_z", verbose);
  loadData::loadPtreeValue(pt, lin_acceleration_x, prefix + "lin_acceleration_x", verbose);
  loadData::loadPtreeValue(pt, lin_acceleration_y, prefix + "lin_acceleration_y", verbose);
  loadData::loadPtreeValue(pt, lin_acceleration_z, prefix + "lin_acceleration_z", verbose);
  loadData::loadPtreeValue(pt, ang_acceleration_x, prefix + "ang_acceleration_x", verbose);
  loadData::loadPtreeValue(pt, ang_acceleration_y, prefix + "ang_acceleration_y", verbose);
  loadData::loadPtreeValue(pt, ang_acceleration_z, prefix + "ang_acceleration_z", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  EndEffectorDynamicsWeights weights;

  weights.contactPositionErrorWeight = vector3_t(pos_x, pos_y, pos_z);
  weights.contactOrientationErrorWeight = vector3_t(orientation_x, orientation_y, orientation_z);
  weights.contactLinearVelocityErrorWeight = vector3_t(lin_velocity_x, lin_velocity_y, lin_velocity_z);
  weights.contactAngularVelocityErrorWeight = vector3_t(ang_velocity_x, ang_velocity_y, ang_velocity_z);
  weights.contactLinearVelocityErrorWeight = vector3_t(lin_acceleration_x, lin_acceleration_y, lin_acceleration_z);
  weights.contactAngularVelocityErrorWeight = vector3_t(ang_acceleration_x, ang_acceleration_y, ang_acceleration_z);

  return weights;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

template <typename SCALAR_T>
VECTOR18_T<SCALAR_T> computeTaskSpaceErrors(const EndEffectorDynamicsCostElement<SCALAR_T>& current,
                                            const EndEffectorDynamicsCostElement<SCALAR_T>& reference) {
  const VECTOR3_T<SCALAR_T> orientationError = quaternionDistance<SCALAR_T>(current.getOrientation(), reference.getOrientation());

  VECTOR18_T<SCALAR_T> errors;
  errors << (current.getPosition() - reference.getPosition()), orientationError,
      (current.getLinearVelocity() - reference.getLinearVelocity()), (current.getAngularVelocity() - reference.getAngularVelocity()),
      (current.getLinearAcceleration() - reference.getLinearAcceleration()),
      (current.getAngularAcceleration() - reference.getAngularAcceleration());
  return errors;
}
template VECTOR18_T<scalar_t> computeTaskSpaceErrors(const EndEffectorDynamicsCostElement<scalar_t>& current,
                                                     const EndEffectorDynamicsCostElement<scalar_t>& reference);
template VECTOR18_T<ad_scalar_t> computeTaskSpaceErrors(const EndEffectorDynamicsCostElement<ad_scalar_t>& current,
                                                        const EndEffectorDynamicsCostElement<ad_scalar_t>& reference);

}  // namespace ocs2::humanoid
