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

#include <ocs2_robotic_tools/common/RotationTransforms.h>

#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

template <typename SCALAR_T>
using VECTOR18_T = Eigen::Matrix<SCALAR_T, 18, 1>;
template <typename SCALAR_T>
using VECTOR19_T = Eigen::Matrix<SCALAR_T, 19, 1>;

struct EndEffectorDynamicsWeights {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Cost weights [w_x,w_y,w_z]
  vector3_t contactPositionErrorWeight{30.0, 30.0, 30.0};
  vector3_t contactOrientationErrorWeight{30.0, 30.0, 30.0};
  vector3_t contactLinearVelocityErrorWeight{0.1, 0.1, 0.1};
  vector3_t contactAngularVelocityErrorWeight{0.1, 0.1, 0.1};
  vector3_t contactLinearAccelerationErrorWeight{0.01, 0.01, 0.01};
  vector3_t contactAngularAccelerationErrorWeight{0.01, 0.01, 0.01};

  VECTOR18_T<scalar_t> toVector();

  static EndEffectorDynamicsWeights getWeights(const std::string& taskFile, const std::string prefix, bool verbose);
};

template <typename SCALAR_T>
struct EndEffectorDynamicsCostElement {
  EndEffectorDynamicsCostElement(const VECTOR19_T<SCALAR_T>& vector) : costElementVector(vector){};
  EndEffectorDynamicsCostElement() : costElementVector(VECTOR19_T<SCALAR_T>::Zero()){};
  VECTOR19_T<SCALAR_T> costElementVector;

  VECTOR19_T<SCALAR_T> getValues() { return costElementVector; };

  VECTOR3_T<SCALAR_T> getPosition() const { return costElementVector.head(3); }
  QUATERNION_T<SCALAR_T> getOrientation() const { return QUATERNION_T<SCALAR_T>(VECTOR4_T<SCALAR_T>(costElementVector.segment(3, 4))); }
  VECTOR3_T<SCALAR_T> getLinearVelocity() const { return costElementVector.segment(7, 3); }
  VECTOR3_T<SCALAR_T> getAngularVelocity() const { return costElementVector.segment(10, 3); }
  VECTOR3_T<SCALAR_T> getLinearAcceleration() const { return costElementVector.segment(13, 3); }
  VECTOR3_T<SCALAR_T> getAngularAcceleration() const { return costElementVector.segment(16, 3); }

  void setPosition(const VECTOR3_T<SCALAR_T>& position) { costElementVector.head(3) = position; };
  void setOrientation(const VECTOR4_T<SCALAR_T>& orientation) { costElementVector.segment(3, 4) = orientation; };
  void setOrientation(const QUATERNION_T<SCALAR_T>& orientation) { costElementVector.segment(3, 4) = orientation.coeffs(); };
  void setLinearVelocity(const VECTOR3_T<SCALAR_T>& linearVelocity) { costElementVector.segment(7, 3) = linearVelocity; };
  void setAngularVelocity(const VECTOR3_T<SCALAR_T>& angularVelocity) { costElementVector.segment(10, 3) = angularVelocity; };
  void setLinearAcceleration(const VECTOR3_T<SCALAR_T>& linearAcceleration) { costElementVector.segment(13, 3) = linearAcceleration; };
  void setAngularAcceleration(const VECTOR3_T<SCALAR_T>& angularAcceleration) { costElementVector.segment(16, 3) = angularAcceleration; };
};

template <typename SCALAR_T>
VECTOR18_T<SCALAR_T> computeTaskSpaceErrors(const EndEffectorDynamicsCostElement<SCALAR_T>& current,
                                            const EndEffectorDynamicsCostElement<SCALAR_T>& reference);

template <typename SCALAR_T>
struct PlanarEndEffectorDynamicsReference {
  PlanarEndEffectorDynamicsReference(const VECTOR18_T<SCALAR_T>& vector) : costElementVector(vector){};
  PlanarEndEffectorDynamicsReference() : costElementVector(VECTOR18_T<SCALAR_T>::Zero()){};
  VECTOR18_T<SCALAR_T> costElementVector;

  VECTOR18_T<SCALAR_T> getValues() { return costElementVector; };

  VECTOR3_T<SCALAR_T> getPosition() const { return costElementVector.head(3); }
  VECTOR3_T<SCALAR_T> getPlaneNormal() const { return costElementVector.segment(3, 3); }
  VECTOR3_T<SCALAR_T> getLinearVelocity() const { return costElementVector.segment(6, 3); }
  VECTOR3_T<SCALAR_T> getAngularVelocity() const { return costElementVector.segment(9, 3); }
  VECTOR3_T<SCALAR_T> getLinearAcceleration() const { return costElementVector.segment(12, 3); }
  VECTOR3_T<SCALAR_T> getAngularAcceleration() const { return costElementVector.segment(15, 3); }

  void setPosition(const VECTOR3_T<SCALAR_T>& position) { costElementVector.head(3) = position; };
  void setPlaneNormal(const VECTOR3_T<SCALAR_T>& planeNormal) { costElementVector.segment(3, 3) = planeNormal; };
  void setLinearVelocity(const VECTOR3_T<SCALAR_T>& linearVelocity) { costElementVector.segment(6, 3) = linearVelocity; };
  void setAngularVelocity(const VECTOR3_T<SCALAR_T>& angularVelocity) { costElementVector.segment(9, 3) = angularVelocity; };
  void setLinearAcceleration(const VECTOR3_T<SCALAR_T>& linearAcceleration) { costElementVector.segment(12, 3) = linearAcceleration; };
  void setAngularAcceleration(const VECTOR3_T<SCALAR_T>& angularAcceleration) { costElementVector.segment(15, 3) = angularAcceleration; };
};

}  // namespace ocs2::humanoid