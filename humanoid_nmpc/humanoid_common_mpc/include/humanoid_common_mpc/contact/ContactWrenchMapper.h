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

#include "humanoid_common_mpc/contact/ContactPolygon.h"

namespace ocs2::humanoid {

///
/// \brief Mapps between a set of forces, applied in each corner of the contact polygon, and an equivalent contact
/// wrench. In case of mapping from wrench to contact corner forces the set of orces will be computed s.t. it is equal
/// to the contact wrench and has the smallest sum of l2 norms.
///
/// \param[in] contactPolygon A contact polygon specifying all the corners of the polygon in the local foot frame.
///

template <int N_POLYGON_POINTS>
class ContactWrenchMapper {
 public:
  ContactWrenchMapper(const ContactPolygon& contactPolygon) : contactPolygon_(contactPolygon) {
    assert(N_POLYGON_POINTS == contactPolygon.getNumberOfContactPoints() &&
           "Number of contact points in contact polygon does not match with value specified to template");

    Eigen::Matrix<scalar_t, 6, 3 * N_POLYGON_POINTS> A = Eigen::Matrix<scalar_t, 6, 3 * N_POLYGON_POINTS>::Zero();
    for (int i = 0; i < N_POLYGON_POINTS; i++) {
      A.block(0, 3 * i, 3, 3) = matrix3_t::Identity();
      A.block(3, 3 * i, 3, 3) = contactPolygon.getContactPointTranslationCrossProductMatrix(i);
    }

    mapForcesToContactWrench_ = A;
    mapWrenchToVisualiazationForces_ = A.transpose() * (A * A.transpose()).inverse();
  }

  /// \brief Converts a contact wrench into an a set of equivalent contact forces at the corner of the contact polygon
  ///
  /// \param[in] contactWrench Expressed in the local contact frame
  ///
  /// \param[out] contactPointForceVec vector of 3D contact forces

  std::array<vector3_t, N_POLYGON_POINTS> computeVisualizationForceArray(const vector6_t& contactWrench) const {
    std::array<vector3_t, N_POLYGON_POINTS> contactPointForceVec;
    Eigen::Matrix<scalar_t, 3 * N_POLYGON_POINTS, 1> visualizationForces = mapWrenchToVisualiazationForces_ * contactWrench;
    for (int i = 0; i < N_POLYGON_POINTS; i++) {
      contactPointForceVec[i] << (visualizationForces.segment(3 * i, 3));
    }
    return contactPointForceVec;
  }

  const ContactPolygon contactPolygon_;

 private:
  Eigen::Matrix<scalar_t, 6, 3 * N_POLYGON_POINTS> mapForcesToContactWrench_;
  Eigen::Matrix<scalar_t, 3 * N_POLYGON_POINTS, 6> mapWrenchToVisualiazationForces_;
};

}  // namespace ocs2::humanoid
