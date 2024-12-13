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

#include "humanoid_common_mpc/contact/ContactCenterPoint.h"

namespace ocs2::humanoid {

/// \brief the maximum extension of the polygon with respect to the specified frame

struct PolygonBounds {
  PolygonBounds(
      const scalar_t& x_min, const scalar_t& x_max, const scalar_t& y_min, const scalar_t& y_max, const scalar_t& scaleFactor = 1.0)
      : x_min(x_min * scaleFactor), x_max(x_max * scaleFactor), y_min(y_min * scaleFactor), y_max(y_max * scaleFactor){};

  scalar_t x_min;
  scalar_t x_max;
  scalar_t y_min;
  scalar_t y_max;
};

///
/// \brief A planar contact polygon defined by the convex hull spanned up by a set of corner points expressed in the
/// frame name specified
///
/// \param[in] polygonPoints A vector of 3D points on the xy plane (z = 0)
/// \param[in] frameName name of pinocchio frame the polygon is specified in
/// \param[in] scaleFactor A factor to shrink or extent the polygon
///

class ContactPolygon {
 public:
  ContactPolygon(const std::vector<vector3_t>& polygonPoints,
                 const ContactCenterPoint& contactCenterPoint,
                 const scalar_t& scaleFactor = 1.0);

  size_t getNumberOfContactPoints() const { return polygonPoints_.size(); };
  vector3_t getContactPointTranslation(int index) const { return vector3_t(polygonPoints_[index][0], polygonPoints_[index][1], 0.0); };
  matrix3_t getContactPointTranslationCrossProductMatrix(int index) const;
  const std::string& getParentJointName() const { return contactCenterPoint_.parentJointName; };
  const std::string& getPolygonPointFrameName(int i) const { return polygonPointFrameNames_[i]; };
  const ContactCenterPoint& getContactCenterPoint() const { return contactCenterPoint_; };
  const PolygonBounds& getBounds() const { return polygonLimits_; };

 protected:
  // constructor can be used by child class. Child class has to ensure that the polygonLimits are correct
  ContactPolygon(const std::vector<vector3_t>& polygonPoints,
                 const PolygonBounds& polygonBounds,
                 const ContactCenterPoint& contactCenterPoint);

 private:
  // contact dimensions relative to frame attached at contact surface
  PolygonBounds polygonLimits_;
  std::vector<vector3_t> polygonPoints_;
  std::vector<std::string> polygonPointFrameNames_;
  const ContactCenterPoint contactCenterPoint_;
};  // namespace ContactPolygon

}  // namespace ocs2::humanoid
