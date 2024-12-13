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

#include <gtest/gtest.h>
#include <vector>

#include "humanoid_centroidal_mpc/test/CentroidalTestingModelInterface.h"
#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"
#include "humanoid_common_mpc/pinocchio_model/PinocchioFrameConversions.h"

#include "humanoid_centroidal_mpc/common/CentroidalMpcRobotModel.h"

namespace ocs2::humanoid {

class TestPinocchioFrameConversions : public ::testing::Test {
 protected:
  CentroidalTestingModelInterface testingModelInterface = CentroidalTestingModelInterface();
  PinocchioInterface pinocchioInterface = testingModelInterface.getPinocchioInterface();
  const CentroidalMpcRobotModel<scalar_t>& mpcRobotModel = testingModelInterface.getMpcRobotModel();
  vector_t q = vector_t::Zero(mpcRobotModel.getGenCoordinatesDim());

  void SetUp() override {
    q = vector_t::Zero(mpcRobotModel.getGenCoordinatesDim());
    q[2] = 0.8415;
  }

  void TearDown() override {
    // This will be called after each test
  }
};

TEST_F(TestPinocchioFrameConversions, rotateVectorLocalToWorld3D) {
  updateFramePlacements<scalar_t>(q, pinocchioInterface);

  for (const pinocchio::FrameIndex& contactIndex : getContactFrameIndices(pinocchioInterface)) {
    vector3_t testVector = vector3_t::Random();
    EXPECT_TRUE(testVector.isApprox(rotateVectorLocalToWorld<scalar_t>(testVector, pinocchioInterface.getData(), contactIndex)));
  }

  scalar_t kneeAngle = 0.5;
  q[6] = kneeAngle;

  matrix3_t R_l_w = getRotationMatrixFromZyxEulerAngles<scalar_t>(vector3_t(0.0, kneeAngle, 0.0));
  updateFramePlacements<scalar_t>(q, pinocchioInterface);

  for (const pinocchio::FrameIndex& contactIndex : getContactFrameIndices(pinocchioInterface)) {
    vector3_t testVector = vector3_t::Random();
    vector3_t testVectorRotated = R_l_w * testVector;
    EXPECT_TRUE(testVectorRotated.isApprox(rotateVectorLocalToWorld<scalar_t>(testVector, pinocchioInterface.getData(), contactIndex)));
  }
}

TEST_F(TestPinocchioFrameConversions, rotateVectorWorldToLocal3D) {
  updateFramePlacements<scalar_t>(q, pinocchioInterface);

  for (const pinocchio::FrameIndex& contactIndex : getContactFrameIndices(pinocchioInterface)) {
    vector3_t testVector = vector3_t::Random();
    EXPECT_TRUE(testVector.isApprox(rotateVectorWorldToLocal<scalar_t>(testVector, pinocchioInterface.getData(), contactIndex)));
  }

  scalar_t kneeAngle = 0.5;
  q[6] = kneeAngle;

  matrix3_t R_l_w = getRotationMatrixFromZyxEulerAngles<scalar_t>(vector3_t(0.0, -kneeAngle, 0.0));
  updateFramePlacements<scalar_t>(q, pinocchioInterface);

  for (const pinocchio::FrameIndex& contactIndex : getContactFrameIndices(pinocchioInterface)) {
    vector3_t testVector = vector3_t::Random();
    vector3_t testVectorRotated = R_l_w * testVector;
    EXPECT_TRUE(testVectorRotated.isApprox(rotateVectorWorldToLocal<scalar_t>(testVector, pinocchioInterface.getData(), contactIndex)));
  }
}

TEST_F(TestPinocchioFrameConversions, backAndForthVector3D) {
  for (int i = 0; i < 25; ++i) {
    vector_t q = vector_t::Random(mpcRobotModel.getGenCoordinatesDim());
    q[2] = 0.88;

    updateFramePlacements<scalar_t>(q, pinocchioInterface);

    for (const pinocchio::FrameIndex& contactIndex : getContactFrameIndices(pinocchioInterface)) {
      vector3_t testVector = vector3_t::Random();
      EXPECT_TRUE(testVector.isApprox(
          rotateVectorWorldToLocal<scalar_t>(rotateVectorLocalToWorld<scalar_t>(testVector, pinocchioInterface.getData(), contactIndex),
                                             pinocchioInterface.getData(), contactIndex)));
    }
  }
}

TEST_F(TestPinocchioFrameConversions, rotateVectorLocalToWorld6D) {
  updateFramePlacements<scalar_t>(q, pinocchioInterface);

  for (const pinocchio::FrameIndex& contactIndex : getContactFrameIndices(pinocchioInterface)) {
    vector6_t testVector = vector6_t::Random();
    EXPECT_TRUE(testVector.isApprox(rotateVectorLocalToWorld<scalar_t>(testVector, pinocchioInterface.getData(), contactIndex)));
  }

  scalar_t kneeAngle = 0.5;
  q[6] = kneeAngle;

  matrix3_t R_l_w = getRotationMatrixFromZyxEulerAngles<scalar_t>(vector3_t(0.0, kneeAngle, 0.0));
  updateFramePlacements<scalar_t>(q, pinocchioInterface);

  for (const pinocchio::FrameIndex& contactIndex : getContactFrameIndices(pinocchioInterface)) {
    vector6_t testVector = vector6_t::Random();
    vector6_t testVectorRotated(6);
    testVectorRotated << R_l_w * testVector.head(3), R_l_w * testVector.tail(3);
    EXPECT_TRUE(testVectorRotated.isApprox(rotateVectorLocalToWorld<scalar_t>(testVector, pinocchioInterface.getData(), contactIndex)));
  }
}

TEST_F(TestPinocchioFrameConversions, rotateVectorWorldToLocal6D) {
  updateFramePlacements<scalar_t>(q, pinocchioInterface);

  for (const pinocchio::FrameIndex& contactIndex : getContactFrameIndices(pinocchioInterface)) {
    vector6_t testVector = vector6_t::Random();
    EXPECT_TRUE(testVector.isApprox(rotateVectorWorldToLocal<scalar_t>(testVector, pinocchioInterface.getData(), contactIndex)));
  }

  scalar_t kneeAngle = 0.5;
  q[6] = kneeAngle;

  matrix3_t R_l_w = getRotationMatrixFromZyxEulerAngles<scalar_t>(vector3_t(0.0, -kneeAngle, 0.0));
  updateFramePlacements<scalar_t>(q, pinocchioInterface);

  for (const pinocchio::FrameIndex& contactIndex : getContactFrameIndices(pinocchioInterface)) {
    vector6_t testVector = vector6_t::Random();
    vector6_t testVectorRotated(6);
    testVectorRotated << R_l_w * testVector.head(3), R_l_w * testVector.tail(3);
    EXPECT_TRUE(testVectorRotated.isApprox(rotateVectorWorldToLocal<scalar_t>(testVector, pinocchioInterface.getData(), contactIndex)));
  }
}

TEST_F(TestPinocchioFrameConversions, backAndForthVector6D) {
  for (int i = 0; i < 25; ++i) {
    vector_t q = vector_t::Random(mpcRobotModel.getGenCoordinatesDim());
    q[2] = 0.88;

    updateFramePlacements<scalar_t>(q, pinocchioInterface);

    for (const pinocchio::FrameIndex& contactIndex : getContactFrameIndices(pinocchioInterface)) {
      vector6_t testVector = vector6_t::Random();
      EXPECT_TRUE(testVector.isApprox(
          rotateVectorWorldToLocal<scalar_t>(rotateVectorLocalToWorld<scalar_t>(testVector, pinocchioInterface.getData(), contactIndex),
                                             pinocchioInterface.getData(), contactIndex)));
    }
  }
}

// These tests are disabled since they change with model updates

// TEST_F(TestPinocchioFrameConversions, transformPointLocalToWorld) {
//   scalar_t kneeAngle = 0.5;
//   q[6] = kneeAngle;

//   updateFramePlacements<scalar_t>(q, pinocchioInterface);

//   vector3_t leftContactPosInWorldFrame(-0.36582, 0.08950, 0.12097);
//   EXPECT_TRUE(leftContactPosInWorldFrame.isApprox(
//       transformPointLocalToWorld<scalar_t>(vector3_t::Zero(), pinocchioInterface.getData(), getContactFrameIndex(pinocchioInterface, 0)),
//       1e-3));
//   vector3_t rightContactPosInWorldFrame(0.04286, -0.08950, 0.03850);
//   EXPECT_TRUE(rightContactPosInWorldFrame.isApprox(
//       transformPointLocalToWorld<scalar_t>(vector3_t::Zero(), pinocchioInterface.getData(), getContactFrameIndex(pinocchioInterface, 1)),
//       1e-3));
// }

// TEST_F(TestPinocchioFrameConversions, transformPointWorldToLocal) {
//   scalar_t kneeAngle = 0.5;
//   q[2] = 0.88;
//   q[6] = kneeAngle;
//   vector3_t zeroVec = vector3_t::Zero();

//   updateFramePlacements<scalar_t>(q, pinocchioInterface);

//   vector3_t leftContactPosInWorldFrame(-0.36582, 0.08950, 0.12097);
//   EXPECT_TRUE(transformPointWorldToLocal<scalar_t>(leftContactPosInWorldFrame, pinocchioInterface.getData(),
//                                                    getContactFrameIndex(pinocchioInterface, 0))
//                   .isZero(1e-3));
//   vector3_t rightContactPosInWorldFrame(0.04286, -0.08950, 0.03850);
//   EXPECT_TRUE(transformPointWorldToLocal<scalar_t>(rightContactPosInWorldFrame, pinocchioInterface.getData(),
//                                                    getContactFrameIndex(pinocchioInterface, 1))
//                   .isZero(1e-3));
// }

TEST_F(TestPinocchioFrameConversions, transformPointBackAndForth) {
  for (int i = 0; i < 25; ++i) {
    vector_t q = vector_t::Random(mpcRobotModel.getGenCoordinatesDim());
    q[2] = 0.88;

    updateFramePlacements<scalar_t>(q, pinocchioInterface);

    for (const pinocchio::FrameIndex& contactIndex : getContactFrameIndices(pinocchioInterface)) {
      vector3_t testPoint = vector3_t::Random();
      EXPECT_TRUE(testPoint.isApprox(
          transformPointLocalToWorld<scalar_t>(rotateVectorWorldToLocal<scalar_t>(testPoint, pinocchioInterface.getData(), contactIndex),
                                               pinocchioInterface.getData(), contactIndex)));
    }
  }
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace ocs2::humanoid
