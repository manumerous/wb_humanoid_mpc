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

#include "ocs2_centroidal_model/AccessHelperFunctions.h"
#include "ocs2_centroidal_model/ModelHelperFunctions.h"
#include "ocs2_robotic_tools/common/RotationDerivativesTransforms.h"

#include "humanoid_centroidal_mpc/test/CentroidalTestingModelInterface.h"
#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

static const std::string WARNING_MESSAGE =
    " \n These tests are to ensure consistency with the Whole Body Controller. \n If tests here fail then it is likely that the urdf has "
    "been "
    "updated, the robot mass has changed, or the input and state size has been "
    "updated. \n Ensure both WBC and MPC are consistent";

static const Eigen::Matrix<scalar_t, STATE_DIM, 1> STATE = (Eigen::Matrix<scalar_t, STATE_DIM, 1>() << 0.680375,
                                                            -0.211234,
                                                            0.566198,
                                                            1.5708,
                                                            0.785398,
                                                            -0.785398,
                                                            -0.329554,
                                                            0.536459,
                                                            -0.444451,
                                                            0.10794,
                                                            -0.0452059,
                                                            0.257742,
                                                            -0.270431,
                                                            0.0268018,
                                                            0.904459,
                                                            0.83239,
                                                            0.271423,
                                                            0.434594,
                                                            -0.716795,
                                                            0.213938,
                                                            -0.967399,
                                                            -0.514226,
                                                            -0.725537,
                                                            0.608354,
                                                            -0.686642,
                                                            -0.198111,
                                                            -0.740419,
                                                            -0.782382,
                                                            0.997849,
                                                            -0.563486,
                                                            0.0258648,
                                                            0.678224,
                                                            0.22528,
                                                            -0.407937,
                                                            0.275105,
                                                            0.0485744,
                                                            -0.012834,
                                                            0.94555,
                                                            -0.414966,
                                                            0.542715)
                                                               .finished();

static const Eigen::Matrix<scalar_t, INPUT_DIM, 1> INPUT = (Eigen::Matrix<scalar_t, INPUT_DIM, 1>() << 0.05349,
                                                            0.539828,
                                                            -0.199543,
                                                            0.783059,
                                                            -0.433371,
                                                            -0.295083,
                                                            0.615449,
                                                            0.838053,
                                                            -0.860489,
                                                            0.898654,
                                                            0.0519907,
                                                            -0.827888,
                                                            -0.615572,
                                                            0.326454,
                                                            0.780465,
                                                            -0.302214,
                                                            -0.871657,
                                                            -0.959954,
                                                            -0.0845965,
                                                            -0.873808,
                                                            -0.52344,
                                                            0.941268,
                                                            0.804416,
                                                            0.70184,
                                                            -0.466669,
                                                            0.0795207,
                                                            -0.249586,
                                                            0.520497,
                                                            0.0250707,
                                                            0.335448,
                                                            0.0632129,
                                                            -0.921439,
                                                            -0.124725,
                                                            0.86367,
                                                            0.86162,
                                                            0.441905,
                                                            -0.431413,
                                                            0.477069,
                                                            0.279958,
                                                            -0.291903)
                                                               .finished();

static const vector3_t expectedLinearVelocity(0.62015, 2.74312, 1.66311);
static const vector3_t expectedAngularVelocity(33.1713, 19.5390, -54.0927);
static const vector3_t expectedLinearMomentumRate(0.6689, 1.3779, -431.8019);
static const vector3_t expectedAngularMomentumRate(2.42452, -0.18036, -0.68041);

TEST(TestCentroidalConversions, numberOfStateInputVariables) {
  EXPECT_EQ(INPUT.size(), 40) << "40 input variables are expected" << WARNING_MESSAGE;
  EXPECT_EQ(STATE.size(), 40) << "40 state variables are expected" << WARNING_MESSAGE;
}

TEST(TestCentroidalConversions, computePelvisTwist) {
  CentroidalTestingModelInterface testingModelInterface = CentroidalTestingModelInterface();

  PinocchioInterface pinocchioInterface = testingModelInterface.getPinocchioInterface();
  CentroidalModelInfo centroidalModelInfo = testingModelInterface.getCentroidalModelInfo(pinocchioInterface);
  CentroidalModelPinocchioMapping centroidalModelPinocchioMapping(centroidalModelInfo);
  centroidalModelPinocchioMapping.setPinocchioInterface(pinocchioInterface);

  vector_t qPinocchio = centroidalModelPinocchioMapping.getPinocchioJointPosition(STATE);
  updateCentroidalDynamics(pinocchioInterface, centroidalModelInfo, qPinocchio);
  vector_t jointVelocities = centroidalModelPinocchioMapping.getPinocchioJointVelocity(STATE, INPUT);

  vector3_t eulerAngleZYX = qPinocchio.segment(3, 3);

  vector_t pelvisTwist = jointVelocities.head(6);
  vector_t pelvisLinearVelocity = pelvisTwist.head(3);
  vector_t pelvisAngularVelocityEulerZYX = pelvisTwist.tail(3);

  vector_t globalAngularVelocity =
      getGlobalAngularVelocityFromEulerAnglesZyxDerivatives<scalar_t>(eulerAngleZYX, pelvisAngularVelocityEulerZYX);

  EXPECT_TRUE(pelvisLinearVelocity.isApprox(expectedLinearVelocity, 1e-1))
      << "expected " << expectedLinearVelocity.transpose() << '\n'
      << "computed " << pelvisLinearVelocity.transpose() << WARNING_MESSAGE;

  EXPECT_TRUE(globalAngularVelocity.isApprox(expectedAngularVelocity, 1e-1))
      << "expected " << expectedAngularVelocity.transpose() << '\n'
      << "computed " << globalAngularVelocity.tail(3).transpose() << WARNING_MESSAGE;
}

TEST(TestCentroidalConversions, computeMomentumRateOfChange) {
  CentroidalTestingModelInterface testingModelInterface = CentroidalTestingModelInterface();

  PinocchioInterface pinocchioInterface = testingModelInterface.getPinocchioInterface();
  CentroidalModelInfo centroidalModelInfo = testingModelInterface.getCentroidalModelInfo(pinocchioInterface);
  CentroidalModelPinocchioMapping centroidalModelPinocchioMapping(centroidalModelInfo);
  centroidalModelPinocchioMapping.setPinocchioInterface(pinocchioInterface);

  vector_t qPinocchio = centroidalModelPinocchioMapping.getPinocchioJointPosition(STATE);
  updateCentroidalDynamics(pinocchioInterface, centroidalModelInfo, qPinocchio);

  Eigen::Matrix<scalar_t, 6, 1> centroidalMomentumRate =
      centroidalModelInfo.robotMass * getNormalizedCentroidalMomentumRate<scalar_t>(pinocchioInterface, centroidalModelInfo, INPUT);

  vector3_t linearMomentumRate = centroidalMomentumRate.head(3);
  vector3_t angularMomentumRate = centroidalMomentumRate.tail(3);

  EXPECT_TRUE(linearMomentumRate.isApprox(expectedLinearMomentumRate, 1e-1))
      << "expected " << expectedLinearMomentumRate.transpose() << '\n'
      << "computed " << linearMomentumRate.transpose() << WARNING_MESSAGE;

  EXPECT_TRUE(angularMomentumRate.isApprox(expectedAngularMomentumRate, 1e-1))
      << "expected " << expectedAngularMomentumRate.transpose() << '\n'
      << "computed " << angularMomentumRate.transpose() << WARNING_MESSAGE;
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace ocs2::humanoid
