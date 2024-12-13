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

#include "humanoid_centroidal_mpc/common/CentroidalMpcRobotModel.h"
#include "humanoid_centroidal_mpc/dynamics/DynamicsHelperFunctions.h"
#include "humanoid_centroidal_mpc/test/CentroidalTestingModelInterface.h"
#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/Types.h"
#include "humanoid_common_mpc/pinocchio_model/DynamicsHelperFunctions.h"

#include "ocs2_centroidal_model/ModelHelperFunctions.h"

namespace ocs2::humanoid {

TEST(TestDynamicsHelperFunctions, computeContactCoP) {
  CentroidalTestingModelInterface testingModelInterface = CentroidalTestingModelInterface();

  PinocchioInterface pinocchioInterface = testingModelInterface.getPinocchioInterface();
  vector_t q = vector_t::Zero(6 + testingModelInterface.modelSettings.mpc_joint_dim);
  q[2] = 0.86566;
  q.tail(testingModelInterface.modelSettings.mpc_joint_dim) = vector_t::Random(testingModelInterface.modelSettings.mpc_joint_dim);

  vector_t input = vector_t::Zero(INPUT_DIM);
  testingModelInterface.getMpcRobotModel().setContactForce(input, vector3_t(0, 0, 100), 0);
  testingModelInterface.getMpcRobotModel().setContactForce(input, vector3_t(50, 30, 100), 1);

  std::vector<vector3_t> contactPositions = computeContactPositions<scalar_t>(q, pinocchioInterface);
  std::vector<vector3_t> contactCoPs = computeContactsCoP(input, pinocchioInterface, {1, 1}, testingModelInterface.getMpcRobotModel());

  std::cout << "contactCoP[0]; " << contactCoPs[0].transpose() << std::endl;
  std::cout << "contactCoP[1]; " << contactCoPs[1].transpose() << std::endl;

  EXPECT_TRUE(contactPositions[0].isApprox(contactCoPs[0]));
  EXPECT_TRUE(contactPositions[1].isApprox(contactCoPs[1]));
}

// These tests are disabled since they change with model updates

// TEST(TestDynamicsHelperFunctions, computeContactPositions) {
//   CentroidalTestingModelInterface testingModelInterface = CentroidalTestingModelInterface();

//   PinocchioInterface pinocchioInterface = testingModelInterface.getPinocchioInterface();
//   vector_t q = vector_t::Zero(BASE_DIM + testingModelInterface.modelSettings.mpc_joint_dim);
//   q[2] = 0.8415;

//   std::vector<vector3_t> contactPositions = computeContactPositions<scalar_t>(q, pinocchioInterface);

//   std::cout << "contactPositions[0]; " << contactPositions[0].transpose() << std::endl;
//   std::cout << "contactPositions[1]; " << contactPositions[1].transpose() << std::endl;

//   EXPECT_TRUE(contactPositions[0].isApprox(vector3_t(0.04286, 0.0895, 0.0), 1e-3));
//   EXPECT_TRUE(contactPositions[1].isApprox(vector3_t(0.04286, -0.0895, 0.0), 1e-3));

//   q[0] = -1.0;

//   contactPositions = computeContactPositions<scalar_t>(q, pinocchioInterface);

//   std::cout << "contactPositions[0]; " << contactPositions[0].transpose() << std::endl;
//   std::cout << "contactPositions[1]; " << contactPositions[1].transpose() << std::endl;

//   EXPECT_TRUE(contactPositions[0].isApprox(vector3_t(-0.95714, 0.0895, 0.0), 1e-3));
//   EXPECT_TRUE(contactPositions[1].isApprox(vector3_t(-0.95714, -0.0895, 0.0), 1e-3));
// }

TEST(TestDynamicsHelperFunctions, weightCompensatingInput) {
  CentroidalTestingModelInterface testingModelInterface = CentroidalTestingModelInterface();

  PinocchioInterface pinocchioInterface = testingModelInterface.getPinocchioInterface();
  CentroidalModelInfo centroidalModelInfo = testingModelInterface.getCentroidalModelInfo(pinocchioInterface);

  const static scalar_t totalGravitationalForce = centroidalModelInfo.robotMass * 9.81;

  vector_t inputDoubleContact = vector_t::Zero(INPUT_DIM);
  testingModelInterface.getMpcRobotModel().setContactForce(inputDoubleContact, vector3_t(0, 0, totalGravitationalForce / 2), 0);
  testingModelInterface.getMpcRobotModel().setContactForce(inputDoubleContact, vector3_t(0, 0, totalGravitationalForce / 2), 1);
  EXPECT_TRUE(
      inputDoubleContact.isApprox(weightCompensatingInput(centroidalModelInfo, {true, true}, testingModelInterface.getMpcRobotModel())));
  EXPECT_TRUE(
      inputDoubleContact.isApprox(weightCompensatingInput(pinocchioInterface, {true, true}, testingModelInterface.getMpcRobotModel())));

  vector_t inputLeftContact = vector_t::Zero(INPUT_DIM);
  testingModelInterface.getMpcRobotModel().setContactForce(inputLeftContact, vector3_t(0, 0, totalGravitationalForce), 0);
  EXPECT_TRUE(
      inputLeftContact.isApprox(weightCompensatingInput(centroidalModelInfo, {true, false}, testingModelInterface.getMpcRobotModel())));
  EXPECT_TRUE(
      inputLeftContact.isApprox(weightCompensatingInput(pinocchioInterface, {true, false}, testingModelInterface.getMpcRobotModel())));

  vector_t inputRightContact = vector_t::Zero(INPUT_DIM);
  testingModelInterface.getMpcRobotModel().setContactForce(inputRightContact, vector3_t(0, 0, totalGravitationalForce), 1);
  EXPECT_TRUE(
      inputRightContact.isApprox(weightCompensatingInput(centroidalModelInfo, {false, true}, testingModelInterface.getMpcRobotModel())));
  EXPECT_TRUE(
      inputRightContact.isApprox(weightCompensatingInput(pinocchioInterface, {false, true}, testingModelInterface.getMpcRobotModel())));

  EXPECT_TRUE(
      ocs2::getNormalizedCentroidalMomentumRate(pinocchioInterface, centroidalModelInfo, inputDoubleContact).isApprox(vector_t::Zero(6)));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

}  // namespace ocs2::humanoid
