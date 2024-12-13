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

#include <string>
#include <utility>
#include <vector>

#include <ocs2_core/Types.h>

#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

namespace ocs2 {

/** The Kinematics function which maps state-input pair to the end-effector (position, velocity, orientation error) */
template <typename SCALAR_T>
class EndEffectorDynamics : public EndEffectorKinematics<SCALAR_T> {
 public:
  using vector3_t = Eigen::Matrix<SCALAR_T, 3, 1>;
  using matrix3x_t = Eigen::Matrix<SCALAR_T, 3, Eigen::Dynamic>;
  using vector6_t = Eigen::Matrix<SCALAR_T, 6, 1>;
  using matrix6x_t = Eigen::Matrix<SCALAR_T, 6, Eigen::Dynamic>;
  using vector_t = Eigen::Matrix<SCALAR_T, Eigen::Dynamic, 1>;
  using quaternion_t = Eigen::Quaternion<SCALAR_T>;

  EndEffectorDynamics() = default;
  virtual ~EndEffectorDynamics() = default;
  virtual EndEffectorDynamics* clone() const = 0;
  EndEffectorDynamics& operator=(const EndEffectorDynamics&) = delete;

  /**
   * Get linear end-effector accelerations in world frame
   *
   * @param [in] state: state vector
   * @param [in] input: input vector
   * @return array of velocity vectors
   */
  virtual std::vector<vector3_t> getLinearAcceleration(const vector_t& state, const vector_t& input) const = 0;

  /**
   * Get end-effector angular accelerations in world frame
   *
   * @param [in] state vector
   * @param [in] input: input vector
   * @return array of angular velocities
   */
  virtual std::vector<vector3_t> getAngularAcceleration(const vector_t& state, const vector_t& input) const = 0;

  /**
   * Get end-effector linear & angular accelerations in world frame
   *
   * @param [in] state vector
   * @param [in] input: input vector
   * @return array of twists
   */
  virtual std::vector<vector6_t> getAccelerations(const vector_t& state, const vector_t& input) const = 0;

  /**
   * Get linear end-effector accelerations linear approximation in world frame
   *
   * @param [in] state: state vector
   * @param [in] input: input vector
   * @return array of velocity function linear approximations
   */
  virtual std::vector<VectorFunctionLinearApproximation> getLinearAccelerationLinearApproximation(const vector_t& state,
                                                                                                  const vector_t& input) const = 0;

  /**
   * GGet end-effector angular accelerations linear approximation in world frame
   *
   * @param [in] state: state vector
   * @param [in] input: input vector
   * @return array of velocity function linear approximations
   */
  virtual std::vector<VectorFunctionLinearApproximation> getAngularAccelerationLinearApproximation(const vector_t& state,
                                                                                                   const vector_t& input) const = 0;

  /**
   * Get end-effector linear & angular accelerations linear approximation in world frame
   *
   * @param [in] state: state vector
   * @param [in] input: input vector
   * @return array of twist function linear approximations
   */
  virtual std::vector<VectorFunctionLinearApproximation> getAccelerationsLinearApproximation(const vector_t& state,
                                                                                             const vector_t& input) const = 0;
};

}  // namespace ocs2
