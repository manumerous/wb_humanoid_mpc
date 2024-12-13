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

#include <array>
#include <cstddef>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>

namespace ocs2 {

template <typename SCALAR_T>
using VECTOR_T = Eigen::Matrix<SCALAR_T, -1, 1>;
template <typename SCALAR_T>
using VECTOR2_T = Eigen::Matrix<SCALAR_T, 2, 1>;
template <typename SCALAR_T>
using VECTOR3_T = Eigen::Matrix<SCALAR_T, 3, 1>;
template <typename SCALAR_T>
using VECTOR4_T = Eigen::Matrix<SCALAR_T, 4, 1>;
template <typename SCALAR_T>
using VECTOR6_T = Eigen::Matrix<SCALAR_T, 6, 1>;
template <typename SCALAR_T>
using VECTOR12_T = Eigen::Matrix<SCALAR_T, 12, 1>;
template <typename SCALAR_T>
using MATRIX_T = Eigen::Matrix<SCALAR_T, -1, -1>;
template <typename SCALAR_T>
using MATRIX3_T = Eigen::Matrix<SCALAR_T, 3, 3>;
template <typename SCALAR_T>
using MATRIX4_T = Eigen::Matrix<SCALAR_T, 4, 4>;
template <typename SCALAR_T>
using MATRIX6_T = Eigen::Matrix<SCALAR_T, 6, 6>;
template <typename SCALAR_T>
using QUATERNION_T = Eigen::Quaternion<SCALAR_T>;

using vector2_t = VECTOR2_T<scalar_t>;
using vector3_t = VECTOR3_T<scalar_t>;
using vector4_t = VECTOR4_T<scalar_t>;
using vector6_t = VECTOR6_T<scalar_t>;
using vector12_t = VECTOR12_T<scalar_t>;
using matrix3_t = MATRIX3_T<scalar_t>;
using matrix4_t = MATRIX4_T<scalar_t>;
using matrix6_t = MATRIX6_T<scalar_t>;
using quaternion_t = QUATERNION_T<scalar_t>;

using ad_vector2_t = VECTOR2_T<ad_scalar_t>;
using ad_vector3_t = VECTOR3_T<ad_scalar_t>;
using ad_vector4_t = VECTOR4_T<ad_scalar_t>;
using ad_vector6_t = VECTOR6_T<ad_scalar_t>;
using ad_vector12_t = VECTOR12_T<ad_scalar_t>;
using ad_matrix3_t = MATRIX3_T<ad_scalar_t>;
using ad_matrix4_t = MATRIX4_T<ad_scalar_t>;
using ad_matrix6_t = MATRIX6_T<ad_scalar_t>;
using ad_quaternion_t = QUATERNION_T<ad_scalar_t>;

/******************************************************************************************************/
/* Contacts definition

  Contact wrench F = [f_x, f_y, f_z, M_x, M_y, M_z]^T

*/
/******************************************************************************************************/

constexpr size_t N_CONTACTS = 2;

template <typename T>
using feet_array_t = std::array<T, N_CONTACTS>;

template <typename T>
using feet_vec_t = std::vector<T>;

using contact_flag_t = feet_array_t<bool>;  // describes which feet are in contacts [left_contact, right_contact]

constexpr size_t CONTACT_WRENCH_DIM = 6;

}  // namespace ocs2
