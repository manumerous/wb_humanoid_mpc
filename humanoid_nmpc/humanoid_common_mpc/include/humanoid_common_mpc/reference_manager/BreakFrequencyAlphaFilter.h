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

#include <humanoid_common_mpc/common/Types.h>
#include <cmath>

namespace ocs2::humanoid {

class BreakFrequencyAlphaFilter final {
 public:
  /**
   * Constructor
   *
   * @param [in] breakFrequency: Break frequency (cut-off frequency) in Hz.
   */
  BreakFrequencyAlphaFilter(scalar_t breakFrequency, const vector_t& y_init)
      : breakDeltaT_(1 / (2 * M_PI * breakFrequency)), y_last_(y_init) {
    lastTimeFilterCalled = std::chrono::steady_clock::now();
  };

  vector_t getFilteredVector(const vector_t& x) {
    assert(x.size() == y_last_.size());
    scalar_t alpha = computeAlpha();
    return (alpha * x + (1 - alpha) * y_last_);
  }

 private:
  scalar_t computeAlpha() {
    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> durationInSeconds = now - lastTimeFilterCalled;
    scalar_t delta_t = durationInSeconds.count();
    return (delta_t / (delta_t + breakDeltaT_));
  }

  scalar_t breakDeltaT_;
  vector_t y_last_;
  std::chrono::time_point<std::chrono::steady_clock> lastTimeFilterCalled;
};

}  // namespace ocs2::humanoid
