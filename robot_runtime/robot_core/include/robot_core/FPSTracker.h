/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.

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

#include <assert.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>

namespace robot {

class FPSTracker {
 public:
  explicit FPSTracker(const std::string& name, double alpha = 0.1)
      : initialized_(false), sampleCount_(0), alpha_(alpha), fps_(0.0), name_(name) {
    assert(alpha > 0 && alpha <= 1);
    lastTimePoint_ = std::chrono::steady_clock::now();
  }

  ~FPSTracker() = default;

  void tick() {
    auto now = std::chrono::steady_clock::now();
    double deltaTime = std::chrono::duration<double>(now - lastTimePoint_).count();
    lastTimePoint_ = now;

    ++sampleCount_;
    double currentFPS = 1.0 / deltaTime;

    if (initialized_) {
      fps_ = alpha_ * currentFPS + (1.0 - alpha_) * fps_;
    } else {
      fps_ = currentFPS;
      initialized_ = true;
    }
  }

  void reset() { initialized_ = false; }

  void print() const { std::cerr << "FPS [" << name_ << "]: " << static_cast<int>(fps_) << std::endl; }

  double alpha() const { return alpha_; }

  double fps() const { return fps_; }

  size_t sampleCount() const { return sampleCount_; }

 private:
  bool initialized_;
  size_t sampleCount_;

  double alpha_;  // Smoothing factor (0 < alpha <= 1)
  double fps_;

  std::chrono::time_point<std::chrono::steady_clock> lastTimePoint_;

  std::string name_;
};
}  // namespace robot
