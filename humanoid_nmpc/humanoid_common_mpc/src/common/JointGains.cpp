/******************************************************************************
Copyright (c) 2025, Manuel Yves Galliker. All rights reserved.
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

#include "humanoid_common_mpc/common/JointGains.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <stdexcept>

namespace ocs2::humanoid {

namespace {

// Value of the first entry of table whose key is a substring of jointName, else of its "default" entry.
scalar_t lookupGain(const boost::property_tree::ptree& table, const std::string& tableName, const std::string& jointName) {
  for (const auto& [pattern, value] : table) {
    if (pattern != "default" && jointName.find(pattern) != std::string::npos) {
      return value.get_value<scalar_t>();
    }
  }
  if (const auto fallback = table.get_optional<scalar_t>("default")) {
    return *fallback;
  }
  throw std::runtime_error("jointGains." + tableName + " has no entry matching joint '" + jointName + "' and no 'default'.");
}

}  // namespace

std::vector<JointGains> loadJointGains(const std::string& file, const std::vector<std::string>& jointNames) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(file, pt);
  const auto& kpTable = pt.get_child("jointGains.kp");
  const auto& kdTable = pt.get_child("jointGains.kd");
  const auto dampingTable = pt.get_child_optional("jointGains.dampingCompensation");

  std::vector<JointGains> gains;
  gains.reserve(jointNames.size());
  for (const auto& jointName : jointNames) {
    const scalar_t damping = dampingTable ? lookupGain(*dampingTable, "dampingCompensation", jointName) : 0.0;
    gains.push_back({lookupGain(kpTable, "kp", jointName), lookupGain(kdTable, "kd", jointName), damping});
  }
  return gains;
}

}  // namespace ocs2::humanoid
