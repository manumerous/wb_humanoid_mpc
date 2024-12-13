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

#include "humanoid_common_mpc/contact/ContactCenterPoint.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>
#include "humanoid_common_mpc/common/ModelSettings.h"

namespace ocs2::humanoid {

ContactCenterPoint ContactCenterPoint::loadContactCenterPoint(const std::string& taskFile,
                                                              const ModelSettings& modelSettings,
                                                              int contactIndex,
                                                              bool verbose) {
  assert(contactIndex < N_CONTACTS && "Contact index is out of bound!");
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "contacts.";

  scalar_t x, y, z;
  if (verbose) {
    std::cerr << "\n #### Contact Center Point Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, x, prefix + "contact_frame_translation.x", verbose);
  loadData::loadPtreeValue(pt, y, prefix + "contact_frame_translation.y", verbose);
  loadData::loadPtreeValue(pt, z, prefix + "contact_frame_translation.z", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================\n";
  }

  vector3_t translationFromParent;
  translationFromParent << x, y, z;
  std::string frameName = modelSettings.contactNames[contactIndex];
  std::string parentJointName = modelSettings.contactParentJointNames[contactIndex];
  return ContactCenterPoint(frameName, parentJointName, translationFromParent);
}

}  // namespace ocs2::humanoid