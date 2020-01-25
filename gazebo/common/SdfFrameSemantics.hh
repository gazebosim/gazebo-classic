/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef GAZEBO_COMMON_SDFFRAMESEMANTICS_HH_
#define GAZEBO_COMMON_SDFFRAMESEMANTICS_HH_

#include <string>
#include <ignition/math/Pose3.hh>
#include <sdf/SemanticPose.hh>

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common
    /// \{
    /// \brief Helper function for resolving SDF poses with a SemanticPose
    /// object. If there are errors resolving the pose, return the raw pose
    /// instead.
    /// \param[in] _semPose SemanticPose object to resolve.
    /// \param[in] _resolveTo Name of frame to resolve to.
    /// \return Resolved pose.
    GZ_COMMON_VISIBLE ignition::math::Pose3d resolveSdfPose(
        const sdf::SemanticPose &_semPose, const std::string &_resolveTo="");

    /// \brief Resolve all the poses that use frame semantics and update
    /// _modelElem so that all poses are expressed in the sdf1.6 convention
    /// (i.e. relative to the poses default `relative_to` attribute).
    /// \param[in, out] _modelElem Model element that will have its poses
    /// resolved
    GZ_COMMON_VISIBLE void convertPosesToSdf16(
        const sdf::ElementPtr &_modelElem);
    /// \}
  }
}

#endif
