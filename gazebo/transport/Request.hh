/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_TRANSPORT_REQUEST_HH_
#define GAZEBO_TRANSPORT_REQUEST_HH_

#include <string>

#include <ignition/math/Helpers.hh>
#include <ignition/math/Pose3.hh>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace transport
  {
    /// \brief Helper function to create entity delete requests using ignition
    /// transport.
    /// \param[in] _uri URI of entity to be deleted.
    GZ_TRANSPORT_VISIBLE
    size_t RequestEntityDelete(const std::string &_uri);

    /// \brief Helper class to create requests for ignition transport.
    GZ_TRANSPORT_VISIBLE
    size_t RequestEntityInsert(const std::string &_sdf,
        const ignition::math::Pose3d &_pose =
        ignition::math::Pose3d(IGN_DBL_MAX, IGN_DBL_MAX, IGN_DBL_MAX,
                               0, 0, 0));

    /// \brief Helper class to create requests for ignition transport.
    GZ_TRANSPORT_VISIBLE
    size_t RequestEntityClone(const std::string &_name,
        const ignition::math::Pose3d &_pose =
        ignition::math::Pose3d(IGN_DBL_MAX, IGN_DBL_MAX, IGN_DBL_MAX,
                               0, 0, 0));

    /// \brief Helper class to create requests for ignition transport.
    GZ_TRANSPORT_VISIBLE
    size_t RequestLightInsert(const msgs::Light &_light);
  }
}
#endif
