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
#ifndef GAZEBO_UTIL_IGNMSGSDF_HH_
#define GAZEBO_UTIL_IGNMSGSDF_HH_

#include <ignition/msgs/plugin.pb.h>
#include <sdf/sdf.hh>

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace util
  {
    /// \brief Create an ignition::msgs::Plugin from a plugin SDF element.
    /// \param[in] _sdf The sdf element.
    /// \return The new ignition::msgs::Plugin object.
    GAZEBO_VISIBLE
    ignition::msgs::Plugin PluginSdfToIgnMsg(const sdf::ElementPtr _sdf);

    /// \brief Create or update an SDF element from ignition::msgs::Plugin.
    /// \param[in] _msg Plugin messsage.
    /// \param[in] _sdf if supplied, performs an update from _msg instead of
    /// creating a new sdf element.
    /// \return The new SDF element.
    GAZEBO_VISIBLE
    sdf::ElementPtr PluginIgnMsgToSdf(const ignition::msgs::Plugin &_plugin,
        sdf::ElementPtr _sdf = sdf::ElementPtr());
  }
}
#endif
