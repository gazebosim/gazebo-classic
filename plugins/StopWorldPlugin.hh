/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef GAZEBO_PLUGINS_STOPWORLDPLUGIN_HH_
#define GAZEBO_PLUGINS_STOPWORLDPLUGIN_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \brief This plugin will stop the world.
  class GAZEBO_VISIBLE StopWorldPlugin : public SystemPlugin
  {
    /// \brief Destructor
    public: virtual ~StopWorldPlugin();

    /// \brief Load the plugin.
    /// \param[in] _argc Number of command line arguments.
    /// \param[in] _argv Array of command line arguments.
    public: void Load(int _argc, char **_argv);

    /// \brief Initialize the plugin.
    private: void Init();

    /// \brief Callback for World Update events.
    private: void OnWorldUpdate();

    /// \brief Callback triggered when the world has been stopped.
    private: void OnWorldStopped();

    /// \brief The world connection.
    private: event::ConnectionPtr worldConn;

    /// \brief Pointer to the world.
    private: physics::WorldPtr world;
  };
}

#endif
