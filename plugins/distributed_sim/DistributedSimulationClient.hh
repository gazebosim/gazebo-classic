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

#ifndef GAZEBO_PLUGINS_DISTRIBUTEDSIMULATIONCLIENT_HH_
#define GAZEBO_PLUGINS_DISTRIBUTEDSIMULATIONCLIENT_HH_

#include "gazebo/common/Plugin.hh"
#include "gazebo/util/system.hh"

#include <ignition/msgs.hh>
#include <ignition/transport.hh>

namespace gazebo
{
  class DistributedSimulationPrivate;

  /// \brief This plugin will stop the world.
  class GAZEBO_VISIBLE DistributedSimulationClient: public SystemPlugin
  {
    /// \brief Destructor
    public: virtual ~DistributedSimulationClient ();

    /// \brief Load the plugin.
    /// \param[in] _argc Number of command line arguments.
    /// \param[in] _argv Array of command line arguments.
    public: virtual void Load(int _argc, char **_argv);

    /// \brief Initialize the plugin.
    private: void Init();

    private: std::unique_ptr<DistributedSimulationPrivate> dataPtr;
  };
}

#endif




