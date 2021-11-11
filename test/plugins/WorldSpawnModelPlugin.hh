/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

#ifndef GAZEBO_TEST_PLUGINS_WORLDSPAWNMODELPLUGIN_HH_
#define GAZEBO_TEST_PLUGINS_WORLDSPAWNMODELPLUGIN_HH_

#include <memory>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/World.hh>
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \brief Forward declarations
  class WorldSpawnModelPluginPrivate;

  /// \brief A plugin that spawns a model in the attached World.
  ///
  /// The plugin requires the following parameter:
  /// <sdf>     SDFormat description of model to load
  class GZ_PLUGIN_VISIBLE WorldSpawnModelPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: WorldSpawnModelPlugin();

    /// \brief Load the plugin.
    /// \param[in] _world Pointer to world
    /// \param[in] _sdf Pointer to the SDF configuration.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Pointer to private data.
    private: std::unique_ptr<WorldSpawnModelPluginPrivate> dataPtr;
  };
}
#endif
