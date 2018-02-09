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

#ifndef GAZEBO_PLUGINS_STATICMAPPLUGIN_HH_
#define GAZEBO_PLUGINS_STATICMAPPLUGIN_HH_

#include <string>
#include <memory>

#include <gazebo/common/Plugin.hh>
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \brief Forward declarations
  class StaticMapPluginPrivate;

  /// \brief A plugin that creates a model with textured map images.
  /// Map tiles are downloaded using Google Static Maps API:
  /// https://developers.google.com/maps/documentation/static-maps
  /// The map tiles are used as texture images to create a map model in the
  /// world.
  ///
  /// The plugin requires the following parameters:
  /// <center>     Center (latitude and longitude) of map
  /// <world_size> Target size of world to cover (in meters). The plugin will
  ///              fetch enough tiles to create a model that is larger than
  ///              the specified size.
  /// <api_key>    Google API key
  ///
  /// optional parameters:
  /// <model_name> Name of map model
  /// <pose>       Pose of map model in the world
  /// <zoom>       Zoom level from 0 (entire world) to 21+ (streets)
  /// <map_type>   Type of map to use: roadmap, satellite, terrain, hybrid
  /// <tile_size>  Size of map tiles in pixels. Max of 640 for standard usage
  ///              (only standard usage is supported). See Google Static Maps
  ///              API documentation for more details.
  /// <use_cache>  Use model in gazebo model path if exists, otherwise
  ///              recreate the model and save it in <HOME>/.gazebo/models
  class GAZEBO_VISIBLE StaticMapPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: StaticMapPlugin();

    /// \brief Load the plugin.
    /// \param[in] _world Pointer to world
    /// \param[in] _sdf Pointer to the SDF configuration.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    /// \brief Initialize the plugin.
    public: virtual void Init();

    /// \brief Pointer to private data.
    private: std::unique_ptr<StaticMapPluginPrivate> dataPtr;
  };
}
#endif
