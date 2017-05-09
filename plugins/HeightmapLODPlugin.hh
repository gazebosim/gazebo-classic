/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#ifndef GAZEBO_PLUGINS_HEIGHTMAPLODPLUGIN_HH_
#define GAZEBO_PLUGINS_HEIGHTMAPLODPLUGIN_HH_

#include <memory>
#include <gazebo/common/Plugin.hh>

namespace gazebo
{
  // Forward declare private data class.
  class HeightmapLODPluginPrivate;

  /// \brief Plugin that sets the heightmap LOD
  class GAZEBO_VISIBLE HeightmapLODPlugin : public VisualPlugin
  {
    /// \brief Constructor.
    public: HeightmapLODPlugin();

    /// \brief Destructor.
    public: ~HeightmapLODPlugin() = default;

    // Documentation inherited
    public: virtual void Load(rendering::VisualPtr _visual,
        sdf::ElementPtr _sdf);

    /// \internal
    /// \brief Private data pointer
    private: std::unique_ptr<HeightmapLODPluginPrivate> dataPtr;
  };
}
#endif
