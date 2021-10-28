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

  /// \brief Plugin that sets the heightmap Level of Detail (LOD) parameters.
  /// `lod`: a render-engine specific value used to compute Level of Detail.
  /// `skirt_length`: length of skirts on LOD tiles.
  /// These parameters can be set uniformly for all scenes in a simulation
  /// by specifying the parameters directly under the <plugin /> element:
  /** \verbatim
    <plugin filename="libHeightmapLODPlugin.so" name="heightmap_lod">
      <lod>5</lod>
      <skirt_length>0.5</skirt_length>
    </plugin>
   \endverbatim */
  /// Alternatively, you can specify distinct values for server scene used
  /// for rendering camera sensors, and the gui scene used for gzclient's
  /// graphical interface.
  /** \verbatim
    <plugin filename="libHeightmapLODPlugin.so" name="heightmap_lod">
      <gui>
        <lod>5</lod>
        <skirt_length>1.5</skirt_length>
      </gui>
      <server>
        <lod>0</lod>
        <skirt_length>0.5</skirt_length>
      </server>
    </plugin>
   \endverbatim */
  /// Parameters specified in the root namespace have precedence. The
  /// <server/> and <gui/> elements are only checked if parameters
  /// are not set in the root namespace.
  /** \verbatim
    <plugin filename="libHeightmapLODPlugin.so" name="heightmap_lod">
      <skirt_length>2.5</skirt_length>
      <gui>
        <lod>5</lod>
        <skirt_length>1.5</skirt_length> <!-- this is ignored -->
      </gui>
      <server>
        <lod>0</lod>
        <skirt_length>0.5</skirt_length> <!-- this is ignored -->
      </server>
    </plugin>
   \endverbatim */
  class GZ_PLUGIN_VISIBLE HeightmapLODPlugin : public VisualPlugin
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
